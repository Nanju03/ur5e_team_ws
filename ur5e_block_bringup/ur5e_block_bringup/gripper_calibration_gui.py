"""Tk client for the sensor-backed AG95 calibration state machine.

The backend owns ROS actions, hardware checks, CSVs and the recommendation.
Opening this window does not start a calibration or initialize hardware.
"""
import argparse
import copy
import math
from pathlib import Path
import signal
import time
import tkinter as tk
from tkinter import messagebox, ttk


class GripperCalibrationWindow:
    """A single calibration session whose backend must confirm safe closure."""

    MAX_FEEDBACK_AGE = .5

    def __init__(self, master, calibration_file, physical_profile, *, on_close,
                 invalidate_plan, on_status=None, backend_factory=None, gripper_only=False,
                 execution_mode='step'):
        if not isinstance(gripper_only, bool):
            raise ValueError("그리퍼 단독 모드는 True/False로 지정하세요.")
        self.gripper_only = gripper_only
        if execution_mode not in ('step', 'continuous'):
            raise ValueError('execution_mode must be step or continuous')
        self.execution_mode = execution_mode
        if backend_factory is None:
            from ur5e_block_bringup.gripper_calibration import CalibrationBackend
            backend_factory = CalibrationBackend
        self.backend = None
        self.on_close = on_close
        self.invalidate_plan = invalidate_plan
        self.on_status = on_status or (lambda message: None)
        self.state = {}
        self.closed = False
        self.close_requested = False
        self.confirming = False
        self.command_pending = False
        self.stop_pending = False
        self.poll_failed = False
        self.configuration_loaded = False
        self.last_error = ""
        self.window = tk.Toplevel(master)
        self.window.title("AG95 " + ("단독 · " if self.gripper_only else "· ") + "25mm 파지 보정")
        self.window.geometry("780x810")
        self.window.minsize(680, 720)
        self.window.transient(master)
        # Keep the main window's always-visible Stop available; no grab_set().
        self.window.protocol("WM_DELETE_WINDOW", self.request_close)
        self.open_q = tk.StringVar(self.window, value="0.50")
        self.max_q = tk.StringVar(self.window, value="0.78")
        self.step_q = tk.StringVar(self.window, value="0.01")
        self.force_percent = tk.StringVar(self.window, value="20")
        self.settle_s = tk.StringVar(self.window, value="0.4")
        self.feedback_text = tk.StringVar(self.window, value="센서 상태를 확인 중입니다.")
        self.readiness_text = tk.StringVar(self.window)
        self.phase_text = tk.StringVar(self.window, value="확인 중 · 아직 명령을 보내지 않았습니다.")
        self.csv_text = tk.StringVar(self.window, value="기록 파일: 아직 없음")
        self.recommendation_text = tk.StringVar(self.window, value="추천값: 아직 없음")
        self.notice = tk.StringVar(self.window, value=(
            ("그리퍼 단독 모드 · AG95를 단단히 고정하고 블록 받침을 준비하세요. 초기화 명령은 보내지 않습니다.\n"
             if self.gripper_only else
             "팔을 수동으로 맞춘 뒤 사용하세요. 이 도구는 팔 이동·그리퍼 초기화 명령을 보내지 않습니다.\n") +
            "1. 블록 없는 기준 측정 → 2. 받침 위 25mm 블록 잡기 → 3. 실제 결과 기록 → 4. 받침 위에서 열기\n"
            "잡힘 뒤에는 그대로 유지합니다. 자동으로 열거나 블록을 들어 올리지 않습니다."))
        self.entries = []
        self.motion_buttons = {}
        try:
            self._build()
            # Do not acquire a backend Node/session lock until Tk has built the
            # whole window successfully. No physical command is issued here.
            options = {"allow_mock": False, "execution_mode": self.execution_mode}
            if self.gripper_only:
                options["gripper_only"] = True
            self.backend = backend_factory(calibration_file, physical_profile, **options)
        except Exception:
            self.window.destroy()
            raise
        self.invalidate_plan()
        self.window.after(0, self.poll)

    def _build(self):
        outer = ttk.Frame(self.window, padding=14)
        outer.pack(fill="both", expand=True)
        ttk.Label(outer, text="25mm 블록 · 그리퍼 각도 / 센서 보정" + (" · 단독 모드" if self.gripper_only else ""),
                  font=("Sans", 15, "bold")).pack(anchor="w")
        top = ttk.Frame(outer)
        top.pack(fill="x", pady=(8, 6))
        self.stop_button = ttk.Button(top, text="그리퍼 정지 요청", command=self.stop)
        self.stop_button.pack(side="right", padx=(8, 0))
        self.approve_button = ttk.Button(top, text="미리보기 OK · 한 단계 실행",
                                        command=self.approve_next, state="disabled")
        self.approve_button.pack(side="right", padx=(8, 0))
        ttk.Label(top, textvariable=self.phase_text, wraplength=310,
                  font=("Sans", 10, "bold")).pack(side="left", fill="x", expand=True)
        ttk.Label(outer, textvariable=self.feedback_text, wraplength=730,
                  font=("Monospace", 10)).pack(fill="x", pady=3)
        ttk.Label(outer, textvariable=self.readiness_text, wraplength=730).pack(fill="x", pady=3)
        ttk.Label(outer, text="gap은 로봇 모델로 계산한 추정 간격입니다. 힘(%)은 설정값이며 실측 힘이 아닙니다.",
                  wraplength=730).pack(fill="x", pady=(0, 8))
        settings = ttk.LabelFrame(outer, text="보정 범위 · 각도가 커질수록 닫힘", padding=8)
        settings.pack(fill="x")
        rows = (("시작 열림각 (rad)", self.open_q), ("최대 닫힘각 (rad) · 최대 0.85", self.max_q),
                ("닫기 간격 (rad) · 0.001~0.03", self.step_q), ("힘 설정 (%) · 20~100", self.force_percent),
                ("각 단계 대기 (s) · 0.3~2", self.settle_s))
        for row, (label, variable) in enumerate(rows):
            ttk.Label(settings, text=label).grid(row=row, column=0, sticky="w", padx=(0, 12), pady=2)
            entry = ttk.Entry(settings, textvariable=variable, width=12)
            entry.grid(row=row, column=1, sticky="w", pady=2)
            self.entries.append(entry)
            if variable is self.force_percent:
                self.force_entry = entry
                entry.configure(state="disabled")
        ttk.Label(settings, text="블록 폭: 25mm · 힘 지원 확인 전에는 힘 입력을 잠급니다.",
                  wraplength=680).grid(row=len(rows), column=0, columnspan=2, sticky="w", pady=(5, 0))
        actions = ttk.Frame(outer)
        actions.pack(fill="x", pady=8)
        actions.columnconfigure(0, weight=1)
        actions.columnconfigure(1, weight=1)
        for row, items in enumerate((
                (("① 빈 공간 기준 측정(닫기 → 열기)", "baseline"),
                 ("② 25mm 블록 1회 단계 잡기 · 유지", "grasp")),
                (("한 단계 닫기", "step_close"), ("받침 위에서 기준 열림각으로 열기", "open")))):
            for column, (label, mode) in enumerate(items):
                button = ttk.Button(actions, text=label, command=lambda mode=mode: self.start(mode))
                button.grid(row=row, column=column, sticky="ew", padx=3, pady=3)
                self.motion_buttons[mode] = button
        results = ttk.Frame(outer)
        results.pack(fill="x")
        self.success_button = ttk.Button(results, text="③ 실제 잡힘 성공 확인 / 기록",
                                         command=lambda: self.record_result(True))
        self.success_button.pack(side="left", fill="x", expand=True, padx=(0, 4))
        self.failure_button = ttk.Button(results, text="잡힘 실패 확인 / 기록",
                                         command=lambda: self.record_result(False))
        self.failure_button.pack(side="left", fill="x", expand=True)
        ttk.Label(outer, textvariable=self.notice, wraplength=730, justify="left").pack(fill="x", pady=8)
        recording = ttk.LabelFrame(outer, text="CSV와 추천값", padding=8)
        recording.pack(fill="x")
        ttk.Label(recording, textvariable=self.csv_text, wraplength=700).pack(fill="x", pady=2)
        ttk.Label(recording, textvariable=self.recommendation_text, wraplength=700).pack(fill="x", pady=3)
        ttk.Label(recording, text="빈 공간 기준 측정 후, 잡기 성공과 놓기를 3회 확인하면 추천값이 표시됩니다.",
                  wraplength=700).pack(fill="x", pady=2)
        self.apply_button = ttk.Button(recording, text="④ 추천값 확인 후 보정 프로파일에 적용",
                                       command=self.apply_recommendation)
        self.apply_button.pack(fill="x", pady=(5, 0))
        self.close_button = ttk.Button(outer, text="보정 창 닫기 · 정지 확인 후 종료", command=self.request_close)
        self.close_button.pack(side="bottom", fill="x", pady=(8, 0))
        self._render()

    @staticmethod
    def _number(value, digits=3):
        try:
            number = float(value)
            return f"{number:.{digits}f}" if math.isfinite(number) else "없음"
        except (ValueError, TypeError):
            return "없음"

    def _feedback_fresh(self):
        try:
            age = float(self.state.get("age", math.inf))
            return (self.state.get("feedback_valid") is True and math.isfinite(age)
                    and 0 <= age <= self.MAX_FEEDBACK_AGE)
        except (TypeError, ValueError):
            return False

    def _ready_for_motion(self):
        return (not self.closed and not self.close_requested and not self.confirming
                and not self.command_pending and not self.stop_pending and not self.poll_failed
                and self.state.get("ready") is True and not self.state.get("busy", True)
                and self.state.get("controller_active") is True
                and self._arm_condition_met() and self._feedback_fresh())

    def _can_approve(self):
        preview = self.state.get('pending_approval')
        return (self.execution_mode == 'step' and isinstance(preview, dict)
                and isinstance(preview.get('request_id'), str) and bool(preview['request_id'])
                and not self.closed and not self.close_requested and not self.confirming
                and not self.command_pending and not self.stop_pending and not self.poll_failed
                and self.state.get('phase') == 'WAIT_APPROVAL' and self.state.get('busy') is True
                and self.state.get('controller_active') is True
                and self._arm_condition_met() and self._feedback_fresh())

    def approve_next(self):
        if not self._can_approve():
            return
        preview = copy.deepcopy(self.state['pending_approval'])
        self.confirming = True
        self._render()
        try:
            accepted = messagebox.askokcancel(
                '그리퍼 다음 단계 확인',
                f"단계 {preview['index']}/{preview['total']} · {preview['operation']}\n"
                f"각도 {self._number(preview['start_q'])} → {self._number(preview['target_q'])}rad\n"
                f"모델 간격 {self._number(preview['start_gap_mm'],1)} → {self._number(preview['target_gap_mm'],1)}mm (추정)\n\n"
                '이 미리보기는 팔의 이동 경로가 아닌 그리퍼 개폐 각도와 추정 간격입니다.\n'
                '표시한 한 단계만 실행하고 다음 OK를 기다립니다. 실행하시겠습니까?', parent=self.window)
        finally:
            self.confirming = False
        if accepted and not self.close_requested and not self.closed:
            try:
                self._read_state()
                if not self._can_approve() or self.state.get('pending_approval') != preview:
                    raise RuntimeError('확인 중 그리퍼 미리보기나 연결 상태가 바뀌었습니다.')
                self.command_pending = True
                self.backend.approve_next(preview['request_id'])
                self.last_error = ''
                self._read_state()
            except Exception as error:
                self._error(error)
            finally:
                self.command_pending = False
        self._render()

    def _arm_condition_met(self):
        if self.state.get("arm_required", True) is False:
            return self.gripper_only and self.state.get("gripper_only") is True
        return not self.gripper_only and self.state.get("arm_stationary") is True

    def _idle(self):
        return (not self.closed and not self.close_requested and not self.confirming
                and not self.command_pending and not self.stop_pending and not self.poll_failed
                and not self.state.get("busy", True))

    def _can_record(self):
        return self._idle() and self.state.get("can_record") is True and self._arm_condition_met()

    def _configuration(self):
        values = {"open_q": float(self.open_q.get()), "max_q": float(self.max_q.get()),
                  "step_q": float(self.step_q.get()), "force_percent": float(self.force_percent.get()),
                  "settle_s": float(self.settle_s.get()), "width_mm": 25.0}
        if not all(math.isfinite(value) for value in values.values()):
            raise ValueError("설정값은 유한한 숫자로 입력하세요.")
        if not 0 <= values["open_q"] < values["max_q"] <= .85:
            raise ValueError("각도는 0~0.85rad이며 시작 열림각이 최대 닫힘각보다 작아야 합니다.")
        if not .001 <= values["step_q"] <= .03:
            raise ValueError("닫기 간격은 0.001~0.03rad로 입력하세요.")
        if not 20 <= values["force_percent"] <= 100 or not values["force_percent"].is_integer():
            raise ValueError("힘 설정은 20~100% 정수로 입력하세요.")
        if not .3 <= values["settle_s"] <= 2:
            raise ValueError("각 단계 대기는 0.3~2초로 입력하세요.")
        values["force_percent"] = int(values["force_percent"])
        return values

    def _read_state(self):
        self.poll_failed = True
        state = self.backend.poll()
        if not isinstance(state, dict):
            raise ValueError("보정 상태 정보를 읽을 수 없습니다.")
        self.state = state
        self.poll_failed = False
        if not self.configuration_loaded and isinstance(state.get("configuration_values"), dict):
            variables = {"open_q": self.open_q, "max_q": self.max_q, "step_q": self.step_q,
                         "force_percent": self.force_percent, "settle_s": self.settle_s}
            for key, variable in variables.items():
                if key in state["configuration_values"]:
                    variable.set(str(state["configuration_values"][key]))
            self.configuration_loaded = True
        if self.stop_pending and state.get("stop_confirmed") is True and not state.get("busy", True):
            self.stop_pending = False
        return state

    def _error(self, error):
        message = str(error)
        changed = self.last_error != message
        self.last_error = message
        self.phase_text.set("보정 오류: " + self.last_error)
        if changed:
            self.on_status("그리퍼 보정: " + self.last_error)

    def start(self, mode):
        if mode not in self.motion_buttons or not self._ready_for_motion():
            return
        try:
            configuration = self._configuration()
        except (ValueError, TypeError) as error:
            self._error(error)
            return
        descriptions = {
            "baseline": "블록이 없는 빈 공간에서 그리퍼를 단계적으로 닫은 뒤 기준 열림각으로 엽니다.",
            "grasp": "받침 위 25mm 블록을 단계적으로 잡고, 센서가 잡힘을 알리면 그 위치를 유지합니다. 자동으로 열지 않습니다.",
            "step_close": "받침 위 블록을 향해 그리퍼를 한 단계만 닫고 그 위치를 유지합니다.",
            "open": "블록을 받침에 안전하게 놓은 상태에서 그리퍼를 기준 열림각으로 엽니다.",
        }
        setup_instruction = ("블록을 완전히 빼고 그리퍼가 닫힐 공간에 닿는 물체가 없는지 확인하세요.\n"
                             if mode == "baseline" else
                             "블록이 안정적인 받침 위에 있으며 손가락을 열어도 떨어지지 않는지 확인하세요.\n")
        device_instruction = ("그리퍼를 단단히 고정하고 비상 전원 차단 수단을 준비하세요.\n"
                              if self.gripper_only else
                              "팔 이동 명령은 보내지 않습니다. 팔이 멈춰 있는지 확인하세요.\n"
                              "비상정지 버튼을 사용할 준비가 되었는지 확인하세요.\n")
        execution_instruction = (
            '단계별 확인: 각 개폐 목표의 각도·추정 간격을 먼저 표시하고, 매번 OK를 눌러야 움직입니다.\n'
            if self.execution_mode == 'step' else
            '전체 연속 실행: 위 작업 순서와 각도 범위를 한 번 확인하면 개폐 단계를 자동으로 진행합니다.\n')
        self.confirming = True
        self._render()
        try:
            accepted = messagebox.askokcancel(
                "실제 AG95 그리퍼 동작 확인",
                descriptions[mode] + "\n\n" + execution_instruction + device_instruction +
                f"열림 {configuration['open_q']:.3f}rad / 최대 닫힘 {configuration['max_q']:.3f}rad / "
                f"간격 {configuration['step_q']:.3f}rad\n"
                f"힘 설정 {configuration['force_percent']}% (실측 힘 아님)\n"
                + setup_instruction + "그리퍼 사이와 주변에 손이 없는지 확인하세요.\n"
                "빈 공간 기준 측정은 블록을 빼고, 블록 잡기는 25mm 블록을 넣은 상태여야 합니다.\n\n"
                "이 조건을 확인하고 실제 그리퍼를 움직이시겠습니까?", parent=self.window)
        finally:
            self.confirming = False
        if not accepted or self.close_requested or self.closed:
            self._render()
            return
        try:
            # Tk's modal loop continues processing status updates. Recheck the
            # backend after confirmation rather than relying on its old state.
            self._read_state()
            if not self._ready_for_motion():
                raise RuntimeError("확인 중 연결이나 정지 상태가 바뀌었습니다. 센서 상태를 다시 확인하세요.")
            if self._configuration() != configuration:
                raise RuntimeError("확인 중 보정 범위가 바뀌었습니다. 설정값을 다시 확인하세요.")
            self.backend.configure(**configuration)
            self.command_pending = True
            self.invalidate_plan()
            self.backend.start(mode, workspace_clear=True)
            self.last_error = ""
            self._read_state()
        except Exception as error:
            self.poll_failed = True
            self._error(error)
        finally:
            self.command_pending = False
            self._render()

    def record_result(self, success):
        if not self._can_record():
            return
        self.confirming = True
        self._render()
        try:
            accepted = messagebox.askyesno(
                "실제 파지 결과 기록",
                ("25mm 블록이 양쪽 패드 사이에 안정적으로 잡혔음을 직접 확인했습니까?" if success else
                 "이번 25mm 블록 파지를 실패로 기록하시겠습니까?") +
                "\n\n이 버튼은 결과만 CSV에 기록하며 움직임 명령을 보내지 않습니다.\n"
                "기록 후에는 블록을 받침에 놓은 상태에서 별도로 열기를 누르세요.", parent=self.window)
        finally:
            self.confirming = False
        if accepted and not self.close_requested and not self.closed:
            try:
                self._read_state()
                if not self._can_record():
                    raise RuntimeError("현재 단계에서는 파지 결과를 기록할 수 없습니다.")
                self.backend.record_result(success=bool(success))
                self.last_error = ""
                self._read_state()
            except Exception as error:
                self._error(error)
        self._render()

    def apply_recommendation(self):
        recommendation = copy.deepcopy(self.state.get("recommendation"))
        if not self._idle() or not isinstance(recommendation, dict):
            return
        self.confirming = True
        self._render()
        try:
            accepted = messagebox.askokcancel(
                "그리퍼 추천값 적용",
                self._recommendation_description(recommendation) +
                "\n\n확인한 추천값을 보정 프로파일에 저장합니다. 이 버튼은 그리퍼를 움직이지 않습니다.\n"
                + ("적용하시겠습니까?" if self.gripper_only else
                   "기존 팔 계획은 사용할 수 없으며 다시 계획해야 합니다. 적용하시겠습니까?"), parent=self.window)
        finally:
            self.confirming = False
        if accepted and not self.close_requested and not self.closed:
            try:
                self._read_state()
                if not self._idle() or self.state.get("recommendation") != recommendation:
                    raise RuntimeError("확인 중 추천값이나 작업 상태가 바뀌었습니다. 다시 확인하세요.")
                path = self.backend.apply_recommendation()
                self.invalidate_plan()
                self.last_error = ""
                self.notice.set(f"추천값을 보정 프로파일에 저장했습니다: {path}\n"
                                + ("25mm 블록의 파지·놓기 확인 기록을 함께 보관하세요." if self.gripper_only else
                                   "실제 전체 블록 쌓기는 아직 검증되지 않았습니다. 팔 경로를 다시 계획하세요."))
                self.on_status(f"그리퍼 보정 추천값 저장: {path}" +
                               ("" if self.gripper_only else " · 기존 팔 계획 무효화"))
                self._read_state()
            except Exception as error:
                self._error(error)
        self._render()

    def stop(self):
        if self.closed:
            return
        try:
            self.stop_pending = True
            self.backend.request_stop()
            self.invalidate_plan()
            self.phase_text.set("그리퍼 중지 요청 · 제어기와 센서의 정지 확인을 기다립니다.")
            self.on_status("그리퍼 중지 요청 중 · 정지 확인 전까지 보정 창을 유지합니다.")
        except Exception as error:
            self._error(error)
        self._render()

    def request_close(self):
        if self.closed:
            return
        self.close_requested = True
        self.stop()
        self.phase_text.set("창 닫기 대기 · 그리퍼 정지를 확인할 때까지 상태를 계속 수신합니다.")
        self._render()

    def focus(self):
        if not self.closed:
            self.window.lift()
            self.window.focus_set()

    def _recommendation_description(self, recommendation):
        trials = recommendation.get("trials", "없음")
        if isinstance(trials, (list, tuple)):
            trials = len(trials)
        return (f"추천 명령각 {self._number(recommendation.get('commanded_q'))}rad / "
                f"센서각 {self._number(recommendation.get('measured_q'))}rad\n"
                f"힘 설정 {self._number(recommendation.get('force_percent'), 0)}% (실측 힘 아님) / "
                f"검증 기록 {trials}회")

    def _render(self):
        if self.closed:
            return
        state = self.state
        phase = str(state.get("phase", "센서 확인 중"))
        phase = {"IDLE":"시험 대기", "BASELINE":"빈 상태 기준 측정", "GRASP":"단계별 잡기",
                 "OPEN":"그리퍼 열기", "STEP_CLOSE":"한 단계 닫기", "HOLD":"잡힘 유지",
                 "WAIT_APPROVAL":"다음 개폐 미리보기 · OK 대기",
                 "CANCELLING":"그리퍼 정지 확인 중", "ERROR":"시험 중지", "CLOSED":"종료"}.get(phase,phase)
        error = str(state.get("error") or self.last_error or "")
        if self.close_requested:
            self.phase_text.set("닫기 대기 · 그리퍼 정지 확인 중" + (f" · {error}" if error else ""))
        elif self.stop_pending:
            self.phase_text.set("그리퍼 중지 요청 · 제어기와 센서 확인 대기" + (f" · {error}" if error else ""))
        elif error:
            self.phase_text.set(f"{phase} · {error}")
        else:
            self.phase_text.set(phase + (" · 진행 중" if state.get("busy") else ""))
        valid = "최신 센서 수신" if self._feedback_fresh() else "센서 없음 / 오래됨"
        self.feedback_text.set(
            f"센서 각도: {self._number(state.get('q_rad'))}rad   위치 원시값: {self._number(state.get('raw_position'),0)}   "
            f"모델 gap(추정): {self._number(state.get('model_gap_mm'), 1)}mm\n"
            f"잡힘 센서: {state.get('grip_label') or state.get('grip_state', '없음')}   "
            f"힘 설정: {self._number(state.get('force_percent'), 0)}% (실측 힘 아님)\n"
            f"{valid} · 수신 경과 {self._number(state.get('age'), 1)}s")
        preview = state.get('pending_approval')
        if isinstance(preview, dict):
            self.feedback_text.set(self.feedback_text.get()+
                f"\n다음 {preview.get('index')}/{preview.get('total')}: "
                f"{self._number(preview.get('start_q'))} → {self._number(preview.get('target_q'))}rad · "
                f"간격 {self._number(preview.get('start_gap_mm'),1)} → {self._number(preview.get('target_gap_mm'),1)}mm (모델 추정)")
        arm_status = ("그리퍼 단독 모드 · 팔 피드백 불필요" if self.gripper_only else
                      "팔 정지 확인: " + ("예" if state.get("arm_stationary") is True else "아니오"))
        self.readiness_text.set(arm_status +
            "  |  그리퍼 제어기: " + ("active" if state.get("controller_active") is True else "미연결") +
            "  |  힘 설정 지원: " + ("확인" if state.get("force_supported") is True else "확인 전 · 입력 잠금")+
            "  |  실행: " + ('단계별 확인' if self.execution_mode == 'step' else '전체 연속'))
        self.csv_text.set("CSV 기록: " + str(state.get("csv_path") or "아직 없음"))
        recommendation = state.get("recommendation")
        self.recommendation_text.set(self._recommendation_description(recommendation)
                                     if isinstance(recommendation, dict) else "추천값: 아직 없음")
        editable = self._idle()
        for entry in self.entries:
            enabled = editable and (entry is not self.force_entry or state.get("force_supported") is True)
            entry.configure(state="normal" if enabled else "disabled")
        ready = self._ready_for_motion()
        for button in self.motion_buttons.values():
            button.configure(state="normal" if ready else "disabled")
        for button in (self.success_button, self.failure_button):
            button.configure(state="normal" if self._can_record() else "disabled")
        self.apply_button.configure(state="normal" if editable and isinstance(recommendation, dict) else "disabled")
        # Stop remains available even if telemetry or a backend response fails.
        self.stop_button.configure(state="normal")
        self.approve_button.configure(state='normal' if self._can_approve() else 'disabled')
        self.close_button.configure(state="disabled" if self.close_requested else "normal")

    def poll(self):
        if self.closed:
            return
        try:
            self._read_state()
            self._render()
            if self.close_requested and not self.confirming and self.state.get("can_close") is True:
                # close() is authoritative: process exit, a zero velocity field
                # or a stale pre-cancel snapshot never proves the goal stopped.
                if self.backend.close() is True:
                    self.closed = True
                    self.window.destroy()
                    self.on_close()
                    return
        except Exception as error:
            self.poll_failed = True
            self._error(error)
            self._render()
        self.window.after(100, self.poll)


def main(args=None):
    """Standalone Tk entry point; keep ROS alive until the backend can close."""
    parser = argparse.ArgumentParser(description="AG95 25mm 블록 파지 보정 GUI")
    parser.add_argument("--gripper-only", action="store_true", help="팔 없이 고정된 AG95만 보정")
    parser.add_argument('--execution-mode', choices=['step', 'continuous'], default='step')
    parser.add_argument("--calibration-file", default=str(Path.home()/"ur5e_ws/robot_calibration/ur5e_20261031.yaml"))
    parser.add_argument("--physical-profile", default=str(Path.home()/"ur5e_ws/physical_setup.yaml"))
    options, ros_args = parser.parse_known_args(args)

    # Embedded StackGui never calls main(); its ROS context remains its own.
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    root = None
    window = None
    previous_handlers = {}
    try:
        root = tk.Tk()
        root.withdraw()
        window = GripperCalibrationWindow(
            root, str(Path(options.calibration_file).expanduser()), str(Path(options.physical_profile).expanduser()),
            on_close=root.destroy, invalidate_plan=lambda: None, gripper_only=options.gripper_only,
            execution_mode=options.execution_mode)
        # A transient child of a hidden root may itself remain withdrawn.
        window.window.transient("")
        window.window.deiconify()
        def request_close(*_):
            if not window.closed:
                root.after(0, window.request_close)
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, request_close)
        while not window.closed:
            root.mainloop()
            if not window.closed:
                window.request_close()
    finally:
        if window is not None and not window.closed:
            try:
                window.request_close()
            except Exception:
                pass  # A lost Tk window must not tear down a live ROS goal.
            # An interrupted Tk loop still owns its remote goal. Keep polling
            # before releasing the ROS context, even if the display was lost.
            while not window.closed:
                try:
                    root.update()
                except tk.TclError:
                    pass
                if not window.closed:
                    try:
                        window.closed = window.backend.close() is True
                    except Exception:
                        pass
                    if window.closed:
                        try:
                            root.destroy()
                        except tk.TclError:
                            pass
                if not window.closed:
                    time.sleep(.1)
        for signum, previous in previous_handlers.items():
            signal.signal(signum, previous)
        # Constructor failures occur before any motion. Normal closure reaches
        # here only after close() confirms no outstanding gripper goal remains.
        if root is not None and (window is None or not window.closed):
            root.destroy()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
