#!/usr/bin/env python3
"""A3 task client for a separately started simulation or real robot backend."""
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import re
import signal
import subprocess
import threading
import time
import uuid
import tkinter as tk
from tkinter import messagebox, ttk

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import Trigger
from moveit_msgs.srv import GetPlanningScene
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray
from ur5e_block_bringup.plate_geometry import ALIGNMENT_GRIPPER_POSITION, ALIGNMENT_POLICY
from ur5e_block_bringup.path_archive import archive_from_log
from ur5e_block_bringup.physical_motion import ARM_SPEEDS, APPROACH_CLEARANCE, CONTACT_CLEARANCE, motion_parameters
from ur5e_block_bringup.physical_motion import (FAST_LAYERS, PICKUP_SPEED, LOW_TOWER_SPEED,
    HIGH_TOWER_SPEED, incremental_motion_parameters)
from ur5e_block_bringup.incremental_state import POLICY as INCREMENTAL_POLICY
from ur5e_block_bringup.physical_held import HOME_PRESERVE_POLICY, EMPTY_UNLOCK_POLICY
import yaml


class StackGui:
    COLORS = {"block": "#3489c7", "obstacle": "#d94c45", "target": "#e3b33e", "empty": "#d7dfe6"}
    ARM_JOINTS = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}
    HAND_JOINT = "left_outer_knuckle_joint"
    ALIGNMENT_HAND = ALIGNMENT_GRIPPER_POSITION
    PLACEMENT_LABELS = {"side": "19층까지 수직 / 20층부터 측면", "vertical": "모든 높이에서 수직"}
    EXECUTION_LABELS = {"step": "단계별 확인", "continuous": "전체 연속 실행"}
    MOTION_ACTIONS = ("run", "alignment_gripper", "recovery")

    def __init__(self, root):
        self.root = root
        self.node = rclpy.create_node("ur5e_stack_gui_client")
        self.backend = self.node.declare_parameter("backend", "simulation").value
        if self.backend not in ("simulation", "real"):
            raise ValueError("backend must be simulation or real")
        self.real_backend = self.backend == "real"
        self.simulation_backend = self.node.declare_parameter("simulation_backend", "mock").value
        if self.simulation_backend not in ("mock", "gazebo") or self.real_backend and self.simulation_backend != "mock":
            raise ValueError("Gazebo is supported only as an explicit simulation backend")
        simulation_riser = self.node.declare_parameter("simulation_riser_height", 0.0).value
        self.simulation_riser_height = 0.0
        if not self.real_backend:
            self.simulation_riser_height = self.checked_simulation_riser_height(simulation_riser)
        self.calibration_file = str(Path(self.node.declare_parameter(
            "calibration_file", str(Path.home()/"ur5e_ws/robot_calibration/ur5e_20261031.yaml")
        ).value).expanduser())
        self.physical_profile = str(Path(self.node.declare_parameter(
            "physical_profile", str(Path.home()/"ur5e_ws/physical_setup.yaml")
        ).value).expanduser())
        self.scene_client = self.node.create_client(GetPlanningScene, "/get_planning_scene")
        self.stop_client = self.node.create_client(Trigger, "/ur5e_a3_stack/stop")
        managers = ["/controller_manager"]
        if self.real_backend:
            managers.append("/gripper/controller_manager")
        self.managers = {
            name: {"client": self.node.create_client(ListControllers, name+"/list_controllers"),
                   "future": None, "checked": 0.0, "requested": 0.0, "states": {}}
            for name in managers
        }
        self.joint_values, self.joint_times = {}, {}
        self.joint_velocities, self.velocity_times = {}, {}
        self.feedback_subscription = self.node.create_subscription(
            JointState, "/joint_states", self.joint_feedback, qos_profile_sensor_data)
        self.marker_pub = self.node.create_publisher(MarkerArray, "/ur5e_a3_stack/markers", 1)
        self.share = Path(get_package_share_directory("ur5e_a3_grid"))
        self.common = yaml.safe_load((self.share / "config/a3_grid.yaml").read_text())["/**"]["ros__parameters"]
        self.plate_enabled = self.common.get("plate_enabled", False) is True
        self.plate_defaults = {}
        if self.plate_enabled:
            self.plate_defaults = self.plate_parameters()
            self.common.update(self.plate_defaults)
        self.model_error = ""
        if self.real_backend:
            try:
                self.home = self.real_model_home(self.calibration_file, self.physical_profile)
            except Exception as error:
                # Keep the GUI available to show the error and saved selection,
                # but never substitute nominal geometry for a failed real model.
                self.home = [math.nan, math.nan, math.nan]
                self.model_error = f"실제 모델 설정 오류: {error}"
                self.node.get_logger().error(self.model_error)
        else:
            cells = list(csv.DictReader((self.share / "config/a3_grid_points.csv").open()))
            self.home = [sum(float(row[key]) for row in cells) / len(cells) for key in ("x_m", "y_m", "z_m")]
            self.home[2] += self.simulation_riser_height
        self.columns, self.rows = self.common["columns"], self.common["rows"]
        self.obstacles, self.target = set(), 0
        self.empty = set()
        self.scan_mode = False
        self.plate_alignment = False
        self.alignment_paused = False
        self.alignment_stop = None
        self.pending_alignment_plan = False
        self.recovery_verified = False
        self.job = None
        self.calibration_window = None
        self.events = queue.Queue()
        self.job_failed, self.job_succeeded = False, False
        self.last_failure_message = ""
        self.user_stopped = False
        self.closing = False
        self.close_requested = False
        self.stop_pending = False
        self.stop_future = None
        self.stop_acknowledged = False
        self.stop_backend_confirmed = False
        self.stop_requested_at = 0.0
        self.stop_stationary_since = None
        self.stop_requires_motion = False
        self.execution_mode = tk.StringVar(value=self.EXECUTION_LABELS["step"])
        self.incremental_mode = tk.BooleanVar(value=self.real_backend)
        self.step_status = tk.StringVar(value="실행할 경로를 계획한 뒤 실행 방식을 선택하세요.")
        self.step_request = None
        self.step_client = None
        self.step_future = None
        self.step_future_request_id = None
        self.step_approval_locked = False
        self.step_seen_requests = set()
        self.step_latest_index = 0
        self.step_total = None
        self.job_plan_id = None
        self.domain = os.getenv("ROS_DOMAIN_ID", "0")
        self.folder = Path.home() / ".ros/ur5e_a3_stack"
        self.folder.mkdir(parents=True, exist_ok=True)
        suffix = "_real" if self.real_backend else "_gazebo" if self.simulation_backend == "gazebo" else ""
        geometry_suffix = "_cube_plate_v1" if self.plate_enabled else ""
        self.selection_file = self.folder / f"layout_domain_{self.domain}{suffix}{geometry_suffix}.yaml"
        self.trajectory_file = self.folder / f"air_plan_domain{self.domain}{geometry_suffix}.cdr"
        self.air_trajectory_file = self.trajectory_file
        self.full_physical_trajectory_file = self.folder / f"physical_plan_domain{self.domain}{geometry_suffix}.recipe.json"
        self.incremental_trajectory_file = self.folder / f"physical_next_plan_domain{self.domain}{geometry_suffix}.recipe.json"
        self.physical_trajectory_file = self.incremental_trajectory_file if self.real_backend else self.full_physical_trajectory_file
        self.recovery_trajectory_file = self.folder / f"recovery_domain{self.domain}{geometry_suffix}.recipe.json"
        self.plan_kind = "air"
        self.plan_manifest_file = self.trajectory_file.with_suffix(".gui.json")
        self.plan_valid = False
        self.job_selection_hash = None
        self.mode = tk.StringVar(value="obstacle")
        default_center = self.plate_defaults.get("plate_center_xy", self.home[:2])
        self.center_x = tk.StringVar(value="" if self.model_error else f"{default_center[0]:.6f}")
        self.center_y = tk.StringVar(value="" if self.model_error else f"{default_center[1]:.6f}")
        self.obstacle_height = tk.StringVar(value="150")
        self.speed = tk.StringVar(value="0.05" if self.real_backend else "0.25")
        if self.real_backend:
            self.empty_hand_clearance = tk.StringVar(value="50")
        self.placement_mode = tk.StringVar(value=self.PLACEMENT_LABELS["side"])
        self.summary = tk.StringVar()
        self.corner_text = tk.StringVar()
        self.status = tk.StringVar(value=self.model_error or ("배치를 선택하고 50mm 위 공중 경로를 먼저 계획하세요."
                                         if self.real_backend else
                                         "장애물과 쌓을 칸을 선택하세요. 실행은 가상 로봇에서만 가능합니다."))
        self.connection = tk.StringVar(value="MoveIt 확인 중")
        self.hardware_status = tk.StringVar(value="팔 / DH 제어기 / 관절 피드백 확인 중")
        self.visible_layout = None
        self.last_marker_time = 0.0
        self.controls = []
        self.load_selection()
        if self.real_backend and not self.model_error:
            self.status.set("장애물 위 XY 순회 경로를 먼저 계획하세요. 전체 24칸을 방문하고 마지막 위치를 유지합니다."
                            if self.scan_mode else
                            f"배치를 선택하고 {self.empty_hand_clearance.get()}mm 위 공중 경로를 먼저 계획하세요.")
        self.build()
        for variable in (self.center_x, self.center_y, self.obstacle_height, self.speed, self.placement_mode):
            variable.trace_add("write", self.invalidate_plan)
        self.placement_mode.trace_add("write", lambda *_: self.redraw())
        if self.real_backend:
            self.empty_hand_clearance.trace_add("write", self.clearance_changed)
            self.obstacle_height.trace_add("write", lambda *_: self.redraw() if self.scan_mode else None)
            self.plan_valid = self.plan_matches_selection()
        self.redraw()
        self.refresh_run_button()
        self.root.after(100, self.poll)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    @staticmethod
    def checked_simulation_riser_height(value):
        if isinstance(value, bool):
            raise ValueError("simulation_riser_height must be finite and between 0 and 0.3 m")
        try:
            height = float(value)
        except (TypeError, ValueError) as error:
            raise ValueError("simulation_riser_height must be finite and between 0 and 0.3 m") from error
        if not math.isfinite(height) or not 0.0 <= height <= 0.3:
            raise ValueError("simulation_riser_height must be finite and between 0 and 0.3 m")
        return height

    @classmethod
    def real_model_home(cls, calibration_file, physical_profile):
        # The real task and GUI must derive travel height from the same native
        # calibrated model and measured mounting distances.
        import xml.etree.ElementTree as ET
        from ur5e_block_bringup.model import load_moveit_settings
        from ur5e_block_bringup.air_scene import RobotKinematics

        settings = load_moveit_settings(calibration_file, physical_profile)
        semantic = ET.fromstring(settings["robot_description_semantic"])
        states = semantic.findall("group_state[@group='ur5e_arm'][@name='home']")
        if len(states) != 1:
            raise ValueError("SRDF에 ur5e_arm/home 상태가 하나 있어야 합니다.")
        values = {}
        for joint in states[0].findall("joint"):
            name = joint.get("name")
            if name in values:
                raise ValueError(f"home 상태에 중복 관절이 있습니다: {name}")
            values[name] = float(joint.get("value"))
        if not cls.ARM_JOINTS <= values.keys() or not all(math.isfinite(value) for value in values.values()):
            raise ValueError("home 상태에 유한한 팔 관절각 6개가 필요합니다.")
        home_q = {name: values[name] for name in cls.ARM_JOINTS}
        transform = RobotKinematics(settings["robot_description"]).tcp(home_q)
        home = [float(transform[index, 3]) for index in range(3)]
        if not all(math.isfinite(value) for value in home):
            raise ValueError("실제 모델의 home TCP 위치를 계산하지 못했습니다.")
        return home

    def build(self):
        simulator = "Gazebo 물리 시뮬레이션" if getattr(self, "simulation_backend", "mock") == "gazebo" else "가상 로봇"
        self.root.title(("UR5e · cube plate " if self.plate_enabled else "UR5e · A3 ") + ("실물 로봇 제어" if self.real_backend else f"{simulator} · ROS {self.domain}"))
        width = min(1880, max(640, self.root.winfo_screenwidth()-60))
        height = min(1040, max(480, self.root.winfo_screenheight()-100))
        self.root.geometry(f"{width}x{height}")
        self.root.minsize(min(width, 1200), min(height, 680))
        self.root.configure(bg="#eef2f6")
        self.layout_panes = tk.PanedWindow(self.root, orient="horizontal", sashwidth=8,
                                          bg="#c6d0da", bd=0, opaqueresize=True)
        self.layout_panes.pack(fill="both", expand=True)
        outer = ttk.Frame(self.layout_panes, padding=(12, 12, 8, 12))
        terminal = ttk.Frame(self.layout_panes, padding=(8, 12, 12, 12))
        self.layout_panes.add(outer, width=width//2, minsize=min(570, width//2), stretch="always")
        self.layout_panes.add(terminal, width=width//2, minsize=min(320, width//3), stretch="always")
        # Initial requested sizes are not the mapped window size. Apply this
        # once after mapping; later user drags/resizes keep their position.
        def half_split(_):
            self.layout_panes.unbind("<Map>")
            self.root.after_idle(lambda: self.layout_panes.sash_place(
                0, self.layout_panes.winfo_width()//2, 0))
        self.layout_panes.bind("<Map>", half_split)

        def wrapped_label(parent, **options):
            label = ttk.Label(parent, wraplength=600, **options)
            label.bind("<Configure>", lambda event: label.configure(wraplength=max(100, event.width)))
            return label

        wrapped_label(outer, text=(f"플레이트 {self.columns}열 × {self.rows}행 · 쌓을 홈 선택 · " if self.plate_enabled else "A3 4열 × 6행 · ") + ("실물 쌓기 / 팔 경로 시운전" if self.real_backend else "블록 쌓기"),
                      font=("Sans", 16, "bold")).pack(fill="x")
        top = ttk.Frame(outer)
        top.pack(fill="x", pady=(8, 10))
        ttk.Label(top, textvariable=self.connection).pack(side="left")
        wrapped_label(top, text=("start_block_robot.sh와 RViz를 먼저 실행하세요." if self.real_backend else
                                 "start_block_gazebo.sh · Gazebo에서 접촉·쌓기를 확인하세요." if getattr(self, "simulation_backend", "mock") == "gazebo" else
                                 "start_block_sim.sh 가상 모드 · RViz에서 동작을 확인하세요.")).pack(side="right")
        wrapped_label(outer, textvariable=self.hardware_status).pack(fill="x", pady=(0, 8))
        execution_bar = ttk.Frame(outer)
        execution_bar.pack(fill="x", pady=(0, 5))
        ttk.Label(execution_bar, text="실행 방식").pack(side="left", padx=(0, 8))
        execution_choice = ttk.Combobox(execution_bar, textvariable=self.execution_mode,
                                       values=tuple(self.EXECUTION_LABELS.values()), state="readonly", width=19)
        execution_choice.pack(side="left")
        self.controls.append(execution_choice)
        self.run_button = ttk.Button(
            execution_bar,
            text="⑤ 블록 없는 팔 경로 실행" if self.real_backend else "⑤ 선택한 방식으로 가상 실행",
            command=lambda: self.launch_task("run"))
        self.run_button.pack(side="left", padx=(12, 0))
        self.controls.append(self.run_button)
        self.approve_button = ttk.Button(outer, text="확인 OK · 표시한 경로 실행",
                                         command=self.approve_next_step, state="disabled")
        self.approve_button.pack(fill="x", pady=(0, 5))
        wrapped_label(outer, textvariable=self.step_status).pack(fill="x", pady=(0, 7))
        # Approval, Stop and status stay above the scrollable settings area.
        status_bar = ttk.Frame(outer)
        status_bar.pack(fill="x", pady=(0, 8))
        self.stop_button = ttk.Button(status_bar, text="실행 중지", command=self.stop, state="disabled")
        self.stop_button.pack(side="right", padx=(8, 0))
        wrapped_label(status_bar, textvariable=self.status,
                      font=("Sans", 10, "bold")).pack(side="left", fill="x", expand=True)
        body = ttk.Frame(outer)
        board = ttk.LabelFrame(body, text="① 종류 선택 → ② 홈 클릭 (+Y 위)", padding=8)
        options = ttk.Frame(board)
        options.pack(fill="x", pady=(0, 8))
        kinds = [("일반 블록", "block"), ("장애물", "obstacle"),
                 ("쌓을 칸", "target"), ("빈칸", "empty")]
        for index, (label, mode) in enumerate(kinds):
            button = ttk.Radiobutton(options, text=label, value=mode, variable=self.mode)
            button.grid(row=index//2, column=index%2, sticky="w", padx=5, pady=1)
            self.controls.append(button)
        if self.plate_enabled:
            hint = tk.Label(board, text="◆ ‘쌓을 칸’을 고른 뒤 원하는 홈을 클릭하세요. 그 홈의 블록 1개를 바닥으로 유지합니다.",
                            bg=self.COLORS["target"], fg="#17222b", font=("Sans", 10, "bold"), wraplength=280)
            hint.pack(fill="x", pady=(0, 6))
            hint.bind("<Configure>", lambda event: hint.configure(wraplength=max(100, event.width)))
        grid = ttk.Frame(board)
        grid.pack(fill="both", expand=True)
        self.cells = {}
        for row in range(self.rows):
            grid.rowconfigure(row, weight=1)
            for column in range(self.columns):
                grid.columnconfigure(column, weight=1)
                cell = row * self.columns + column + 1
                button = tk.Button(grid, font=("Sans", 12, "bold"), relief="flat", bd=0,
                                   command=lambda cell=cell: self.select_cell(cell))
                button.grid(row=row, column=column, sticky="nsew", padx=3, pady=3)
                self.cells[cell] = button
                self.controls.append(button)
        panel = ttk.Frame(body, width=320)
        panel.pack(side="right", fill="y")
        panel.pack_propagate(False)
        board.pack(side="left", fill="both", expand=True, padx=(0, 10))
        self.settings_canvas = tk.Canvas(panel, width=300, highlightthickness=0, bd=0, bg="#eef2f6")
        scrollbar = ttk.Scrollbar(panel, orient="vertical", command=self.settings_canvas.yview)
        scrollbar.pack(side="right", fill="y")
        self.settings_canvas.pack(side="left", fill="both", expand=True)
        self.settings_canvas.configure(yscrollcommand=scrollbar.set)
        right = ttk.Frame(self.settings_canvas, padding=(0, 0, 6, 0))
        window = self.settings_canvas.create_window((0, 0), window=right, anchor="nw")
        right.bind("<Configure>", lambda _: self.settings_canvas.configure(
            scrollregion=self.settings_canvas.bbox("all")))
        self.settings_canvas.bind("<Configure>", lambda event: self.settings_canvas.itemconfigure(
            window, width=event.width))
        self.settings_canvas.bind("<Button-4>", lambda _: self.settings_canvas.yview_scroll(-1, "units"))
        self.settings_canvas.bind("<Button-5>", lambda _: self.settings_canvas.yview_scroll(1, "units"))
        self.settings_canvas.bind("<MouseWheel>", lambda event: self.settings_canvas.yview_scroll(
            -1 if event.delta > 0 else 1, "units"))
        wrapped_label(right, textvariable=self.summary,
                      font=("Sans", 10 if self.real_backend else 11)).pack(fill="x", pady=(0, 8 if self.real_backend else 12))
        settings = ttk.LabelFrame(right, text="배치와 속도", padding=10)
        settings.pack(fill="x")
        entries = [("판 중심 X (m)", self.center_x), ("판 중심 Y (m)", self.center_y),
                   ("장애물 높이 (mm)", self.obstacle_height),
                   ("팔 시운전 속도 (최대 0.05)" if self.real_backend else "팔 속도 배율", self.speed)]
        if self.real_backend:
            entries.append(("빈 경로 높이 여유 (mm)", self.empty_hand_clearance))
        for index, (label, variable) in enumerate(entries):
            ttk.Label(settings, text=label).grid(row=index, column=0, sticky="w", pady=2)
            entry = ttk.Entry(settings, textvariable=variable, width=10)
            entry.grid(row=index, column=1, padx=8, pady=2)
            self.controls.append(entry)
        for label, callback in ((("검사한 판 배치로 복원" if self.plate_enabled else "현재 home 중심으로"), self.home_board),
                                ("간단한 3층 예시", self.example),
                                (("쌓을 홈 선택 · 전체 예시" if self.plate_enabled else "전체 23층 예시 / 19층 이후 측면"), self.full_example),
                                ("선택 초기화", self.clear_selection)):
            button = ttk.Button(settings, text=label, command=callback)
            button.grid(column=0, columnspan=2, sticky="ew", pady=1)
            self.controls.append(button)
        placement_row = ttk.Frame(settings)
        placement_row.grid(column=0, columnspan=2, sticky="ew", pady=3)
        ttk.Label(placement_row, text="탑에 놓는 방식").pack(anchor="w")
        placement = ttk.Combobox(placement_row, textvariable=self.placement_mode,
                                 values=tuple(self.PLACEMENT_LABELS.values()), width=28, state="readonly")
        placement.pack(fill="x", pady=(3, 0))
        self.controls.append(placement)
        if self.real_backend:
            incremental = ttk.Checkbutton(right, text="한 블록씩 계획·반복 실행 (시간 제한 없음)",
                variable=self.incremental_mode, command=self.incremental_changed)
            incremental.pack(fill="x", pady=(6, 3))
            self.controls.append(incremental)
        actions = [("③ Gazebo 배치 초기화 / RViz에 표시" if getattr(self, "simulation_backend", "mock") == "gazebo" else "③ 선택 배치를 RViz에 표시", "scene")]
        if self.real_backend:
            actions.append(("④ 실물 경로 준비 (한 블록 / 전체)", "physical_plan"))
            actions.append(("저장 경로 사전 검사 (현재 정책)", "physical_prepare"))
            actions.append(("블록 없이 팔 경로 계획 · 시운전", "plan"))
        else:
            actions.append(("④ 전체 경로 / 높이 확인", "plan"))
        if self.plate_enabled:
            actions.append(("플레이트 맞춤 · 모서리 X/Y", "plate_align"))
        if self.real_backend:
            actions.append(("장애물 위 XY 순회 경로", "scan"))
            actions.append(("안전 확인 후 초기 자세 복구", "recovery"))
            actions.append(("빈 그리퍼 확인 · 중단 잠금 해제", "unlock_empty"))
        actions.append(("Gazebo / RViz 작업 물체 지우기" if getattr(self, "simulation_backend", "mock") == "gazebo" else "RViz 작업 물체 지우기", "reset"))
        for label, action in actions:
            button = ttk.Button(right, text=label, command=lambda action=action: self.launch_task(action))
            button.pack(fill="x", pady=2)
            self.controls.append(button)
        if self.plate_enabled:
            wrapped_label(right, textvariable=self.corner_text,
                          font=("Monospace", 8)).pack(fill="x", pady=(5, 3))
            wrapped_label(right, text=f"그리퍼 {self.ALIGNMENT_HAND:g}rad · 홈 중심 / 상판 위 10mm\n각 홈 X/Y에서 정지 · 상승 후 90° 회전 · 조정 후 손을 빼고 OK").pack(fill="x", pady=(0, 4))
        if self.real_backend:
            self.calibration_button = ttk.Button(right, text="25mm 파지 보정",
                                                 command=self.open_gripper_calibration)
            self.calibration_button.pack(fill="x", pady=(7, 2))
            self.controls.append(self.calibration_button)
        guidance = ("파랑: 25mm 블록 / 빨강: 25×25×150mm 장애물\n노랑: 바닥 블록이 있는 쌓을 홈 / 회색: 빈칸\n\n실물 집기·쌓기: 잡기 0.73 → 놓기 실측 기반 1차 열림(최대 0.70) → 0.60까지 연 뒤 팔 이동.\n"
                    f"전체 경로 모드: 접촉 {ARM_SPEEDS['contact']:.2f} / 하강 {ARM_SPEEDS['descent_fast']:.2f} / 이동 {ARM_SPEEDS['transfer']:.2f}.\n"
                    f"한 블록 모드: 집기 접근·하강 {PICKUP_SPEED[0]:.2f} / 운반→놓기 직전까지 한 번에 이동.\n"
                    f"운반 속도·가속도: {FAST_LAYERS}층까지 {LOW_TOWER_SPEED[0]:.2f}/{LOW_TOWER_SPEED[1]:.2f}, "
                    f"{FAST_LAYERS+1}층부터 {HIGH_TOWER_SPEED[0]:.2f}/{HIGH_TOWER_SPEED[1]:.2f}. 최종 놓기는 {ARM_SPEEDS['approach']:.2f}.\n"
                    "실물 경로 계획 후 상단 실행과 OK로 승인합니다.\n팔 경로 시운전은 블록 없이 현재 그리퍼 자세를 유지합니다.\n모서리 맞춤은 그리퍼 0.65, 각 X/Y 위치에서 조정 후 OK로 재개합니다."
                    if self.real_backend else
                    ("파랑: 블록 / 빨강: 장애물\n노랑: 바닥 블록 1개가 있는 쌓을 홈 / 회색: 빈칸\n노란 블록은 유지하고 파란 블록을 2층부터 쌓습니다.\n" if self.plate_enabled else
                     "파랑: 블록 / 빨강: 장애물\n노랑: 탑 밑 블록 / 회색: 빈칸\n모든 파란 블록을 노란 칸에 쌓습니다.\n") +
                    "25mm 블록은 19층(475mm)까지 수직, 20층부터 측면입니다. 모든 높이 수직도 선택할 수 있습니다.\n\n단계별 확인은 구간마다, 전체 연속 실행은 전체 경로 확인 후 상단 OK로 움직입니다.\n가상 로봇과 GUI를 같은 ROS domain에서 실행하세요.")
        wrapped_label(right, text=guidance, justify="left").pack(fill="x", pady=8)
        log_header = ttk.Frame(terminal)
        log_header.pack(fill="x")
        ttk.Label(log_header, text="실행 터미널 · 로그", font=("Sans", 14, "bold")).pack(side="left")
        self.copy_log_button = ttk.Button(log_header, text="로그 전체 복사", command=self.copy_log)
        self.copy_log_button.pack(side="right")
        self.log_copy_status = tk.StringVar(value="")
        wrapped_label(terminal, textvariable=self.log_copy_status).pack(fill="x", pady=(3, 0))
        wrapped_label(terminal, text="로그 전체 복사는 현재 작업의 저장 로그를 복사합니다. 화면에서 잘린 앞부분도 포함합니다.").pack(fill="x", pady=(5, 8))
        log_area = ttk.Frame(terminal)
        log_area.pack(fill="both", expand=True)
        log_area.rowconfigure(0, weight=1)
        log_area.columnconfigure(0, weight=1)
        self.log = tk.Text(log_area, height=1, width=1, wrap="none", bg="#18232e", fg="#dce7f0",
                           insertbackground="#dce7f0", font=("Monospace", 11))
        log_vertical = ttk.Scrollbar(log_area, orient="vertical", command=self.log.yview)
        log_horizontal = ttk.Scrollbar(log_area, orient="horizontal", command=self.log.xview)
        self.log.configure(yscrollcommand=log_vertical.set, xscrollcommand=log_horizontal.set)
        self.log.grid(row=0, column=0, sticky="nsew")
        log_vertical.grid(row=0, column=1, sticky="ns")
        log_horizontal.grid(row=1, column=0, sticky="ew")
        if self.real_backend:
            self.footer_note = tk.StringVar(value=self.air_path_note())
            wrapped_label(outer, textvariable=self.footer_note,
                          font=("Sans", 9)).pack(side="bottom", anchor="w", fill="x", pady=(5, 0))
        else:
            physics = "Gazebo 접촉·마찰·중력 · 시험 물성" if getattr(self, "simulation_backend", "mock") == "gazebo" else "GenericSystem 시뮬레이션"
            wrapped_label(outer, text=f"ROS domain {self.domain} · 수직으로 집기 / 측면 또는 수직으로 놓기 · {physics}",
                          font=("Sans", 9)).pack(side="bottom", fill="x", pady=(5, 0))
        body.pack(fill="both", expand=True)

    def select_cell(self, cell):
        if self.job or self.stop_pending or self.calibration_window is not None:
            return
        self.invalidate_plan()
        if self.mode.get() == "target":
            self.target = cell
            self.obstacles.discard(cell)
            self.empty.discard(cell)
        elif self.mode.get() == "obstacle":
            if cell in self.obstacles:
                self.obstacles.remove(cell)
            else:
                self.obstacles.add(cell)
            self.empty.discard(cell)
            if self.target == cell:
                self.target = 0
        elif self.mode.get() == "empty":
            self.empty.add(cell)
            self.obstacles.discard(cell)
            if self.target == cell:
                self.target = 0
        else:
            self.obstacles.discard(cell)
            self.empty.discard(cell)
            if self.target == cell:
                self.target = 0
        self.redraw()
        self.status.set("배치를 변경했습니다. ‘RViz에 표시’로 반영하세요.")

    def redraw(self):
        for cell, button in self.cells.items():
            kind = "target" if cell == self.target else "obstacle" if cell in self.obstacles else "empty" if cell in self.empty else "block"
            label = {"target": "쌓을 곳 · 바닥 1개" if getattr(self, "plate_enabled", False) else "쌓을 곳",
                     "obstacle": "장애물", "block": "블록", "empty": "빈칸"}[kind]
            button.configure(text=f"{cell}\n{label}", bg=self.COLORS[kind], activebackground=self.COLORS[kind],
                             fg="#17222b" if kind in ("target", "empty") else "white")
        count = self.columns*self.rows-len(self.obstacles)-len(self.empty)
        if getattr(self, "plate_enabled", False):
            try:
                geometry = self.current_plate_parameters()
                corners = geometry.get("plate_alignment_points_xyz", geometry["plate_corners_xyz"])
                ids = geometry.get("plate_alignment_cell_ids", [1, self.columns, self.rows*self.columns,
                                                                      (self.rows-1)*self.columns+1])
                self.corner_text.set("모서리 홈 중심 좌표 (world, mm)\n" + "\n".join(
                    f"{name}({ids[i]}번)  X {corners[3*i]*1000:.1f}  Y {corners[3*i+1]*1000:.1f}  Z {corners[3*i+2]*1000:.1f}"
                    for i, name in enumerate("ABCD")))
            except (ValueError, TypeError, KeyError):
                self.corner_text.set("판 중심 좌표를 확인하면 모서리 홈 중심을 표시합니다.")
        if getattr(self, "plate_alignment", False):
            self.summary.set("플레이트 X/Y 맞춤 · 모서리 A→B→C→D\n"
                             "각 홈에서 두 방향으로 맞춥니다.\n"
                             f"그리퍼 {self.ALIGNMENT_HAND:g}rad · 끝은 상판 위 10mm\n"
                             "X 정지 → 상승·90° 회전 → Y 정지\n"
                             "매번 조정 후 손을 빼고 다음 OK")
        elif self.real_backend and self.scan_mode:
            self.summary.set(f"XY {self.columns*self.rows}칸 각 1회 · 블록 없음\n"
                             f"장애물 25×25×{self.obstacle_height.get()}mm\n"
                             f"장애물 칸: {', '.join(map(str, sorted(self.obstacles))) or '없음'}\n"
                             f"상단 여유 {self.empty_hand_clearance.get()}mm · 잡기 없음\n"
                             "마지막 위치 유지")
        else:
            rule = ("25mm 블록 19층(475mm)까지 수직 / 20층부터 측면"
                    if self.placement_mode.get() == self.PLACEMENT_LABELS["side"] else "모든 높이에서 수직으로 놓기")
            if getattr(self, "plate_enabled", False):
                foundation = 1 if self.target else 0
                self.summary.set(f"장애물: {', '.join(map(str, sorted(self.obstacles))) or '없음'}\n"
                                 f"쌓을 홈: {self.target or '미선택'}\n"
                                 f"이동 블록 {max(0, count-foundation)}개 + 바닥 블록 {foundation}개\n"
                                 + ("바닥 블록은 유지 · 2층부터 쌓기\n" if foundation else "쌓을 홈의 바닥 블록 1개를 선택하세요.\n") +
                                 f"완료 시 {count}층 / 높이 {count*25}mm\n{rule}")
            else:
                self.summary.set(f"장애물: {', '.join(map(str, sorted(self.obstacles))) or '없음'}\n쌓을 칸: {self.target or '미선택'}\n"
                                 f"이동 블록 {max(0, count-1)}개 + 바닥 블록 1개\n완료 시 {count}층 / 높이 {count*25}mm (블록 25mm)\n{rule}")

    def home_board(self):
        if self.real_backend and self.model_error:
            self.status.set(self.model_error + " · 보정·현장 설정 파일을 확인하고 GUI를 다시 실행하세요.")
            return
        center = self.plate_defaults["plate_center_xy"] if getattr(self, "plate_enabled", False) else self.home[:2]
        self.center_x.set(f"{center[0]:.6f}")
        self.center_y.set(f"{center[1]:.6f}")
        self.status.set("검사한 플레이트 배치로 복원했습니다. 변경한 실제 판은 모서리 기준점에 맞추세요."
                        if getattr(self, "plate_enabled", False) else
                        "판 중심을 원래 home의 XY로 설정했습니다. RViz에 다시 표시하세요.")

    def example(self):
        self.invalidate_plan()
        self.home_board()
        if getattr(self, "plate_enabled", False):
            self.obstacles, self.target = set(), 21
            self.empty = set(range(1, self.rows*self.columns+1)) - {4, 13, 21}
            self.redraw()
            self.status.set("3층 예시: 21번 홈의 바닥 블록을 유지하고 4·13번 블록을 2·3층에 쌓습니다.")
            return
        self.obstacles, self.target = {7}, 21
        self.empty = {cell for cell in range(1, self.rows*self.columns+1) if cell not in (4, 7, 13, 21)}
        self.redraw()
        self.status.set("3층 예시: 장애물 7번, 4·13번 블록을 21번에 쌓습니다. 먼저 RViz에 표시하세요.")

    def clear_selection(self):
        self.invalidate_plan()
        self.obstacles, self.target = set(), 0
        self.empty = set()
        self.scan_mode = self.plate_alignment = False
        self.redraw()

    def full_example(self):
        self.invalidate_plan()
        self.home_board()
        self.obstacles, self.target, self.empty = (set(), 21, set()) if getattr(self, "plate_enabled", False) else ({1}, 24, set())
        self.placement_mode.set(self.PLACEMENT_LABELS["side"])
        self.redraw()
        if getattr(self, "plate_enabled", False):
            self.status.set("21번 홈의 바닥 블록 1개에 나머지 23개를 더해 24층으로 쌓는 예시입니다. 쌓을 홈은 직접 바꿀 수 있습니다.")
            return
        self.status.set("23층 경로 예시: 19층(475mm)까지 수직, 20층부터 측면. 장애물 1번, 탑 24번입니다. 실제 블록 없이 먼저 계획하세요."
                        if self.real_backend else
                        "23층 예시: 19층(475mm)까지 수직, 20층부터 측면. 장애물 1번, 탑 24번을 demo RViz에 표시하세요.")

    def load_selection(self):
        if not self.selection_file.exists():
            return
        try:
            value = yaml.safe_load(self.selection_file.read_text())["/**"]["ros__parameters"]
            if getattr(self, "plate_enabled", False) and (
                    value.get("geometry_profile") != self.common["geometry_profile"]
                    or value.get("plate_layout_defaults") != self.plate_layout_defaults()):
                backup = self.selection_file.with_name(self.selection_file.name+f".legacy.{time.time_ns()}")
                backup.write_bytes(self.selection_file.read_bytes())
                self.status.set("기존 격자 배치는 별도 보존했습니다. 새 플레이트의 홈 좌표로 다시 선택하세요.")
                return
            count = self.columns*self.rows
            self.obstacles = {int(cell) for cell in value.get("obstacle_cells", []) if 1 <= int(cell) <= count}
            self.empty = {int(cell) for cell in value.get("empty_cells", []) if 1 <= int(cell) <= count and int(cell) not in self.obstacles}
            self.target = int(value.get("target_cell", 0))
            if not 1 <= self.target <= count or self.target in self.obstacles or self.target in self.empty:
                self.target = 0
            center = value.get("center_xyz", self.home)
            self.center_x.set(str(center[0]))
            self.center_y.set(str(center[1]))
            self.obstacle_height.set(str(value.get("obstacle_size_xyz", [0, 0, .15])[2]*1000))
            self.speed.set(str(min(float(value.get("velocity_scaling", .05)), .05))
                           if self.real_backend else str(value.get("velocity_scaling", .25)))
            if self.real_backend:
                self.empty_hand_clearance.set(f"{float(value.get('empty_hand_clearance', .05))*1000:g}")
                scan_mode = value.get("empty_xy_scan", False)
                if not isinstance(scan_mode, bool):
                    raise ValueError("empty_xy_scan must be a boolean")
                self.scan_mode = scan_mode
                alignment = value.get("plate_alignment_test", False)
                if not isinstance(alignment, bool):
                    raise ValueError("plate_alignment_test must be a boolean")
                self.plate_alignment = alignment and getattr(self, "plate_enabled", False)
                physical = value.get("physical_stack_task", False)
                if type(physical) is not bool:
                    raise ValueError("physical_stack_task must be a boolean")
                if physical and not self.scan_mode and not self.plate_alignment:
                    self.set_plan_kind("physical", restore=True)
            # Preserve the selected cells without substituting an example layout.
            placement = value.get("placement_mode", "side")
            self.placement_mode.set(self.PLACEMENT_LABELS[placement])
        except (OSError, ValueError, TypeError, KeyError, yaml.YAMLError):
            self.obstacles, self.target = set(), 0
            self.empty = set()
            self.scan_mode = False
            self.plate_alignment = False

    def plate_parameters(self, center_xy=None):
        from ur5e_block_bringup.plate_geometry import plate_parameters
        settings = dict(self.common)
        settings["target_cell"] = getattr(self, "target", 0)
        return plate_parameters(settings, center_xy=center_xy)

    def current_plate_parameters(self):
        return self.plate_parameters([float(self.center_x.get()), float(self.center_y.get())])

    def plate_layout_defaults(self):
        # An operator's XY correction is kept across launches, while a newly
        # selected placement/orientation invalidates the old cell assignment.
        return [float(v) for v in self.plate_defaults["plate_center_xy"]] + [float(self.plate_defaults["plate_yaw_deg"])]

    def save_selection(self):
        if self.real_backend and self.model_error:
            raise ValueError(self.model_error + " · 보정·현장 설정 파일을 확인하고 GUI를 다시 실행하세요.")
        x, y = float(self.center_x.get()), float(self.center_y.get())
        height, speed = float(self.obstacle_height.get())/1000, float(self.speed.get())
        maximum = .05 if self.real_backend else 1.0
        if not all(math.isfinite(value) for value in (x, y, height, speed)) or height <= 0 or not 0 < speed <= maximum:
            raise ValueError(f"중심 좌표·장애물 높이·속도(0 초과~{maximum:g})를 확인하세요.")
        placement = {label: value for value, label in self.PLACEMENT_LABELS.items()}.get(self.placement_mode.get())
        if placement is None:
            raise ValueError("탑에 놓는 방식을 목록에서 선택하세요.")
        data = {"/**": {"ros__parameters": {
            "target_cell": self.target, "obstacle_cells": sorted(self.obstacles), "empty_cells": sorted(self.empty),
            "center_from_home": False, "center_xyz": [x, y, self.home[2]], "center_offset_xyz": [0., 0., 0.],
            "block_size_xyz": [.025, .025, .025],
            "obstacle_size_xyz": [.025, .025, height], "velocity_scaling": speed,
            "robot_riser_height": getattr(self, "simulation_riser_height", 0.0) if not self.real_backend else 0.0,
            "placement_mode": placement,
            "vertical_placement_layers": 19,
            "placement_approach_clearance": .12,
            "side_entry_clearance": .010,
            "gripper_open_position": .4, "gripper_grasp_position": .73,
            "staged_release_enabled": True, "release_first_position": .70,
            "release_clearance_position": .60, "approach_velocity_scaling": .05,
            "transfer_velocity_scaling": 1.0 if self.real_backend else .20,
        }}}
        if getattr(self, "plate_enabled", False):
            data["/**"]["ros__parameters"].update(self.current_plate_parameters())
            data["/**"]["ros__parameters"]["plate_layout_defaults"] = self.plate_layout_defaults()
            data["/**"]["ros__parameters"]["plate_alignment_test"] = getattr(self, "plate_alignment", False)
        if self.real_backend:
            data["/**"]["ros__parameters"]["physical_stack_task"] = getattr(self, "plan_kind", "air") == "physical"
            data["/**"]["ros__parameters"].update(
                physical_approach_clearance=APPROACH_CLEARANCE,
                physical_contact_clearance=CONTACT_CLEARANCE,
                descent_velocity_scaling=ARM_SPEEDS['descent_fast'],
                descent_acceleration_scaling=ARM_SPEEDS['descent_fast'],
                incremental_stack=self.incremental_enabled(), post_place_retreat_clearance=.050,
                incremental_pickup_approach_clearance=.080, incremental_transfer_clearance=.050)
            if data["/**"]["ros__parameters"]["physical_stack_task"]:
                # Keep the selected empty-hand speed in the shared layout.
                # The physical launch applies its own motion speed policy.
                data["/**"]["ros__parameters"].update(motion_parameters(), velocity_scaling=speed)
                data["/**"]["ros__parameters"].update(incremental_motion_parameters())
            try:
                clearance = float(self.empty_hand_clearance.get()) / 1000.0
            except (ValueError, TypeError) as error:
                raise ValueError("빈 경로 높이 여유를 30~100mm 범위로 입력하세요.") from error
            if not math.isfinite(clearance) or not .03 <= clearance <= .10:
                raise ValueError("빈 경로 높이 여유를 30~100mm 범위로 입력하세요.")
            # The real backend gets mounting distances from physical_profile.
            data["/**"]["ros__parameters"].pop("robot_riser_height")
            data["/**"]["ros__parameters"]["empty_hand_clearance"] = clearance
            if getattr(self, "plate_enabled", False):
                data["/**"]["ros__parameters"]["plate_alignment_clearance"] = clearance
            data["/**"]["ros__parameters"]["empty_xy_scan"] = self.scan_mode
            data["/**"]["ros__parameters"]["plate_alignment_test"] = getattr(self, "plate_alignment", False)
        temporary = self.selection_file.with_suffix(".tmp")
        temporary.write_text(yaml.safe_dump(data, allow_unicode=True))
        temporary.replace(self.selection_file)

    def busy(self, active):
        active = active or self.calibration_window is not None or self.close_requested
        for widget in self.controls:
            widget.configure(state="disabled" if active else "readonly" if isinstance(widget, ttk.Combobox) else "normal")
        self.stop_button.configure(state="normal" if active else "disabled")
        self.refresh_run_button()
        self.refresh_approval_button()

    def selected_execution_mode(self):
        selected = {label: mode for mode, label in self.EXECUTION_LABELS.items()}.get(self.execution_mode.get())
        if selected is None:
            raise ValueError("실행 방식을 목록에서 선택하세요.")
        return selected

    def clear_step_request(self, message=None):
        # Destroying this local client does not stop an accepted arm goal. Stop
        # always goes through the backend's separate cancellation service.
        if self.step_future is not None:
            self.step_future.cancel()
        if self.step_client is not None:
            self.node.destroy_client(self.step_client)
        self.step_request = self.step_client = self.step_future = None
        self.step_future_request_id = None
        self.step_approval_locked = False
        if message is not None:
            self.step_status.set(message)
        self.refresh_approval_button()

    def refresh_approval_button(self):
        if not hasattr(self, "approve_button"):
            return
        enabled = (self.job is not None and self.job_action in self.MOTION_ACTIONS and self.step_request is not None
                   and self.step_future is None and not self.step_approval_locked and self.step_client is not None
                   and self.step_client.service_is_ready() and not self.stop_pending
                   and not self.user_stopped and not self.close_requested and self.execution_ready())
        self.approve_button.configure(
            state="normal" if enabled else "disabled",
            text=(f"{(getattr(self, 'alignment_stop', None) or {}).get('axis', '')} 조정 완료 · OK로 계속 이동".strip()
                  if getattr(self, "alignment_paused", False)
                  else "확인 OK · 표시한 경로 실행"))

    def alignment_pause_text(self):
        stop = getattr(self, "alignment_stop", None) or {}
        return (f"모서리 홈 {stop.get('corner', '')} · {stop.get('pocket_cell', '')}번 · "
                f"{stop.get('axis', 'X/Y')}방향 맞춤 정지 중입니다. "
                "판을 맞추고 손을 뺀 뒤 조정 완료 · OK로 계속 이동을 누르세요.")

    def receive_step_ready(self, payload):
        if (self.job is None or self.job_action not in self.MOTION_ACTIONS or self.stop_pending
                or self.user_stopped or self.close_requested):
            return
        try:
            request_id, plan_id = payload["request_id"], payload["plan_id"]
            service = payload["approve_service"]
            index, total = payload["index"], payload["total"]
            duration, label = payload["duration"], payload["label"]
            preview_hash = payload.get("preview_sha256")
            if (not isinstance(request_id, str) or not re.fullmatch(r"[A-Za-z0-9_]{1,80}", request_id)
                    or service != "/ur5e_a3_stack/approve_next_" + request_id
                    or not isinstance(plan_id, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,128}", plan_id)
                    or type(index) is not int or type(total) is not int or not 1 <= index <= total <= 10000
                    or isinstance(duration, bool) or not isinstance(duration, (int, float))
                    or not math.isfinite(duration) or duration <= 0
                    or not isinstance(label, str) or not label.strip() or len(label) > 200
                    or (preview_hash is None and self.real_backend)
                    or (preview_hash is not None and
                        (not isinstance(preview_hash, str) or not re.fullmatch(r"[0-9a-f]{64}", preview_hash)))):
                raise ValueError("경로 확인 정보 형식이 올바르지 않습니다.")
            if request_id in self.step_seen_requests:
                return
            if self.job_plan_id is not None and plan_id != self.job_plan_id:
                raise ValueError("현재 실행 계획과 다른 경로 확인 요청입니다.")
            if index != self.step_latest_index + 1 or self.step_total not in (None, total):
                raise ValueError("경로 확인 단계 순서가 맞지 않습니다.")
            if self.real_backend and self.job_plan_id is None:
                raise ValueError("저장 계획을 확인할 수 없어 실행을 승인하지 않습니다.")
        except (KeyError, TypeError, ValueError) as error:
            self.clear_step_request("경로 확인 오류 · 중지한 뒤 다시 계획하세요.")
            self.status.set(str(error))
            return
        self.clear_step_request()
        self.job_plan_id = plan_id
        self.step_seen_requests.add(request_id)
        self.step_latest_index, self.step_total = index, total
        self.step_request = dict(payload)
        self.step_client = self.node.create_client(Trigger, service)
        self.step_status.set(f"경로 확인 {index}/{total} · {label} · 약 {duration:.1f}초 · RViz 확인 후 상단 OK")
        self.status.set(self.alignment_pause_text()
                        if getattr(self, "alignment_paused", False) else
                        "표시한 경로를 확인하세요. OK를 누르기 전에는 다음 동작을 시작하지 않습니다.")
        self.refresh_approval_button()

    def approve_next_step(self):
        self.refresh_approval_button()
        if (self.step_request is None or self.step_future is not None or self.step_approval_locked or self.step_client is None
                or not self.step_client.service_is_ready() or self.job is None or self.job_action not in self.MOTION_ACTIONS
                or self.stop_pending or self.user_stopped or self.close_requested or not self.execution_ready()):
            return
        self.step_future_request_id = self.step_request["request_id"]
        self.step_approval_locked = True
        try:
            self.step_future = self.step_client.call_async(Trigger.Request())
        except Exception as error:
            self.step_status.set("승인 전송 결과를 확인하지 못했습니다. 중복 승인하지 않습니다.")
            self.status.set(f"승인 요청 실패: {error}. 필요하면 실행 중지를 누르세요.")
            self.refresh_approval_button()
            return
        self.step_approval_sent_at = time.monotonic()
        self.step_approval_warned = False
        self.step_approval_failed = False
        self.step_status.set(f"{self.step_request['index']}/{self.step_request['total']} 경로 승인 응답 대기 중")
        self.refresh_approval_button()

    def receive_step_approved(self, payload):
        if self.step_request is not None and payload.get("request_id") == self.step_request["request_id"]:
            request = self.step_request
            self.clear_step_request(f"{request['index']}/{request['total']} 승인됨 · 표시한 경로 실행 준비 중")

    def refresh_step_approval(self):
        if self.step_future is not None:
            token = self.step_future_request_id
            if self.step_future.done() and not self.step_approval_failed:
                try:
                    reply = self.step_future.result()
                    if self.step_request is not None and token == self.step_request["request_id"]:
                        request = self.step_request
                        if reply is None or not reply.success:
                            reason = "응답이 없습니다" if reply is None else reply.message
                            self.clear_step_request("승인이 거절되었습니다. 중지한 뒤 다시 계획하세요.")
                            self.status.set(f"경로 승인 거절: {reason}")
                        else:
                            self.clear_step_request(f"{request['index']}/{request['total']} 승인됨 · 표시한 경로 실행 준비 중")
                except Exception as error:
                    # A transport error does not prove rejection. Keep OK locked:
                    # the backend might already have accepted this exact token.
                    self.step_status.set("승인 결과를 확인하지 못했습니다. 재승인하지 않고 상태를 기다립니다.")
                    self.status.set(f"승인 응답 확인 실패: {error}. 필요하면 실행 중지를 누르세요.")
                    self.step_approval_failed = True
                    self.step_approval_warned = True
            elif time.monotonic() - self.step_approval_sent_at > 5 and not self.step_approval_warned:
                self.step_approval_warned = True
                self.step_status.set("승인 응답 대기 중 · 중복 승인하지 않습니다. 필요하면 실행 중지를 누르세요.")
        self.refresh_approval_button()

    def process_step_event(self, line):
        if "STACK_ALIGNMENT_PAUSED " in line:
            if (self.job is not None and self.job_action == "run"
                    and getattr(self, "plate_alignment", False)
                    and not self.stop_pending and not self.user_stopped and not self.close_requested):
                try:
                    stop = json.loads(line.split("STACK_ALIGNMENT_PAUSED ", 1)[1].strip())
                    if (not isinstance(stop, dict) or stop.get("corner") not in ("A", "B", "C", "D")
                            or stop.get("axis") not in ("X", "Y")
                            or type(stop.get("pocket_cell")) is not int or stop["pocket_cell"] < 1
                            or type(stop.get("stop_index")) is not int or type(stop.get("total")) is not int
                            or stop["total"] != 8
                            or stop["stop_index"] != 2*"ABCD".index(stop["corner"])+(1 if stop["axis"] == "X" else 2)):
                        raise ValueError("모서리 X/Y 정지 정보가 올바르지 않습니다.")
                    self.alignment_stop = stop
                    self.alignment_paused = True
                    self.status.set(self.alignment_pause_text())
                    self.step_status.set(f"X/Y 맞춤 {stop['stop_index']}/8 · 조정 후 손을 빼고 다음 OK를 누르세요.")
                    self.refresh_approval_button()
                except (ValueError, TypeError, KeyError) as error:
                    self.clear_step_request("맞춤 정지 정보 오류 · 중지한 뒤 X/Y 경로를 다시 계획하세요.")
                    self.status.set(str(error))
            return True
        for marker, phase in (("STACK_STEP_READY ", "ready"),
                              ("STACK_STEP_APPROVED ", "approved"),
                              ("STACK_STEP_EXECUTING ", "executing"),
                              ("STACK_STEP_COMPLETED ", "completed")):
            if marker not in line:
                continue
            try:
                payload = json.loads(line.split(marker, 1)[1].strip())
                if not isinstance(payload, dict):
                    raise ValueError("경로 확인 메시지는 객체여야 합니다.")
                if phase == "executing" and self.job is not None and self.job_action in self.MOTION_ACTIONS:
                    # Any new dispatched motion invalidates old stop evidence,
                    # even if its READY/progress record was missed or Stop was
                    # already requested while that goal was being accepted.
                    self.stop_backend_confirmed = False
                    self.stop_acknowledged = False
                if phase == "ready":
                    self.receive_step_ready(payload)
                elif phase == "approved":
                    self.receive_step_approved(payload)
                else:
                    if (self.job is None or self.job_action not in self.MOTION_ACTIONS
                            or payload.get("index") != self.step_latest_index
                            or payload.get("total") != self.step_total):
                        return True
                    if phase == "executing":
                        self.alignment_paused = False
                        self.alignment_stop = None
                        self.clear_step_request(f"실행 중 {payload['index']}/{payload['total']} · {payload.get('label', '')}")
                        if not self.user_stopped:
                            self.status.set("승인한 경로 실행 중 · 중지가 필요하면 실행 중지를 누르세요.")
                    else:
                        self.clear_step_request(f"완료 {payload['index']}/{payload['total']} · 다음 경로 확인 대기 중")
            except (ValueError, TypeError, KeyError) as error:
                self.clear_step_request("경로 확인 메시지 오류 · 중지한 뒤 다시 계획하세요.")
                self.status.set(f"경로 확인 메시지를 읽지 못했습니다: {error}")
            return True
        return False

    def open_gripper_calibration(self):
        if not self.real_backend or self.closing or self.close_requested:
            return
        if self.calibration_window is not None:
            self.calibration_window.focus()
            return
        if self.job or self.stop_pending or self.stop_future is not None:
            self.status.set("팔 작업과 정지 확인을 마친 뒤 파지 보정을 여세요.")
            return
        if self.model_error:
            self.status.set(self.model_error + " · 보정·현장 설정 파일을 확인하세요.")
            return
        try:
            from ur5e_block_bringup.gripper_calibration_gui import GripperCalibrationWindow
            self.calibration_window = GripperCalibrationWindow(
                self.root, self.calibration_file, self.physical_profile,
                execution_mode=self.selected_execution_mode(),
                on_close=self.gripper_calibration_closed, invalidate_plan=self.invalidate_plan,
                on_status=self.gripper_calibration_status)
        except Exception as error:
            self.status.set(f"파지 보정 창을 열지 못했습니다: {error}")
            return
        self.invalidate_plan()
        self.busy(True)
        self.status.set("25mm 파지 보정 창을 열었습니다. 팔 작업은 잠겼으며 보정 창의 준비 확인 후 그리퍼만 움직입니다.")

    def copy_log(self):
        """Keep complete worker output available even after the 500-line view trims."""
        contents, fallback = None, False
        path = getattr(self, "job_logfile", None)
        if path is not None:
            try:
                contents = Path(path).read_text(encoding="utf-8", errors="replace")
            except OSError:
                fallback = True
        if contents is None:
            contents = self.log.get("1.0", "end-1c")
        if not contents.strip():
            self.log_copy_status.set("복사할 로그가 없습니다.")
            return
        try:
            self.root.clipboard_clear()
            self.root.clipboard_append(contents)
            self.root.update_idletasks()
        except tk.TclError as error:
            self.log_copy_status.set(f"로그 복사 실패: {error}")
            return
        source = "저장 파일을 읽을 수 없어 표시 로그만" if fallback else "로그 전체"
        self.log_copy_status.set(f"{source} {len(contents):,}자 복사 완료")

    def gripper_calibration_status(self, message):
        self.status.set(message)
        self.log.insert("end", message + "\n")
        if int(self.log.index("end-1c").split(".")[0]) > 500:
            self.log.delete("1.0", "80.0")
        self.log.see("end")

    def gripper_calibration_closed(self):
        self.calibration_window = None
        self.invalidate_plan()
        self.busy(bool(self.job or self.stop_pending))
        if self.close_requested:
            self.status.set("그리퍼 보정이 안전하게 종료되었습니다. 창을 닫습니다.")
        else:
            self.status.set("그리퍼 보정 창을 닫았습니다. 변경된 손가락 상태에 맞춰 팔 경로를 다시 계획하세요.")

    @staticmethod
    def file_hash(path):
        return hashlib.sha256(Path(path).read_bytes()).hexdigest()

    def incremental_enabled(self):
        variable = getattr(self, "incremental_mode", None)
        return self.real_backend and variable is not None and variable.get() is True

    def incremental_changed(self):
        self.invalidate_plan()
        self.physical_trajectory_file = (self.incremental_trajectory_file if self.incremental_enabled()
                                         else self.full_physical_trajectory_file)
        if getattr(self, "plan_kind", "air") == "physical":
            self.trajectory_file = self.physical_trajectory_file
            self.plan_manifest_file = self.trajectory_file.with_suffix(".gui.json")
        if hasattr(self, "footer_note"):
            self.footer_note.set(self.air_path_note())
        self.status.set("한 블록 경로 준비 후 반복 실행합니다. 기존 탑·남은 블록 장면을 유지하세요."
                        if self.incremental_enabled() else "전체 경로를 먼저 계획하는 방식입니다.")

    def set_plan_kind(self, kind, restore=False):
        if kind not in ("air", "physical"):
            raise ValueError("Unknown reviewed task kind")
        if not hasattr(self, "air_trajectory_file"):
            self.air_trajectory_file = self.trajectory_file
        if not hasattr(self, "physical_trajectory_file"):
            self.physical_trajectory_file = self.air_trajectory_file.with_name(
                "physical_"+self.air_trajectory_file.stem+".recipe.json")
        if getattr(self, "plan_kind", "air") == kind:
            self.plan_kind = kind
            return
        if not restore:
            self.invalidate_plan()
        self.plan_kind = kind
        self.trajectory_file = self.physical_trajectory_file if kind == "physical" else self.air_trajectory_file
        self.plan_manifest_file = self.trajectory_file.with_suffix(".gui.json")
        self.plan_valid = False
        if not restore:
            self.refresh_run_button()

    def invalidate_plan(self, *_):
        if not self.real_backend:
            return
        self.plan_valid = False
        try:
            self.plan_manifest_file.unlink(missing_ok=True)
        except OSError:
            pass
        self.refresh_run_button()

    def receive_recovery_done(self, payload):
        """Only this recovery's verified terminal record can reset the UI."""
        if (self.job is None or self.job_action != "recovery" or not self.real_backend
                or self.job_failed or self.user_stopped or self.stop_pending or self.close_requested):
            return
        valid_preserved = False
        if isinstance(payload,dict) and getattr(self,"job_recovery_preserve",False):
            position=payload.get('gripper_position')
            valid_preserved=(payload.get('policy')==HOME_PRESERVE_POLICY
                and payload.get('gripper_preserved') is True and payload.get('scene_preserved') is True
                and type(position) in (int,float) and math.isfinite(position) and 0 <= position <= .93
                and type(payload.get('held_fault_retained')) is bool)
        valid_legacy = (isinstance(payload,dict) and not getattr(self,"job_recovery_preserve",False)
                        and payload.get('gripper_empty') is True and payload.get('scene_cleared') is True
                        and type(payload.get('hand_target')) in (int,float) and payload['hand_target']==.4)
        if (not isinstance(payload, dict) or payload.get("plan_id") != self.job_plan_id
                or not self.job_plan_id or payload.get("home_reached") is not True
                or not (valid_preserved or valid_legacy)):
            self.job_failed = True
            self.last_failure_message = "초기 자세 복구의 완료 증거를 확인하지 못했습니다. 현재 상태를 유지합니다."
            self.status.set(self.last_failure_message)
            return
        self.recovery_verified = True
        self.job_succeeded = True
        if valid_preserved:
            self.recovery_held_fault_retained = payload['held_fault_retained']

    def receive_empty_unlock_done(self, payload):
        if (self.job is None or self.job_action != 'unlock_empty' or not self.real_backend
                or self.job_failed or self.user_stopped or self.stop_pending or self.close_requested):
            return
        if (not isinstance(payload,dict) or payload.get('policy') != EMPTY_UNLOCK_POLICY
                or not self.job_plan_id or payload.get('plan_id') != self.job_plan_id
                or payload.get('gripper_empty') is not True or payload.get('fault_cleared') is not True
                or payload.get('arm_motion') is not False or payload.get('gripper_motion') is not False
                or payload.get('scene_preserved') is not True):
            self.job_failed = True
            self.last_failure_message = '빈 그리퍼 확인·잠금 해제 완료 증거가 일치하지 않습니다.'
            self.status.set(self.last_failure_message)
            return
        self.recovery_verified = self.job_succeeded = True

    def finish_recovery(self):
        """Invalidate reviewed paths only after recovery and clean process exit."""
        paths = {self.trajectory_file}
        paths.update(getattr(self, name) for name in ("air_trajectory_file", "physical_trajectory_file")
                     if hasattr(self, name))
        self.plan_valid = False
        for path in paths:
            path.with_name(path.name+".meta.json").unlink(missing_ok=True)
            path.with_suffix(".gui.json").unlink(missing_ok=True)
        self.plan_manifest_file.unlink(missing_ok=True)
        self.step_seen_requests.clear()
        self.step_latest_index, self.step_total = 0, None
        self.job_plan_id = None
        if not getattr(self,"job_recovery_preserve",False):
            self.visible_layout = None
        self.refresh_run_button()

    def air_path_note(self):
        if getattr(self, "plan_kind", "air") == "physical":
            if self.incremental_enabled():
                return (f"ROS domain {self.domain} · 한 블록 반복 · 집기 접근·하강 {PICKUP_SPEED[0]:.2f} 연속 · "
                        f"운반 속도/가속도 {FAST_LAYERS}층까지 {LOW_TOWER_SPEED[0]:.2f}/{LOW_TOWER_SPEED[1]:.2f}, "
                        f"{FAST_LAYERS+1}층부터 {HIGH_TOWER_SPEED[0]:.2f}/{HIGH_TOWER_SPEED[1]:.2f} · "
                        f"최종 {APPROACH_CLEARANCE*1000:g}mm 저속 놓기 · 탑 위 50mm 통과·후퇴")
            return (("한 블록씩 계획·실행 반복 / 놓은 탑 위 50mm 후퇴 · " if self.incremental_enabled() else "")
                    + f"ROS domain {self.domain} · 실물 집기·쌓기 · 0.73 잡기 명령 완료 후 상승 / 놓기 실측 기반 열림→0.60 · "
                    f"최종 {APPROACH_CLEARANCE*1000:g}mm·접촉 {ARM_SPEEDS['contact']:.2f} / "
                    f"하강 {ARM_SPEEDS['descent_fast']:.2f} / 이동 {ARM_SPEEDS['transfer']:.2f} · 측면 10mm 높게 진입 후 내려놓기")
        if getattr(self, "plate_alignment", False):
            return (f"ROS domain {self.domain} · 모서리 X/Y 두 방향 8회 맞춤 · 그리퍼 {self.ALIGNMENT_HAND:g}rad / 끝은 상판 위 10mm · "
                    "상승 후 90° 회전 · 정지 후 판을 맞추고 손을 뺀 뒤 다음 OK")
        reference = ("플레이트 상판보다" if getattr(self, "plate_alignment", False) else
                     "장애물 상단보다" if self.scan_mode else "블록 지지면보다" if getattr(self, "plate_enabled", False) else
                     "실제 테이블보다")
        return (f"ROS domain {self.domain} · {reference} {self.empty_hand_clearance.get()}mm 이상 여유의 공중 경로 · "
                "RViz의 저장 경로를 확인한 뒤 실제 팔만 시운전")

    def set_scan_mode(self, scan, alignment=False):
        if self.scan_mode != scan or getattr(self, "plate_alignment", False) != alignment:
            self.scan_mode = scan
            self.plate_alignment = alignment
            self.invalidate_plan()
        if hasattr(self, "footer_note"):
            self.footer_note.set(self.air_path_note())
        self.redraw()

    def clearance_changed(self, *_):
        self.invalidate_plan()
        if hasattr(self, "footer_note"):
            self.footer_note.set(self.air_path_note())
        self.redraw()

    def plan_fingerprints(self):
        return {
            "task_kind": getattr(self, "plan_kind", "air"),
            "incremental_stack": self.incremental_enabled() and getattr(self, "plan_kind", "air") == "physical",
            "selection": self.file_hash(self.selection_file),
            "trajectory": self.file_hash(self.trajectory_file),
            "calibration": self.file_hash(self.calibration_file),
            "physical_profile": self.file_hash(self.physical_profile),
        }

    def plan_matches_selection(self):
        try:
            if self.trajectory_file.stat().st_size == 0:
                return False
            return json.loads(self.plan_manifest_file.read_text()) == self.plan_fingerprints()
        except (OSError, ValueError, TypeError):
            return False

    def remember_plan(self):
        try:
            if self.file_hash(self.selection_file) != self.job_selection_hash:
                raise ValueError("계획 중 배치 파일이 바뀌었습니다. 다시 계획하세요.")
            if self.trajectory_file.stat().st_size == 0:
                raise ValueError("저장된 팔 경로가 비어 있습니다.")
            fingerprints = self.plan_fingerprints()
            temporary = self.plan_manifest_file.with_suffix(".tmp")
            temporary.write_text(json.dumps(fingerprints, sort_keys=True))
            temporary.replace(self.plan_manifest_file)
            self.plan_valid = True
            return True
        except (OSError, ValueError) as error:
            self.invalidate_plan()
            self.status.set(f"계획 파일을 확인하지 못했습니다: {error}")
            return False

    def refresh_run_button(self):
        if self.real_backend and hasattr(self, "run_button"):
            enabled = (self.plan_valid and self.trajectory_file.is_file() and
                       not self.job and not self.stop_pending and not self.close_requested and
                       self.calibration_window is None and
                       self.execution_ready())
            self.run_button.configure(state="normal" if enabled else "disabled")
            self.run_button.configure(text="⑤ 실물 집기·쌓기 실행" if getattr(self, "plan_kind", "air") == "physical"
                                      else "⑤ 블록 없는 팔 경로 실행")

    def launch_task(self, action):
        if self.job or self.stop_pending or self.close_requested or self.calibration_window is not None:
            return
        if self.real_backend and self.model_error:
            self.status.set(self.model_error + " · 보정·현장 설정 파일을 확인하고 GUI를 다시 실행하세요.")
            return
        if not self.scene_client.service_is_ready():
            self.status.set("start_block_robot.sh와 같은 ROS domain에서 GUI를 실행하세요." if self.real_backend else
                            "start_block_sim.sh로 가상 로봇과 GUI를 함께 실행하세요. 두 창의 ROS_DOMAIN_ID도 같아야 합니다.")
            return
        if action == "plate_align" and not getattr(self, "plate_enabled", False):
            self.status.set("모서리 홈 맞춤은 플레이트 모드에서 사용하세요.")
            return
        if action in ("physical_plan", "physical_prepare") and not self.real_backend:
            self.status.set("실물 집기·쌓기 계획은 실물 모드에서 사용하세요.")
            return
        if action in ("recovery", "unlock_empty") and not self.real_backend:
            self.status.set("초기 자세 복구는 실물 모드에서 사용하세요.")
            return
        if self.real_backend and action in ("plan", "scan", "plate_align", "physical_plan", "physical_prepare"):
            self.set_plan_kind("physical" if action in ("physical_plan", "physical_prepare") else "air")
        if (action in ("run", "recovery") or self.real_backend and action in ("plan", "scan", "plate_align", "physical_plan")) and not self.execution_ready():
            self.status.set("대기: 팔 제어기 active와 팔 6개·그리퍼 1개 관절의 최신 피드백을 확인하세요." if self.real_backend else
                            "실행 대기: 팔·DH 제어기가 active이고 관절 피드백이 수신되어야 합니다. 위 상태와 demo 터미널을 확인하세요.")
            return
        if action in ("plan", "scan", "plate_align", "physical_plan", "physical_prepare"):
            self.set_scan_mode(action in ("scan", "plate_align"), alignment=action == "plate_align")
        if self.real_backend and action == "plate_align":
            current_hand = getattr(self, "joint_values", {}).get(self.HAND_JOINT, math.nan)
            if not math.isfinite(current_hand) or abs(current_hand-self.ALIGNMENT_HAND) > .003:
                action = "alignment_gripper"
        execution_mode = None
        if action in self.MOTION_ACTIONS:
            try:
                execution_mode = self.selected_execution_mode()
            except ValueError as error:
                self.status.set(str(error))
                return
        assumed_obstacle = False
        if self.real_backend and action in ("plan", "scan", "plate_align"):
            if self.scan_mode and not getattr(self, "plate_alignment", False) and not self.obstacles:
                self.obstacles.add(7)
                self.empty.discard(7)
                if self.target == 7:
                    self.target = 0
                assumed_obstacle = True
                self.redraw()
        requires_target = action in ("plan", "physical_plan") or action == "run" and not self.scan_mode
        if requires_target and not self.target:
            self.status.set("‘쌓을 칸’을 선택한 뒤 원하는 칸을 눌러주세요.")
            return
        try:
            self.save_selection()
        except (ValueError, OSError) as error:
            self.status.set(str(error))
            return
        preparation_id = None
        if action == 'unlock_empty':
            if not messagebox.askokcancel(
                    '빈 그리퍼 확인 · 팔 이동 없음',
                    '실제 그리퍼에 블록이 없는지 확인하세요.\n'
                    '팔·그리퍼를 움직이지 않고 최신 빈 그리퍼 센서와 정지 상태를 확인합니다.\n'
                    'RViz에 부착 블록이 없을 때만 이전 중단 잠금을 해제합니다.\n'
                    '현재 탑·장애물·블록 장면은 유지합니다.',parent=self.root):
                return
            preparation_id = 'unlock_'+uuid.uuid4().hex
        if action == "recovery":
            if not messagebox.askokcancel(
                    "초기 자세 복귀 · 현재 그리퍼 유지",
                    "그리퍼의 현재 단계·열림 각도를 유지하며 팔을 초기 자세로 복귀합니다.\n"
                    "실제 잡고 있는 블록과 RViz의 부착 물체가 일치하는지 확인하세요.\n"
                    "주변 사람과 물체를 치우고 비상정지 버튼을 사용할 준비가 되었는지 확인하세요.\n\n"
                    "관절·그리퍼 피드백과 정지를 확인한 뒤, 현재 충돌 장면으로 복귀 경로를 계획합니다.\n"
                    "그리퍼를 열거나 초기화하지 않으며, 쌓인 탑·남은 블록 장면을 유지합니다.\n"
                    "실제 이동은 경로 확인 후 상단 OK로 승인합니다.",
                    parent=self.root):
                return
            if not self.execution_ready():
                self.status.set("복구 확인 중 연결 상태가 바뀌었습니다. 연결을 확인하고 다시 시도하세요.")
                return
            preparation_id = "recover_"+uuid.uuid4().hex
        if action == "alignment_gripper":
            if not messagebox.askokcancel(
                    "플레이트 맞춤 · 그리퍼 준비",
                    f"실제 AG95 그리퍼를 {self.ALIGNMENT_HAND:g}rad로 움직이는 준비 단계입니다. 팔은 현재 자세를 유지합니다.\n"
                    "그리퍼 주변을 비우고 비상정지 버튼을 사용할 준비가 되었는지 확인하세요.\n\n"
                    "확인 후에도 상단 OK를 눌러야 그리퍼가 움직입니다. 완료되면 모서리 홈 경로를 새로 계획합니다.",
                    parent=self.root):
                return
            if not self.execution_ready():
                self.status.set("준비 확인 중 연결 상태가 바뀌었습니다. 연결을 확인하고 다시 시도하세요.")
                return
            preparation_id = "prep_"+uuid.uuid4().hex
        if self.real_backend and action == "run":
            if not self.plan_valid or not self.plan_matches_selection():
                self.invalidate_plan()
                self.status.set("저장한 계획과 현재 배치·보정·현장 설정이 다릅니다. 팔 경로를 다시 계획하세요.")
                return
            try:
                plan_metadata = json.loads(self.trajectory_file.with_name(self.trajectory_file.name + ".meta.json").read_text())
                plan_id = plan_metadata["plan_id"]
                if not isinstance(plan_id, str) or not plan_id:
                    raise ValueError("plan_id is missing")
                if getattr(self, "plan_kind", "air") == "physical" and plan_metadata.get("path_kind") != "physical_stack":
                    raise ValueError("The saved plan is not a physical stacking recipe")
                if (getattr(self, "plate_alignment", False) and getattr(self, "plan_kind", "air") != "physical"
                        and (plan_metadata.get("alignment_policy") != ALIGNMENT_POLICY
                             or plan_metadata.get("valid_for_seconds") != 1800
                             or not isinstance(plan_metadata.get("alignment_stops"), list)
                             or len(plan_metadata["alignment_stops"]) != 8)):
                    raise ValueError("모서리 X/Y 두 방향 맞춤 경로를 다시 계획하세요.")
            except (OSError, ValueError, TypeError, KeyError):
                self.invalidate_plan()
                self.status.set("저장된 실행 계획 정보를 확인하지 못했습니다. 팔 경로를 다시 계획하세요.")
                return
            physical = getattr(self, "plan_kind", "air") == "physical"
            incremental = physical and self.incremental_enabled()
            path_description = ((f"선택한 공급 블록을 {self.target}번 홈의 바닥 블록 위 2층부터 쌓습니다. "
                                 "잡기 0.73rad 명령이 완료되면 잡힌 것으로 간주하고, 잡힘·이탈 센서 상태로 진행을 막지 않습니다. "
                                 "놓기는 실측 파지각보다 열리는 방향(최대 0.70rad)→0.60rad 순서이며, 손가락을 연 뒤 팔이 후퇴합니다. "
                                 f"놓기 마지막 {APPROACH_CLEARANCE*1000:g}mm 하강·놓은 뒤 처음 {CONTACT_CLEARANCE*1000:g}mm 이탈은 "
                                 f"속도·가속도 {ARM_SPEEDS['contact']:.2f}, 그 위 하강은 {ARM_SPEEDS['descent_fast']:.2f}, "
                                 f"나머지 이동은 설정 한계 {ARM_SPEEDS['transfer']:.2f}입니다. "
                                 "20층부터는 받침면보다 10mm 높게 수평 진입한 뒤 수직으로 내려놓습니다.")
                                if physical else
                                (f"빈 플레이트의 모서리 홈 A→B→C→D에서 X/Y 두 방향으로 각각 정지합니다. 그리퍼는 {self.ALIGNMENT_HAND:g}rad, 끝은 상판보다 10mm 높게 유지합니다. "
                                 "X 맞춤 후 위로 올라가 90° 회전하고 Y를 맞춥니다. 팔이 정지한 뒤에만 판을 조정하고 손을 뺀 다음 경로를 승인하세요.")
                                if getattr(self, "plate_alignment", False) else
                                f"25×25×{self.obstacle_height.get()}mm 장애물의 상단보다 {self.empty_hand_clearance.get()}mm 이상 여유를 두고 "
                                "전체 24칸의 XY를 순회한 뒤 마지막 위치를 유지합니다. 잡기 동작은 없습니다."
                                if self.scan_mode else
                                f"테이블보다 {self.empty_hand_clearance.get()}mm 위의 저장 경로로 움직입니다.")
            if incremental:
                path_description += ("\n현재 준비한 한 블록을 쌓은 뒤, 갱신된 탑·남은 블록 상태에서 다음 한 블록의 경로를 계획·검사합니다. "
                                     f"집기 하강은 {ARM_SPEEDS['descent_fast']:.2f}, 집은 뒤 수직 상승은 1.00 배율로 각각 한 번에 움직입니다. "
                                     "탑 위 50mm와 경로상 높은 장애물을 넘는 높이를 계산한 뒤, 집은 곳에서 먼저 상승해 직접 이동합니다. "
                                     "놓은 후 빈 그리퍼의 가장 낮은 부분이 완성된 탑 위 50mm가 되도록 후퇴합니다. "
                                     "시간 제한은 없으며, 중지 버튼·블록 소진·가능한 직접 집기 경로 없음으로 종료합니다.")
            execution_description = ("단계별 확인: 구간마다 RViz 경로를 보고 상단 OK를 눌러야 다음 한 구간을 움직입니다."
                                     if execution_mode == "step" else
                                     "전체 연속 실행도 X/Y 맞춤 위치마다 반드시 멈춥니다. 총 8개 맞춤 구간을 각각 OK로 승인합니다."
                                     if getattr(self, "plate_alignment", False) else
                                     "전체 연속 실행: 전체 RViz 경로를 보고 상단 OK를 한 번 누르면 끝까지 이어서 움직입니다.")
            if incremental and execution_mode == "continuous":
                execution_description = ("한 블록씩 자동 반복: 첫 블록 경로를 확인하고 상단 OK를 누르면, "
                                         "같은 배치·보정·로봇 설정 안에서 이후 블록도 계획·검사 후 자동 실행합니다. "
                                         "이후 경로마다 OK를 다시 묻지 않습니다. 멈추려면 실행 중지를 누르세요.")
            if not messagebox.askokcancel(
                    "실제 팔·그리퍼 집기·쌓기 준비 확인" if physical else "실제 팔 실행 준비 확인",
                    ("배치와 같은 실제 블록을 놓은 상태에서 UR5e 팔과 AG95 그리퍼가 움직입니다.\n" if physical else
                     "블록이 없는 상태에서 실제 UR5e 팔이 움직입니다.\n") + path_description + "\n"
                    + execution_description + "\n"
                    "주변 사람과 물체를 치우고 비상정지 버튼을 사용할 준비가 되었는지 확인하세요.\n"
                    + ("현재 탑의 높이, 남은 공급 블록·장애물 위치가 RViz 장면과 일치하는지 확인하세요.\n\n" if incremental else
                       "쌓을 홈의 바닥 블록 1개, 공급 블록·장애물 위치가 화면과 일치하는지 확인하세요.\n\n" if physical else
                       "그리퍼의 열림 각도는 유지하며 손가락 구동 명령은 보내지 않습니다.\n\n") +
                    "경로 확인을 시작할까요? 실제 이동은 경로가 표시된 뒤 상단 OK를 눌러 시작합니다.",
                    parent=self.root):
                return
            if not self.execution_ready() or not self.plan_matches_selection():
                self.invalidate_plan()
                self.status.set("확인창이 열린 동안 연결이나 계획 파일이 바뀌었습니다. 연결을 확인하고 다시 계획하세요.")
                return
        if self.real_backend:
            if action != "recovery":
                self.invalidate_plan()
            if action in ("plan", "scan", "plate_align", "physical_plan"):
                try:
                    self.trajectory_file.unlink(missing_ok=True)
                except OSError as error:
                    self.status.set(f"이전 경로 파일을 갱신할 수 없습니다: {error}")
                    return
            try:
                self.job_selection_hash = self.file_hash(self.selection_file)
            except OSError as error:
                self.status.set(f"배치 파일을 읽을 수 없습니다: {error}")
                return
        self.clear_step_request("실행할 경로를 준비 중입니다. 표시된 경로를 확인한 뒤 상단 OK를 누르세요."
                                if action in self.MOTION_ACTIONS else "계획이 완료되면 실행 방식을 선택하고 실행을 시작하세요.")
        self.step_seen_requests.clear()
        self.step_latest_index, self.step_total = 0, None
        self.job_plan_id = (preparation_id if action in ("alignment_gripper", "recovery", "unlock_empty") else
                            plan_id if self.real_backend and action == "run" else None)
        self.pending_alignment_plan = action == "alignment_gripper"
        self.recovery_verified = False
        self.job_execution_mode = execution_mode
        self.job_failed = self.job_succeeded = False
        self.last_failure_message = ""
        self.user_stopped = False
        self.stop_acknowledged = False
        self.stop_backend_confirmed = False
        self.job_action = action
        self.job_recovery_preserve = action == "recovery"
        self.recovery_held_fault_retained = False
        self.job_incremental_mode = (self.incremental_enabled() and getattr(self, "plan_kind", "air") == "physical"
                                     and action in ("physical_plan", "physical_prepare", "run"))
        self.incremental_finished_info = None
        self.alignment_paused = False
        self.alignment_stop = None
        self.visible_layout = None if action == "reset" else yaml.safe_load(self.selection_file.read_text())["/**"]["ros__parameters"]
        if self.real_backend and getattr(self, "plan_kind", "air") != "physical" and action in ("plan", "scan", "plate_align", "alignment_gripper", "run"):
            self.visible_layout["_air_preview"] = True
        if self.real_backend:
            mode = {"plan": "dry_plan", "scan": "dry_plan", "plate_align": "dry_plan",
                    "alignment_gripper": "alignment_prepare", "physical_plan": "physical_plan", "physical_prepare": "physical_prepare", "recovery": "physical_recover", "unlock_empty": "physical_unlock",
                    "run": "physical_run" if getattr(self, "plan_kind", "air") == "physical" else "dry_run"}.get(action, action)
            trajectory_file = (getattr(self, "recovery_trajectory_file", self.folder / "recovery.recipe.json")
                               if action in ("recovery", "unlock_empty") else self.trajectory_file)
            command = ["ros2", "launch", "ur5e_block_bringup", "block_task.launch.py",
                       f"stack_mode:={mode}", f"selection_file:={self.selection_file}",
                       f"trajectory_file:={trajectory_file}", f"calibration_file:={self.calibration_file}",
                       f"physical_profile:={self.physical_profile}"]
            if getattr(self, "plan_kind", "air") == "physical" or action == "recovery":
                command.append(f"recipe_manifest_file:={trajectory_file}")
                command.append(f"incremental_stack:={'true' if self.job_incremental_mode else 'false'}")
            if action == "recovery":
                command.append("recovery_preserve_gripper:=true")
            if action == 'unlock_empty':
                command.extend(['recovery_empty_confirmed:=true',f'preparation_id:={preparation_id}'])
            if action in self.MOTION_ACTIONS:
                # This point is reached only after the actual-motion modal is accepted.
                command.extend(["workspace_clear:=true", f"execution_mode:={execution_mode}"])
                if preparation_id is not None:
                    command.append(f"preparation_id:={preparation_id}")
        else:
            command = ["ros2", "launch", "ur5e_a3_grid", "stack_blocks.launch.py",
                       f"stack_mode:={'plan' if action == 'plate_align' else action}", f"selection_file:={self.selection_file}",
                       f"riser_height:={getattr(self, 'simulation_riser_height', 0.0)}",
                       f"simulation_backend:={getattr(self, 'simulation_backend', 'mock')}"]
            if action == "run":
                command.append(f"execution_mode:={execution_mode}")
        try:
            self.job = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        text=True, start_new_session=True, env={**os.environ, "PYTHONUNBUFFERED": "1"})
        except OSError as error:
            self.status.set(f"작업을 시작하지 못했습니다: {error}")
            return
        self.busy(True)
        height_reference = "장애물 상단보다" if self.scan_mode else "테이블보다"
        self.status.set({"scene": "RViz 배치 반영 중", "unlock_empty": "팔 이동 없이 빈 그리퍼·정지·중단 기록 확인 중",
                         "plate_align": f"모서리 X/Y 두 방향 맞춤 경로 계획 중 · 그리퍼 {self.ALIGNMENT_HAND:g}rad / 끝은 상판 위 10mm · 계획만으로 움직이지 않습니다.",
                         "alignment_gripper": f"그리퍼 {self.ALIGNMENT_HAND:g}rad 준비 경로 확인 중 · 상단 OK를 눌러야 움직입니다.",
                         "physical_plan": "일치하는 저장 경로를 찾는 중 · 없으면 새로 계획·사전 검사합니다. 로봇은 움직이지 않습니다.",
                         "physical_prepare": "현재 정책 경로의 사전 검사 결과를 저장 중 · 로봇은 움직이지 않습니다.",
                         "recovery": "현재 그리퍼·충돌 장면을 유지한 초기 자세 복귀 계획 중 · 상단 OK로 승인해야 움직입니다.",
                         "scan": (("장애물 선택이 없어 7번을 장애물로 가정했습니다. " if assumed_obstacle else "") +
                                  f"장애물 상단보다 {self.empty_hand_clearance.get()}mm 이상 여유로 24칸 XY 경로 계획 중 · 로봇은 움직이지 않습니다.")
                                 if self.real_backend else "",
                         "plan": f"테이블보다 {self.empty_hand_clearance.get()}mm 위 공중 경로 계획 중 · 로봇은 움직이지 않습니다." if self.real_backend else
                                 "모든 층의 팔 경로 검사 중 · 로봇은 움직이지 않습니다.",
                         "run": "실행할 경로 준비 중 · RViz에 경로가 표시되면 상단 OK로 승인하세요." if self.real_backend else
                                "전체 사전 검사 후 경로를 표시합니다. 상단 OK를 눌러야 움직입니다.",
                         "reset": "작업 물체를 지우는 중"}[action])
        logfile = self.folder / f"{time.strftime('%Y%m%d_%H%M%S')}_{action}.log"
        self.job_logfile = logfile
        self.saved_path_file = None
        job = self.job
        def read():
            with logfile.open("w") as stream:
                for line in job.stdout:
                    stream.write(line)
                    stream.flush()
                    self.events.put(("job", line))
            try:
                bundle = archive_from_log(logfile)
                if bundle is not None:
                    line = 'STACK_PATH_ARCHIVED '+json.dumps({'file': str(bundle)}, ensure_ascii=False)+'\n'
                    with logfile.open('a') as stream:
                        stream.write(line)
                    self.events.put(("job", line))
            except (OSError, ValueError, KeyError, TypeError) as error:
                line = 'STACK_ARCHIVE_ERROR '+str(error)+'\n'
                with logfile.open('a') as stream:
                    stream.write(line)
                self.events.put(("job", line))
            self.events.put(("finished", str(logfile)))
        threading.Thread(target=read, daemon=True).start()

    def joint_feedback(self, message):
        if len(message.name) != len(message.position):
            return
        now = time.monotonic()
        required = self.ARM_JOINTS | {self.HAND_JOINT}
        # Independent hardware managers publish different subsets on /joint_states.
        # Track freshness per joint so fresh arm data cannot hide stale fingers.
        for name, position in zip(message.name, message.position):
            if name in required and math.isfinite(position):
                self.joint_values[name] = position
                self.joint_times[name] = now
        if len(message.velocity) == len(message.name):
            for name, velocity in zip(message.name, message.velocity):
                if name in self.ARM_JOINTS and math.isfinite(velocity):
                    self.joint_velocities[name] = velocity
                    self.velocity_times[name] = now

    def fresh_joints(self, names, now=None):
        now = time.monotonic() if now is None else now
        return all(name in self.joint_times and now-self.joint_times[name] < 2 for name in names)

    def controller_state(self, manager, controller):
        info = self.managers[manager]
        return info["states"].get(controller, "미연결") if time.monotonic()-info["checked"] < 5 else "미연결"

    def execution_ready(self):
        if self.real_backend and self.model_error:
            return False
        arm = "scaled_joint_trajectory_controller" if self.real_backend else "ur5e_arm_controller"
        ready = (self.scene_client.service_is_ready() and
                 self.controller_state("/controller_manager", arm) == "active" and
                 self.fresh_joints(self.ARM_JOINTS | {self.HAND_JOINT}))
        if self.real_backend:
            # Air paths freeze the measured fingers; physical stacking commands
            # both controllers and cannot be ready with a missing hand driver.
            return ready and (getattr(self, "plan_kind", "air") != "physical" or self.controller_state(
                "/gripper/controller_manager", "dh_ag95_gripper_controller") == "active")
        return ready and self.controller_state("/controller_manager", "dh_ag95_gripper_controller") == "active"

    def refresh_connection(self):
        now = time.monotonic()
        self.connection.set(f"ROS domain {self.domain} · " +
                            ("MoveIt 연결됨" if self.scene_client.service_is_ready() else "MoveIt 미연결"))
        for info in self.managers.values():
            if not info["client"].service_is_ready():
                if info["future"] is not None:
                    info["future"].cancel()
                    info["future"] = None
                info["states"] = {}
                info["checked"] = 0.0
            elif info["future"] is not None:
                if info["future"].done():
                    try:
                        response = info["future"].result()
                        info["states"] = {item.name: item.state for item in response.controller}
                        info["checked"] = now
                    except Exception:
                        info["states"] = {}
                    info["future"] = None
                elif now-info["requested"] > 4:
                    info["future"].cancel()
                    info["future"] = None
                    info["states"] = {}
            elif now-info["checked"] > 1:
                info["requested"] = now
                info["future"] = info["client"].call_async(ListControllers.Request())
        arm_name = "scaled_joint_trajectory_controller" if self.real_backend else "ur5e_arm_controller"
        hand_manager = "/gripper/controller_manager" if self.real_backend else "/controller_manager"
        arm = self.controller_state("/controller_manager", arm_name)
        hand = self.controller_state(hand_manager, "dh_ag95_gripper_controller")
        arm_feedback = sum(self.fresh_joints({name}, now) for name in self.ARM_JOINTS)
        hand_feedback = "수신 중" if self.fresh_joints({self.HAND_JOINT}, now) else "없음"
        self.hardware_status.set(f"팔 제어기: {arm} · 피드백 {arm_feedback}/6  |  DH 제어기: {hand} · 피드백 {hand_feedback}")
        self.refresh_run_button()

    def stop(self):
        if self.calibration_window is not None:
            self.calibration_window.stop()
            return
        if not self.job and not self.stop_pending:
            return
        self.user_stopped = True
        self.alignment_paused = False
        self.clear_step_request("중지 요청 중 · 경로 승인을 잠갔습니다.")
        self.invalidate_plan()
        if self.real_backend:
            if not self.stop_pending:
                self.stop_pending = True
                self.stop_acknowledged = False
                self.stop_backend_confirmed = False
                self.stop_requested_at = time.monotonic()
                self.stop_stationary_since = None
                self.stop_requires_motion = self.job_action in self.MOTION_ACTIONS
            self.busy(True)
            if self.stop_future is None:
                if self.stop_client.service_is_ready():
                    self.stop_future = self.stop_client.call_async(Trigger.Request())
                    self.stop_call_time = time.monotonic()
                    self.stop_warned = False
                    self.status.set("중지 요청 중 · 응답과 팔 정지 피드백을 기다립니다.")
                else:
                    self.status.set("중지 서비스에 연결되지 않았습니다. 정지 확인 전까지 창을 유지합니다. 중지를 다시 누르세요.")
            return
        self.status.set("중지 요청 중 · 현재 블록 배치는 유지합니다.")
        if self.stop_client.service_is_ready():
            self.stop_client.call_async(Trigger.Request())
        job = self.job
        def interrupt():
            if job.poll() is None:
                os.killpg(job.pid, signal.SIGINT)
        self.root.after(1500, interrupt)

    def show_failure_status(self, progress):
        """Keep the failure cause visible while independent stop proof arrives."""
        message = getattr(self, "last_failure_message", "")
        if not self.job_failed or not message:
            return False
        self.status.set(message + " · " + progress)
        return True

    def refresh_stop(self):
        if not self.real_backend or not self.stop_pending:
            return
        now = time.monotonic()
        if self.stop_future is not None:
            if self.stop_future.done():
                try:
                    reply = self.stop_future.result()
                    self.stop_acknowledged = self.stop_acknowledged or bool(reply.success)
                    if not reply.success and not self.stop_backend_confirmed:
                        self.status.set(f"중지 요청이 수락되지 않았습니다: {reply.message}")
                except Exception as error:
                    self.status.set(f"중지 응답 확인 실패: {error}. 정지 완료로 처리하지 않습니다.")
                self.stop_future = None
            elif now-self.stop_call_time > 5 and not self.stop_warned:
                self.stop_warned = True
                self.status.set("중지 응답이 늦습니다. 응답과 정지 피드백을 계속 기다리며 정지 완료로 처리하지 않습니다.")
        stationary = all(
            name in self.velocity_times and self.velocity_times[name] > self.stop_requested_at and
            now-self.velocity_times[name] < 1 and abs(self.joint_velocities[name]) < .01
            for name in self.ARM_JOINTS)
        if stationary:
            if self.stop_stationary_since is None:
                self.stop_stationary_since = now
        else:
            self.stop_stationary_since = None
        stopped = (not self.stop_requires_motion or
                   self.stop_stationary_since is not None and now-self.stop_stationary_since >= .3)
        confirmed = not self.stop_requires_motion or self.stop_backend_confirmed
        if self.stop_acknowledged and confirmed and self.job is None and stopped:
            # DONE may arrive just before a late stop request reaches a retiring
            # backend. Once completion and stationary feedback are verified,
            # releasing that redundant future cannot leave a motion running.
            if self.stop_future is not None:
                self.stop_future.cancel()
                self.stop_future = None
            self.stop_pending = False
            if not self.show_failure_status("정지 확인 · 현재 자세에서 경로를 다시 계획하세요."):
                self.status.set("팔 정지를 피드백으로 확인했습니다. 현재 자세를 유지합니다." if self.stop_requires_motion else
                                "계획 작업을 중지했습니다. 실제 팔은 움직이지 않았습니다.")
            self.busy(False)

    def process_incremental_event(self, line):
        markers = ("STACK_INCREMENTAL_PLANNING_NEXT", "STACK_INCREMENTAL_PLAN ",
                   "STACK_INCREMENTAL_PROGRESS ", "STACK_INCREMENTAL_FINISHED ")
        marker = next((value for value in markers if value in line), None)
        if marker is None:
            return False
        if (not getattr(self, "job_incremental_mode", False) or self.job is None
                or self.job_failed or self.user_stopped or self.stop_pending or self.close_requested):
            return True
        if marker == markers[0]:
            self.status.set("현재 탑·남은 블록을 유지하며 다음 한 블록의 경로를 계획·검사합니다.")
            self.step_status.set("다음 블록 계획 중 · 현재 팔 자세 유지")
            return True
        raw = line.split(marker, 1)[1].strip()
        # The C++ child also emits a plain diagnostic. Only the parent's
        # structured terminal result can mark a complete incremental session.
        if marker == markers[3] and not raw.startswith("{"):
            return True
        try:
            data = json.loads(raw)
            if marker == markers[1]:
                if (not isinstance(data, dict) or type(data.get("source")) is not int
                        or type(data.get("layer")) is not int or data['layer'] < 2
                        or type(data.get("remaining")) is not int or data['remaining'] < 0):
                    raise ValueError("다음 블록 계획 정보가 올바르지 않습니다.")
                if self.job_action == "run":
                    metadata = json.loads(self.trajectory_file.with_name(self.trajectory_file.name+".meta.json").read_text())
                    if (metadata.get("plan_id") != data.get("plan_id")
                            or metadata.get("recipe_policy") != INCREMENTAL_POLICY
                            or metadata.get("path_kind") != "physical_stack"):
                        raise ValueError("다음 한 블록의 저장 계획 정보를 확인하지 못했습니다.")
                    self.clear_step_request("다음 한 블록 경로 준비 완료")
                    self.step_latest_index, self.step_total = 0, None
                    if getattr(self, "job_execution_mode", None) == "continuous":
                        # This block is covered by the existing session
                        # consent. Track its progress without creating OK.
                        self.step_latest_index, self.step_total = 1, 1
                    self.job_plan_id = data['plan_id']
                self.status.set(f"다음 한 블록 준비: {data['source']}번 → {self.target}번 · {data['layer']}층 · 남을 블록 {data['remaining']}개")
            elif marker == markers[2]:
                self.status.set(f"한 블록 적층 완료 · 현재 {data['layers']}층 · 적재 {data['completed']}개 · 남은 {data['remaining']}개")
            else:
                if (not isinstance(data, dict) or data.get("reason") not in ("no_remaining_blocks", "no_feasible_direct_pick")
                        or type(data.get("layers")) is not int or data['layers'] < 1
                        or type(data.get("remaining")) is not int or data['remaining'] < 0
                        or (data['reason'] == "no_remaining_blocks") != (data['remaining'] == 0)):
                    raise ValueError("한 블록 반복 실행의 종료 정보가 올바르지 않습니다.")
                self.incremental_finished_info = data
                self.job_succeeded = True
                self.status.set(self.incremental_finish_message())
        except (OSError, ValueError, TypeError, KeyError) as error:
            self.job_failed = True
            self.last_failure_message = str(error)
            self.clear_step_request("한 블록 실행 상태 확인 오류 · 실행 중지 후 상태를 확인하세요.")
            self.status.set(self.last_failure_message)
        return True

    def incremental_finish_message(self):
        result = self.incremental_finished_info
        if result['reason'] == "no_remaining_blocks":
            return f"적층 완료 · 현재 {result['layers']}층 · 남은 공급 블록 없음"
        return (f"반복 종료 · 현재 {result['layers']}층 · 남은 {result['remaining']}개는 직접 집기 경로를 찾지 못했습니다. "
                "현재 배치를 유지하고 확인하세요.")

    def poll(self):
        if self.closing:
            return
        if rclpy.ok():
            # Drain a bounded batch: high-rate joint states must not starve
            # controller-service responses in the Tk event loop.
            for _ in range(12):
                rclpy.spin_once(self.node, timeout_sec=0)
            if time.monotonic() - self.last_marker_time > 1:
                self.publish_grid()
                self.last_marker_time = time.monotonic()
        self.refresh_connection()
        for _ in range(300):
            try:
                kind, line = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == "finished":
                if self.job.poll() is None:
                    self.root.after(100, lambda value=line: self.events.put(("finished", value)))
                    continue
                successful = (not self.job_failed and self.job_succeeded and self.job.returncode == 0
                              and (self.job_action not in ("recovery", "unlock_empty") or getattr(self, "recovery_verified", False)))
                if (getattr(self, "job_incremental_mode", False) and self.job_action == "run"
                        and getattr(self, "incremental_finished_info", None) is None):
                    successful = False
                restore_recovery_scene = False
                if (self.job_action in ("recovery", "unlock_empty") and self.job.returncode == 0 and self.job_succeeded
                        and not getattr(self, "recovery_verified", False) and not self.job_failed):
                    self.job_failed = True
                    self.last_failure_message = "초기 자세 복구 완료 증거가 없어 초기화하지 않았습니다. 아래 로그를 확인하세요."
                    self.status.set(self.last_failure_message)
                if self.real_backend and self.job_action in self.MOTION_ACTIONS and not successful and not self.stop_pending:
                    # A failed/terminated client does not prove its remote goal stopped.
                    self.stop_pending = True
                    self.stop_requires_motion = True
                    self.stop_requested_at = time.monotonic()
                    self.stop_stationary_since = None
                    self.stop_acknowledged = self.stop_backend_confirmed
                    self.user_stopped = True
                if self.user_stopped:
                    if self.real_backend:
                        # A completed DONE/PLAN_READY job needs no further cancellation.
                        if successful:
                            self.stop_acknowledged = True
                            self.stop_backend_confirmed = True
                        if not self.show_failure_status("작업 종료 · 실제 팔 정지 피드백 확인 중입니다."):
                            self.status.set("작업 프로세스가 종료되었습니다. 중지 응답과 실제 팔 정지를 확인 중입니다.")
                    elif not self.show_failure_status("작업을 중지했습니다. 현재 블록 배치는 유지합니다."):
                        self.status.set("작업을 중지했습니다. 현재 블록 배치는 유지합니다.")
                elif successful:
                    if (self.job_action == "recovery" and not self.stop_pending
                            and not self.close_requested):
                        try:
                            self.finish_recovery()
                            restore_recovery_scene = not getattr(self,"job_recovery_preserve",False)
                        except OSError as error:
                            self.job_failed = True
                            self.last_failure_message = f"초기 자세 복귀는 완료했지만 이전 계획을 폐기하지 못했습니다: {error}"
                            self.status.set(self.last_failure_message)
                    messages = {"scene": "선택한 블록·장애물을 RViz에 표시했습니다.",
                                "plate_align": "모서리 X/Y 두 방향 맞춤 경로를 준비했습니다. 각 맞춤 위치에서 정지한 뒤 판을 조정하고 OK로 재개하세요.",
                                "alignment_gripper": f"그리퍼 {self.ALIGNMENT_HAND:g}rad 준비 완료 · 현재 자세로 모서리 홈 경로를 계획합니다.",
                                "physical_plan": "실물 저장 경로를 준비했습니다. RViz 확인 후 ⑤와 OK로 실행하세요. 실행 중 경로 충돌 재검사는 생략합니다.",
                                "physical_prepare": "현재 정책 경로의 사전 검사 결과를 별도 보관했습니다. 초기 배치를 맞추고 ④로 불러오세요.",
                                "recovery": "초기 자세·그리퍼 0.4rad 복귀 완료 · 이전 계획을 폐기하고 저장한 배치를 다시 표시합니다.",
                                "unlock_empty": "빈 그리퍼·정지·부착 물체 없음 확인 완료 · 중단 잠금 해제 · 새 배치는 ③, 현재 탑에서 계속하려면 ④를 사용하세요.",
                                "scan": "24칸 XY 순회 경로를 저장했습니다. 장애물 위 전체 저장 경로를 RViz에서 확인한 뒤 실행하세요.",
                                "plan": f"{self.empty_hand_clearance.get()}mm 위 공중 경로를 저장했습니다. RViz에서 전체 저장 경로를 확인한 뒤 실행하세요." if self.real_backend else
                                        "모든 블록의 경로 검사가 통과했습니다. 실행할 수 있습니다.",
                                "run": ("실물 집기·쌓기 동작을 완료했습니다. 바닥 블록 위에 공급 블록을 쌓았습니다."
                                        if self.real_backend and getattr(self, "plan_kind", "air") == "physical" else
                                        f"모서리 X/Y 맞춤 완료 · 그리퍼 {self.ALIGNMENT_HAND:g}rad / 끝은 상판 위 10mm · 마지막 D-Y에서 유지합니다."
                                        if getattr(self, 'plate_alignment', False) else
                                        f"{'장애물 상단보다' if self.scan_mode else '블록 지지면보다' if getattr(self, 'plate_enabled', False) else '테이블보다'} {self.empty_hand_clearance.get()}mm 이상 여유의 공중 경로 시운전을 완료했습니다. 그리퍼 명령은 보내지 않았습니다."
                                        if self.real_backend else "선택한 홈의 바닥 블록 위에 나머지 블록을 쌓았습니다."),
                                "reset": "Gazebo / RViz 작업 물체를 지웠습니다." if getattr(self, "simulation_backend", "mock") == "gazebo" else "RViz 작업 물체를 지웠습니다."}
                    if self.job_action == 'recovery' and getattr(self,'job_recovery_preserve',False):
                        messages['recovery'] = "팔 초기 자세 복귀 완료 · 그리퍼 각도와 탑·블록 장면을 유지했습니다."
                        if self.recovery_held_fault_retained:
                            messages['recovery'] += " 잡힌 블록 관련 중단 기록도 유지합니다."
                    incremental_done = (getattr(self, "job_incremental_mode", False)
                                        and getattr(self, "incremental_finished_info", None) is not None)
                    if incremental_done:
                        self.invalidate_plan()
                        messages[self.job_action] = self.incremental_finish_message()
                    elif getattr(self, "job_incremental_mode", False) and self.job_action == "physical_plan":
                        messages['physical_plan'] = "다음 한 블록 경로를 준비했습니다. RViz 확인 후 실행하세요. 전체 연속 실행은 이후 블록도 계획·검사 후 자동 반복합니다."
                    needs_manifest = (self.real_backend and self.job_action in ("plan", "scan", "plate_align", "physical_plan")
                                      and not incremental_done)
                    if (not self.job_failed and (not needs_manifest or self.remember_plan())):
                        message = messages[self.job_action]
                        if self.job_action == 'run' and getattr(self, 'saved_path_file', None):
                            message += '\n성공 경로 저장: '+self.saved_path_file
                        self.status.set(message)
                elif not self.job_failed:
                    self.status.set("작업이 중지되었거나 완료되지 않았습니다. 아래 로그를 확인하세요.")
                self.alignment_paused = False
                self.clear_step_request("작업이 종료되었습니다. 새로 계획하거나 실행 방식을 선택하세요.")
                replan_alignment = (self.job_action == "alignment_gripper" and successful
                                    and not self.user_stopped and not self.stop_pending
                                    and not self.close_requested and getattr(self, "pending_alignment_plan", False))
                self.pending_alignment_plan = False
                self.job = None
                self.busy(self.stop_pending)
                if replan_alignment:
                    self.root.after(100, lambda: self.launch_task("plate_align"))
                elif (restore_recovery_scene and not self.user_stopped and not self.stop_pending
                      and not self.close_requested):
                    self.root.after(100, lambda: self.launch_task("scene"))
                continue
            self.log.insert("end", line)
            if int(self.log.index("end-1c").split(".")[0]) > 500:
                self.log.delete("1.0", "80.0")
            self.log.see("end")
            if self.process_incremental_event(line):
                continue
            if self.process_step_event(line):
                continue
            if "STACK_PATH_ARCHIVED " in line:
                try:
                    bundle = json.loads(line.split('STACK_PATH_ARCHIVED ', 1)[1])['file']
                    self.saved_path_file = bundle
                    self.status.set('적층 완료 · 성공 경로 저장: '+bundle)
                except (ValueError, KeyError, TypeError):
                    self.status.set('경로 저장 결과를 읽지 못했습니다. 실행 로그를 확인하세요.')
            elif "STACK_ARCHIVE_ERROR " in line:
                self.job_failed = True
                self.last_failure_message = '동작 후 성공 경로 저장 실패: '+line.split('STACK_ARCHIVE_ERROR ', 1)[1].strip()
                self.status.set(self.last_failure_message)
            elif "STACK_EMPTY_UNLOCK_DONE " in line:
                try:
                    self.receive_empty_unlock_done(json.loads(line.split('STACK_EMPTY_UNLOCK_DONE ',1)[1].strip()))
                except (ValueError,TypeError) as error:
                    if self.job_action == 'unlock_empty':
                        self.job_failed = True
                        self.last_failure_message = f'빈 그리퍼 확인 완료 정보를 읽지 못했습니다: {error}'
                        self.status.set(self.last_failure_message)
            elif "STACK_RECOVERY_DONE " in line:
                try:
                    self.receive_recovery_done(json.loads(line.split("STACK_RECOVERY_DONE ", 1)[1].strip()))
                except (ValueError, TypeError) as error:
                    if self.job_action == "recovery":
                        self.job_failed = True
                        self.last_failure_message = f"초기 자세 복구 완료 정보를 읽지 못했습니다: {error}"
                        self.status.set(self.last_failure_message)
            elif "STACK_AIR_EXECUTING" in line:
                self.stop_backend_confirmed = False
                if not self.user_stopped:
                    self.stop_acknowledged = False
            elif "STACK_STOPPED_CONFIRMED" in line:
                self.stop_backend_confirmed = True
                self.stop_acknowledged = True
                if not self.show_failure_status("백엔드 중지 확인 · 실제 팔 정지 피드백 확인 중입니다."):
                    self.status.set("백엔드 중지 확인 · 작업 종료와 팔 정지 피드백을 확인 중입니다.")
            elif "STACK_ERROR " in line:
                self.job_failed = True
                message = line.split("STACK_ERROR ", 1)[1].strip()
                height = re.search(r"Tower needs (\d+) layers.*only (\d+)", message)
                if height:
                    message = f"요청 {height[1]}층 중 {height[2]}층까지 도달·충돌 검사를 통과했습니다. 쌓을 칸·배치 또는 놓는 방식을 변경하세요. 로봇은 움직이지 않았습니다."
                elif "robot models differ" in message:
                    message = ("실제 backend와 GUI의 보정·현장 설정·로봇 모델이 다릅니다. 같은 설정으로 다시 실행하세요."
                               if self.real_backend else
                               "실행 중인 demo의 로봇 모델이 다릅니다. ur5e_ag95_moveit_config demo.launch.py와 같은 workspace를 소싱하세요.")
                elif "MONITOR_PREWARM_TIMEOUT" in message:
                    message = "실행 전 상태 확인 응답이 늦어 작업을 중지했습니다. " + message
                elif "MONITOR_RPC_TIMEOUT" in message:
                    message = "상태 확인 응답 시간 초과로 작업을 중지했습니다. " + message
                self.last_failure_message = message
                self.status.set(message)
            elif "STACK_HEIGHT " in line:
                match = re.search(r"required=(\d+) reachable=(\d+)", line)
                if match:
                    self.status.set(f"높이 검사: 필요 {match[1]}층 / 도달 확인 {match[2]}층")
            elif "STACK_REPLAY_SAVED " in line and self.job_action == "physical_prepare":
                self.job_succeeded = True
                self.status.set("저장 경로 빠른 실행 준비 완료 · 초기 배치를 맞추고 ④로 불러오세요.")
            elif "STACK_REPLAY_CACHE_HIT " in line:
                self.status.set("일치하는 저장 경로를 불러왔습니다. 추가 경로 계산·충돌 재검사 없이 실행을 준비합니다.")
            elif "STACK_REPLAY_PREPARING " in line:
                match = re.search(r"step=(\d+) total=(\d+) samples=(\d+)", line)
                if match:
                    self.status.set(f"기존 경로 1회 사전 검사: {match[1]}/{match[2]}단계 · {match[3]}개 상태 · 로봇은 움직이지 않습니다.")
            elif "STACK_PHYSICAL_VALIDATING " in line:
                match = re.search(r"step=(\d+) total=(\d+) samples=(\d+)", line)
                if (match and (self.job_action == "physical_plan" or getattr(self, "job_incremental_mode", False)) and not self.job_failed
                        and not self.user_stopped and not self.stop_pending):
                    self.status.set(f"실물 경로 충돌 검사: {match[1]}/{match[2]}단계 · {match[3]}개 상태 검사 · 로봇은 움직이지 않습니다.")
            elif "STACK_PLANNING " in line:
                match = re.search(r"source=(\d+) target=(\d+) layer=(\d+)/(\d+)", line)
                if match:
                    self.status.set(f"경로 검사 중: {match[1]}번 → {match[2]}번, {match[3]}/{match[4]}층")
            elif "STACK_TRANSFER_COMPLETED " in line:
                match = re.search(r"block=(\d+) destination=(\d+) temporary=(\d+) moved=(\d+)", line)
                if match and match[3] == '1':
                    self.status.set(f"임시 적재: {match[1]}번 블록을 {match[2]}번 블록 위에 보관 · 목표 적재 {match[4]}개")
            elif "STACK_STATUS " in line:
                phase_match = re.search(r"\bphase=(\w+)", line)
                if phase_match:
                    phase = phase_match[1]
                    if phase in ("PLAN_READY", "DONE") or (phase == "SCENE" and self.job_action in ("scene", "reset")):
                        self.job_succeeded = True
                match = re.search(r"phase=(\w+) moved=(\d+) total=(\d+).*layers=(\d+)", line)
                if match:
                    phase = match[1]
                    if phase == "PLACED":
                        self.status.set(f"쌓기 진행: {match[2]}/{match[3]}개 이동 · 현재 {match[4]}층")
            elif "process has died" in line and kind == "job":
                if not self.job_failed:
                    self.last_failure_message = "작업 프로세스가 종료되었습니다. 아래 오류 로그를 확인하세요."
                    self.status.set(self.last_failure_message)
                self.job_failed = True
        self.refresh_step_approval()
        self.refresh_stop()
        if (self.close_requested and self.calibration_window is None and self.job is None
                and not self.stop_pending and self.stop_future is None):
            self.finish_close()
            return
        self.root.after(100, self.poll)

    def publish_grid(self):
        from geometry_msgs.msg import Point
        import math
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        layout = self.visible_layout
        if getattr(self, "plate_enabled", False) and layout is None:
            try:
                layout = self.current_plate_parameters()
            except (ValueError, TypeError):
                self.marker_pub.publish(array)
                return
        if layout is not None and layout.get("plate_enabled"):
            self.publish_plate_markers(array, layout)
            self.marker_pub.publish(array)
            return
        if layout is not None:
            x, y, _ = layout["center_xyz"]
            z = self.common["table_surface_z"]
            if self.real_backend and layout.get("_air_preview"):
                z += layout["empty_hand_clearance"]
                if layout.get("empty_xy_scan"):
                    z += layout["obstacle_size_xyz"][2]
            width, height = self.common["size_x"], self.common["size_y"]
            yaw = math.radians(self.common.get("grid_yaw_deg", 0.))
            def location(dx, dy, dz):
                return Point(x=x+math.cos(yaw)*dx-math.sin(yaw)*dy,
                             y=y+math.sin(yaw)*dx+math.cos(yaw)*dy, z=z+dz)
            for row in range(self.rows):
                for column in range(self.columns):
                    marker = Marker()
                    marker.header.frame_id = self.common["reference_frame"]
                    marker.header.stamp = self.node.get_clock().now().to_msg()
                    marker.ns, marker.id = "cell_ids", row*self.columns+column+1
                    marker.type, marker.action = Marker.TEXT_VIEW_FACING, Marker.ADD
                    marker.pose.orientation.w = 1.
                    marker.pose.position = location(((column+.5)/self.columns-.5)*width-.023,
                                                    (.5-(row+.5)/self.rows)*height+.023, .012)
                    marker.scale.z = .015
                    marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.
                    marker.text = str(marker.id)
                    array.markers.append(marker)
            grid = Marker()
            grid.header.frame_id = self.common["reference_frame"]
            grid.header.stamp = self.node.get_clock().now().to_msg()
            grid.ns, grid.id = "a3_grid", 0
            grid.type, grid.action = Marker.LINE_LIST, Marker.ADD
            grid.pose.orientation.w = 1.
            grid.scale.x = .001
            grid.color.g, grid.color.b, grid.color.a = .8, 1., 1.
            for column in range(self.columns+1):
                dx = (column/self.columns-.5)*width
                grid.points.extend([location(dx, -height/2, .001), location(dx, height/2, .001)])
            for row in range(self.rows+1):
                dy = (row/self.rows-.5)*height
                grid.points.extend([location(-width/2, dy, .001), location(width/2, dy, .001)])
            array.markers.append(grid)
        self.marker_pub.publish(array)

    def publish_plate_markers(self, array, layout):
        """Draw measured pocket coordinates and corner guides, never collision bodies.

        The STL is the planning scene's collision object. Keeping the GUI's
        plate representation to outlines avoids drawing a second coincident
        triangle mesh over that object.
        """
        from geometry_msgs.msg import Point
        corners = layout["plate_corners_xyz"]
        alignment = layout.get("plate_alignment_points_xyz", corners)
        alignment_ids = layout.get("plate_alignment_cell_ids", [1, self.columns, self.rows*self.columns,
                                                                     (self.rows-1)*self.columns+1])
        cells = layout["cell_centers_xy"]
        top = layout["stack_surface_z"]
        stamp = self.node.get_clock().now().to_msg()

        def marker(namespace, number, kind):
            result = Marker()
            result.header.frame_id = self.common["reference_frame"]
            result.header.stamp = stamp
            result.ns, result.id = namespace, number
            result.type, result.action = kind, Marker.ADD
            result.pose.orientation.w = 1.
            result.color.a = 1.
            return result

        outline = marker("cube_plate_outline", 0, Marker.LINE_STRIP)
        outline.scale.x = .0015
        outline.color.r, outline.color.g, outline.color.b = .5, .8, 1.
        outline.points = [Point(x=corners[3*i], y=corners[3*i+1], z=corners[3*i+2]+.001)
                          for i in (0, 1, 2, 3, 0)]
        array.markers.append(outline)
        colors = ((1., .25, .2), (.2, 1., .3), (.2, .55, 1.), (1., .8, .15))
        for index, name in enumerate("ABCD"):
            point = alignment[3*index:3*index+3]
            guide = marker("cube_plate_corners", index, Marker.CUBE)
            guide.pose.position = Point(x=point[0], y=point[1], z=point[2]+.004)
            guide.scale.x = guide.scale.y = .012
            guide.scale.z = .008
            guide.color.r, guide.color.g, guide.color.b = colors[index]
            array.markers.append(guide)
            text = marker("cube_plate_corner_labels", index, Marker.TEXT_VIEW_FACING)
            text.pose.position = Point(x=point[0], y=point[1], z=point[2]+.055)
            text.scale.z = .020
            text.color.r = text.color.g = text.color.b = 1.
            text.text = f"{name}({alignment_ids[index]})\nX{point[0]*1000:.1f}\nY{point[1]*1000:.1f}\nZ{point[2]*1000:.1f}mm"
            array.markers.append(text)
        for index in range(len(cells)//2):
            text = marker("cell_ids", index+1, Marker.TEXT_VIEW_FACING)
            text.pose.position = Point(x=cells[2*index], y=cells[2*index+1], z=top+.012)
            text.scale.z = .014
            text.color.r = text.color.g = text.color.b = 1.
            text.text = str(index+1)
            array.markers.append(text)
        target_cell = layout.get("target_cell", getattr(self, "target", 0))
        if not isinstance(target_cell, int) or not 1 <= target_cell <= len(cells)//2:
            return
        x, y = cells[2*(target_cell-1):2*target_cell]
        target = marker("cube_plate_stack", 0, Marker.LINE_STRIP)
        target.scale.x = .002
        target.color.r, target.color.g, target.color.b = 1., .75, .1
        target.points = [Point(x=x+dx, y=y+dy, z=top+.0015) for dx, dy in
                         ((-.0125, -.0125), (.0125, -.0125), (.0125, .0125), (-.0125, .0125), (-.0125, -.0125))]
        array.markers.append(target)
        label = marker("cube_plate_stack", 1, Marker.TEXT_VIEW_FACING)
        label.pose.position = Point(x=x, y=y, z=top+.035)
        label.scale.z = .014
        label.color.r, label.color.g, label.color.b = 1., .75, .1
        label.text = f"STACK {target_cell} · 바닥 1개"
        array.markers.append(label)

    def close(self):
        if self.closing or self.close_requested:
            return
        if self.real_backend:
            self.close_requested = True
            if self.calibration_window is not None:
                self.calibration_window.request_close()
                self.busy(True)
                self.status.set("본창을 닫기 전에 그리퍼 보정 중지와 정지 확인을 기다립니다.")
            elif self.job or self.stop_pending:
                self.stop()
                self.status.set("창을 닫기 전에 작업 종료와 실제 팔 정지를 확인합니다.")
            else:
                self.finish_close()
            return
        if self.job:
            self.stop()
        self.closing = True
        self.root.after(500, self.finish_close)

    def finish_close(self):
        if not self.real_backend and self.job and self.job.poll() is None:
            try:
                os.killpg(self.job.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        self.closing = True
        self.clear_step_request()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.root.destroy()


def main():
    # Keep ROS alive while an accepted gripper goal is cancelled and its fresh
    # physical stop is confirmed, including terminal Ctrl+C/termination.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    root = tk.Tk()
    app = StackGui(root)
    signal.signal(signal.SIGINT, lambda *_: root.after(0, app.close))
    signal.signal(signal.SIGTERM, lambda *_: root.after(0, app.close))
    root.mainloop()


if __name__ == "__main__":
    main()
