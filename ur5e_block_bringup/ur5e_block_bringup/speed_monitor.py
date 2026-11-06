"""Read-only joint velocities and estimated TCP speeds; no command clients."""
import argparse
from collections import deque
import json
import math
import os
import time

ARM = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')
LABELS = ('베이스', '어깨', '팔꿈치', '손목 1', '손목 2', '손목 3')
STALE_SECONDS = .5


class PositionSpeed:
    """Position difference over approximately 100 ms, in the reported frame."""
    def __init__(self):
        self.samples = deque(maxlen=64)
        self.received = -math.inf
        self.frame = None

    def add(self, stamp, xyz, frame, now):
        if not frame or not all(math.isfinite(x) for x in (stamp, *xyz)) or stamp <= 0:
            return
        if self.frame != frame or (self.samples and
                (stamp < self.samples[-1][0] or stamp-self.samples[-1][0] > STALE_SECONDS)):
            self.samples.clear()
        if self.samples and stamp == self.samples[-1][0]:
            return
        self.frame, self.received = frame, now
        self.samples.append((stamp, tuple(xyz)))
        while len(self.samples) > 2 and stamp-self.samples[1][0] >= .1:
            self.samples.popleft()

    def velocity(self, now):
        if now-self.received > STALE_SECONDS or len(self.samples) < 2:
            return None
        (first, a), (last, b) = self.samples[0], self.samples[-1]
        if not .05 <= last-first <= STALE_SECONDS:
            return None
        return tuple((y-x)/(last-first) for x, y in zip(a, b))


def arm_values(message):
    """Ignore partial/gripper-only messages on the shared /joint_states topic."""
    names = list(message.name)
    if any(name not in names for name in ARM) or len(set(names)) != len(names):
        return None
    indexes = [names.index(name) for name in ARM]
    if max(indexes) >= min(len(message.position), len(message.velocity)):
        return None
    q = tuple(float(message.position[i]) for i in indexes)
    dq = tuple(float(message.velocity[i]) for i in indexes)
    return (q, dq) if all(math.isfinite(x) for x in (*q, *dq)) else None


def grasp_velocity(model, q, dq):
    """Central directional derivative J(q) qdot at the model's grasp_link (m/s)."""
    dt = .0001
    ahead = model.tcp(dict(zip(ARM, (x+v*dt for x, v in zip(q, dq)))))
    behind = model.tcp(dict(zip(ARM, (x-v*dt for x, v in zip(q, dq)))))
    return tuple(float(x) for x in (ahead[:3, 3]-behind[:3, 3])/(2*dt))


def make_node():
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Float64, String

    class SpeedMonitor(Node):
        def __init__(self):
            super().__init__('ur5e_speed_monitor', enable_rosout=False,
                             start_parameter_services=False)
            self.arm = None
            self.arm_received = -math.inf
            self.arm_stamp = None
            self.scale = None
            self.scale_received = -math.inf
            self.tcp = PositionSpeed()
            self.model = None
            self.model_error = '로봇 모델 수신 대기'
            self.model_xml = None
            sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.subscribers = [
                self.create_subscription(JointState, '/joint_states', self.on_joint, sensor),
                self.create_subscription(PoseStamped, '/tcp_pose_broadcaster/pose', self.on_pose, sensor),
                self.create_subscription(Float64, '/speed_scaling_state_broadcaster/speed_scaling', self.on_scale, sensor),
                self.create_subscription(String, '/robot_description', self.on_model, latched),
            ]

        def on_joint(self, message):
            values = arm_values(message)
            stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
            if values is not None and stamp > 0 and stamp != self.arm_stamp:
                self.arm, self.arm_stamp = values, stamp
                self.arm_received = time.monotonic()

        def on_pose(self, message):
            p = message.pose.position
            stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
            self.tcp.add(stamp, (p.x, p.y, p.z), message.header.frame_id, time.monotonic())

        def on_scale(self, message):
            if math.isfinite(message.data):
                # This workspace's UR broadcaster already publishes percent.
                self.scale, self.scale_received = message.data, time.monotonic()

        def on_model(self, message):
            if message.data == self.model_xml:
                return
            self.model_xml = message.data
            try:
                from .air_scene import RobotKinematics
                self.model = RobotKinematics(message.data)
                self.model_error = ''
            except (ValueError, ImportError) as error:
                self.model = None
                self.model_error = '집기점 모델 사용 불가: '+str(error)

        def snapshot(self):
            now = time.monotonic()
            arm = self.arm if now-self.arm_received <= STALE_SECONDS else None
            grasp = None
            if arm and self.model:
                try:
                    grasp = grasp_velocity(self.model, *arm)
                except ValueError as error:
                    self.model_error = str(error)
            return dict(
                joint_rad_s=dict(zip(ARM, arm[1])) if arm else None,
                joint_age_s=round(now-self.arm_received, 3) if self.arm else None,
                grasp_model_velocity_m_s=grasp,
                ur_tcp_pose_velocity_m_s=self.tcp.velocity(now),
                ur_tcp_frame=self.tcp.frame,
                speed_scaling_percent=self.scale if now-self.scale_received <= STALE_SECONDS else None,
                model_status=self.model_error,
            )

    return SpeedMonitor()


def run_gui(node, domain, spin):
    import tkinter as tk
    from tkinter import ttk
    root = tk.Tk()
    root.title(f'UR5e 실시간 속도 · 읽기 전용 · ROS {domain}')
    root.geometry('840x625')
    root.minsize(780, 590)
    root.configure(bg='#edf2f7')
    style = ttk.Style(root)
    style.configure('TLabel', font=('Noto Sans CJK KR', 10))
    frame = ttk.Frame(root, padding=20)
    frame.pack(fill='both', expand=True)
    ttk.Label(frame, text='UR5e 실시간 속도', font=('Noto Sans CJK KR', 19, 'bold')).pack(anchor='w')
    ttk.Label(frame, text=f'ROS domain {domain} · 표시 10회/초 · 센서 수신이 끊기면 — 표시').pack(anchor='w', pady=(2, 14))
    cards = ttk.Frame(frame)
    cards.pack(fill='x')
    numbers = {}
    for column, (key, title, detail) in enumerate((
            ('grasp', '블록 집기점 속도', 'grasp_link · 관절 피드백 + 모델 계산'),
            ('tcp', 'UR 설정 TCP 속도', 'TCP 위치 변화 · 약 0.1초 구간 추정'))):
        card = ttk.LabelFrame(cards, text=title, padding=12)
        card.grid(row=0, column=column, sticky='nsew', padx=(0, 8))
        cards.columnconfigure(column, weight=1)
        numbers[key] = tk.StringVar(value='— mm/s')
        ttk.Label(card, textvariable=numbers[key], font=('Noto Sans CJK KR', 24, 'bold')).pack(anchor='w')
        ttk.Label(card, text=detail, font=('Noto Sans CJK KR', 9)).pack(anchor='w')
    scaling = tk.StringVar(value='적용 속도 배율: —')
    ttk.Label(frame, textvariable=scaling, font=('Noto Sans CJK KR', 12, 'bold')).pack(anchor='w', pady=(12, 4))
    ttk.Label(frame, text='배율은 실제 이동 속도가 아닙니다. 집기점과 UR 설정 TCP는 서로 다른 지점일 수 있습니다.',
              font=('Noto Sans CJK KR', 9)).pack(anchor='w')
    table = ttk.Frame(frame)
    table.pack(fill='x', pady=12)
    for col, title in enumerate(('관절', '실측 속도 (rad/s)', '실측 속도 (°/s)')):
        ttk.Label(table, text=title, font=('Noto Sans CJK KR', 10, 'bold')).grid(row=0, column=col, sticky='w', padx=8, pady=4)
        table.columnconfigure(col, weight=1)
    joint_labels = []
    for row, (name, label) in enumerate(zip(ARM, LABELS), 1):
        ttk.Label(table, text=f'{label}  /  {name}').grid(row=row, column=0, sticky='w', padx=8, pady=3)
        values = (tk.StringVar(value='—'), tk.StringVar(value='—'))
        for col, value in enumerate(values, 1):
            ttk.Label(table, textvariable=value).grid(row=row, column=col, sticky='e', padx=8)
        joint_labels.append(values)
    status = tk.StringVar(value='토픽 수신 대기')
    ttk.Label(frame, textvariable=status, foreground='#175a86').pack(anchor='w', pady=(0, 6))
    ttk.Label(frame, text='/joint_states  ·  /tcp_pose_broadcaster/pose\n/speed_scaling_state_broadcaster/speed_scaling',
              font=('Noto Sans CJK KR', 9)).pack(anchor='w')
    ttk.Button(frame, text='창 닫기', command=root.destroy).pack(anchor='e', pady=(8, 0))

    def tick():
        # Depth-one queues + bounded polling keep the observer inexpensive.
        deadline = time.monotonic()+.006
        for _ in range(16):
            spin()
            if time.monotonic() >= deadline:
                break
        data = node.snapshot()
        for key, field in (('grasp', 'grasp_model_velocity_m_s'), ('tcp', 'ur_tcp_pose_velocity_m_s')):
            velocity = data[field]
            numbers[key].set(f'{math.sqrt(sum(x*x for x in velocity))*1000:.1f} mm/s' if velocity is not None else '— mm/s')
        factor = data['speed_scaling_percent']
        scaling.set(f'적용 속도 배율: {factor:.1f} %' if factor is not None else '적용 속도 배율: —')
        for name, (rad, deg) in zip(ARM, joint_labels):
            value = (data['joint_rad_s'] or {}).get(name)
            rad.set(f'{value:+.4f}' if value is not None else '—')
            deg.set(f'{math.degrees(value):+.2f}' if value is not None else '—')
        text = ('관절 피드백 수신 중' if data['joint_rad_s'] else '관절 피드백 대기 / 0.5초 이상 지연')
        if data['model_status']:
            text += ' · '+data['model_status']
        status.set(text)
        root.after(100, tick)

    root.after(0, tick)
    root.lift()
    root.mainloop()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, default=int(os.environ.get('ROS_DOMAIN_ID', '0')))
    parser.add_argument('--console', action='store_true', help='Print JSON once per second instead of opening a window')
    parser.add_argument('--duration', type=float, default=0., help='Console duration in seconds; 0 means unlimited')
    args = parser.parse_args(argv)
    if not 0 <= args.domain <= 232 or not math.isfinite(args.duration) or args.duration < 0:
        parser.error('Invalid domain or duration')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    import rclpy
    rclpy.init(args=[])
    node = make_node()
    try:
        if args.console:
            began = next_print = time.monotonic()
            while rclpy.ok() and (not args.duration or time.monotonic()-began < args.duration):
                rclpy.spin_once(node, timeout_sec=.05)
                if time.monotonic() >= next_print:
                    print(json.dumps(node.snapshot(), ensure_ascii=False, allow_nan=False), flush=True)
                    next_print = time.monotonic()+1
        else:
            run_gui(node, args.domain, lambda: rclpy.spin_once(node, timeout_sec=0.))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
