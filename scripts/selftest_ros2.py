#!/usr/bin/env python3
"""ROS2 链路自检：用真实 rclpy/DDS 在单进程里模拟官方话题，验证「话题 → 判定 → 状态机」。

不连接机器人、不下发运动指令（干跑）：本脚本自己是 `/IMU_DATA`、`/JOINTS_DATA`
的发布者，同时用 `Ros2Backend` 订阅，跑一遍完整的
「直立 → 侧翻 → 阻尼 → 收腿 → 起身 → 验证」流程。

用途：上机前 5 分钟确认 ROS2 环境、消息类型、字段解析、时间戳调度都正常。

    python scripts/selftest_ros2.py
    python scripts/selftest_ros2.py --duration 15 --rate 200
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from doggie.core.detector import FallDetector  # noqa: E402
from doggie.core.params import Q_PRE, q_stand  # noqa: E402
from doggie.core.recovery_fsm import RecoveryFSM  # noqa: E402
from doggie.core.safety import SafetyMonitor  # noqa: E402
from doggie.core.types import Mode  # noqa: E402
from doggie.io.ros2_backend import Ros2Backend  # noqa: E402
from doggie.supervisor import Supervisor  # noqa: E402
from doggie.utils.logger import load_config  # noqa: E402

FALL_AT = 0.5
RAMP_S = 0.3
FALL_ROLL_DEG = 85.0


def quat_from_roll_pitch(roll: float, pitch: float = 0.0):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return (cr * cp, sr * cp, cr * sp, -sr * sp)  # w, x, y, z


class Feeder:
    """按时间脚本生成仿真的传感器数据并发布到官方话题。"""

    def __init__(self, node, joints_type, imu_type, rate: float):
        self.node = node
        self.rate = rate
        self.imu_pub = node.create_publisher(imu_type, "/IMU_DATA", 10)
        self.joints_pub = node.create_publisher(joints_type, "/JOINTS_DATA", 10)
        self._standup_t0: float | None = None
        self._upright = False
        self._imu_type = imu_type
        self._joints_type = joints_type

    def observe_mode(self, mode: Mode, t: float) -> None:
        if mode == Mode.STANDUP and self._standup_t0 is None:
            self._standup_t0 = t

    def pose(self, t: float):
        if self._standup_t0 is not None and t - self._standup_t0 >= 3.0:
            self._upright = True
        if self._upright:
            return 0.0, 0.0, q_stand()
        if t < FALL_AT:
            return 0.0, 0.0, q_stand()
        frac = min(1.0, (t - FALL_AT) / RAMP_S)
        roll = math.radians(FALL_ROLL_DEG) * frac
        return roll, 0.0, Q_PRE

    def publish(self, t: float) -> None:
        roll, pitch, q = self.pose(t)
        imu = self._imu_type()
        imu.header.stamp = self.node.get_clock().now().to_msg()
        imu.header.frame_id = "base"
        w, x, y, z = quat_from_roll_pitch(roll, pitch)
        imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z = w, x, y, z
        self.imu_pub.publish(imu)

        js = self._joints_type()
        js.header = imu.header
        from doggie.core.kinematics import JOINT_NAMES

        js.name = list(JOINT_NAMES)
        js.position = [float(v) for v in q]
        js.velocity = [0.0] * 12
        self.joints_pub.publish(js)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=12.0)
    ap.add_argument("--rate", type=float, default=200.0)
    args = ap.parse_args()

    import rclpy
    from sensor_msgs.msg import Imu, JointState

    rclpy.init()
    node = rclpy.create_node("doggie_ros2_selftest")
    cfg = load_config(ROOT / "config" / "recovery.yaml")
    cfg["ros2"].update(
        {
            "imu_type": "sensor_msgs/msg/Imu",
            "joints_type": "sensor_msgs/msg/JointState",
            "state_cmd_topic": "",  # 自检不下发运动指令
            "reset_service": "",
        }
    )

    backend = Ros2Backend(cfg, node=node, dry_run=True)
    feeder = Feeder(node, JointState, Imu, args.rate)
    detector = FallDetector(cfg["detector"])
    fsm = RecoveryFSM(cfg["fsm"], detector)
    safety = SafetyMonitor(cfg["safety"])

    print("等待自检话题回环 ...")
    deadline = time.perf_counter() + 5.0
    while time.perf_counter() < deadline:
        feeder.publish(0.0)
        backend.tick()
        if backend.has_data():
            break
    if not backend.has_data():
        raise SystemExit("自检失败：没有收到自己发布的话题，检查 ROS_DOMAIN_ID / DDS 环境")

    print("数据链路正常，开始脚本化侧翻 ...\n")
    det_every = max(1, int(round(args.rate / cfg["detector"]["rate_hz"])))
    period = 1.0 / args.rate
    last_modes: list[tuple[float, Mode]] = []
    t = 0.0
    i = 0
    next_tick = time.perf_counter()
    while t < args.duration:
        feeder.publish(t)
        backend.tick()
        if i % det_every == 0:
            obs = backend.read()
            code = safety.check(
                now=backend.now(), obs_t=obs.t, mode=fsm.mode, damping_ack=backend.damping_ack()
            )
            if code:
                print(f"[safety] {code}")
                break
            detector.update(obs)
            prev = fsm.state
            fsm.step(obs, backend)
            if fsm.state != prev:
                ev = fsm.strategy or ""
                print(f"  t={t:5.2f}s  {prev.value:22s} -> {fsm.state.value:22s} {ev}")
            if fsm.mode != (last_modes[-1][1] if last_modes else None):
                last_modes.append((t, fsm.mode))
                print(f"  t={t:5.2f}s  mode -> {fsm.mode.value}")
            feeder.observe_mode(fsm.mode, t)
            if fsm.state.value == "S0_MONITOR" and "t_recovered" in fsm.m:
                break
        i += 1
        t = i * period
        next_tick += period
        delay = next_tick - time.perf_counter()
        if delay > 0:
            time.sleep(delay)

    result = fsm.summary()
    print("\n===== 自检结果 =====")
    print(
        f"判定 {result['detected']} | cue={result.get('fall_cue','')} | "
        f"确认时延 {result.get('detect_delay_s')} s | 策略 {result.get('recovery_strategy','')}"
    )
    print(
        f"先安全后起身 {result.get('order_safe_before_standup')} | "
        f"尝试 {result['attempts']} | 最终 {result['final_state']} | 故障 {result['fault_code']}"
    )
    ok = bool(result["detected"] and result.get("order_safe_before_standup"))
    print("\nPASS" if ok else "\nFAIL")
    backend.shutdown()
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
