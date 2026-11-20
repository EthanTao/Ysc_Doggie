"""官方 Lite3 ROS2 实机后端。

设计约束（工程上刻意为之）：
1. **不在模块级 import rclpy**：没有 ROS2 环境（CI / 开发机）时仍可导入本模块、
   运行单体测试；只有真正构造 Ros2Backend 时才需要 rclpy。
2. **只做适配，不做决策**：所有阈值/状态机逻辑都在 `doggie.core`，本文件只负责
   话题收发、时间戳、高度/触地估计与安全链。
3. **默认不使能任何运动指令**：`run_real.py` 必须显式 `--enable-actuation`；
   后端本身在 `dry_run=True` 时只打印将要发布的指令。

话题/字段映射见 `docs/09-real-machine.md`（与报告表 6-1 一致）。
"""

from __future__ import annotations

import importlib
import json
import time
from typing import Any, Callable

from ..core import kinematics
from ..core.types import Mode, Observation
from . import ros2_convert as conv
from .base import RobotBackend

DEFAULTS: dict[str, Any] = {
    "node_name": "doggie_recovery",
    # 传感器
    "imu_topic": "/IMU_DATA",
    "imu_type": "drdds/msg/ImuData",
    "imu_rpy_in_degrees": True,
    "joints_topic": "/JOINTS_DATA",
    "joints_type": "drdds/msg/JointsData",
    "joint_order": list(kinematics.JOINT_NAMES),
    # 控制
    "state_cmd_topic": "",  # 官方状态机切换话题；留空 = 只能干跑
    "state_cmd_type": "std_msgs/msg/String",
    "state_names": {
        "damping": "kJointDamping",
        "liedown": "kLieDown",
        "standup": "kStandUp",
        "hold": "kIdle",
        "rl": "kRLControl",
    },
    "state_feedback_topic": "",  # 留空 = 阻尼确认不可观测，看门狗不误报
    "estop_topic": "",
    "status_topic": "/doggie/status",
    "reset_service": "/doggie/reset",
    "cmd_resend_s": 0.20,
    "height_source": "fk",  # fk | msg
    "force_threshold_n": 15.0,
    "spin_timeout_s": 0.01,
}


def _import_msg(spec: str):
    """'pkg/msg/Type' 或 'pkg/Type' -> 消息类。"""
    parts = [p for p in spec.split("/") if p]
    if len(parts) == 3:
        pkg, kind, name = parts
    elif len(parts) == 2:
        pkg, name = parts
        kind = "msg"
    else:
        raise ValueError(f"invalid message type spec: {spec!r}")
    module = importlib.import_module(f"{pkg}.{kind}")
    try:
        return getattr(module, name)
    except AttributeError as exc:
        raise ImportError(f"{spec}: {name} not found in {pkg}.{kind}") from exc


class Ros2Backend(RobotBackend):
    def __init__(
        self,
        cfg: dict,
        node=None,
        dry_run: bool = True,
        on_reset: Callable[[float], None] | None = None,
    ) -> None:
        self.cfg = {**DEFAULTS, **(cfg.get("ros2") or {})}
        self.dry_run = bool(dry_run)
        self.on_reset = on_reset
        self.mode = Mode.HOLD
        self.notes: list[str] = []

        self._imu_msg = None
        self._joints_msg = None
        self._imu_rx: float | None = None
        self._joints_rx: float | None = None
        self._cached: Observation | None = None
        self._dirty = True
        self._bias = (0.0, 0.0)

        self._sent_mode: Mode | None = None
        self._sent_at = 0.0
        self._expected_state: str | None = None
        self._feedback_state: str | None = None

        import rclpy  # 延迟导入：仅实机需要

        self._rclpy = rclpy
        self.node = node or rclpy.create_node(self.cfg["node_name"])
        self._owns_node = node is None

        imu_type = _import_msg(self.cfg["imu_type"])
        joints_type = _import_msg(self.cfg["joints_type"])
        self.node.create_subscription(imu_type, self.cfg["imu_topic"], self._on_imu, 20)
        self.node.create_subscription(joints_type, self.cfg["joints_topic"], self._on_joints, 20)

        self._cmd_pub = None
        if self.cfg["state_cmd_topic"]:
            cmd_type = _import_msg(self.cfg["state_cmd_type"])
            self._cmd_pub = self.node.create_publisher(cmd_type, self.cfg["state_cmd_topic"], 10)
        else:
            self.notes.append(
                "ros2.state_cmd_topic 未配置：移动指令不会下发（只能干跑/只读）"
            )

        if self.cfg["state_feedback_topic"]:
            fb_type = _import_msg(self.cfg["state_cmd_type"])
            self.node.create_subscription(
                fb_type, self.cfg["state_feedback_topic"], self._on_state, 10
            )

        self._estop_pub = None
        if self.cfg["estop_topic"]:
            from std_msgs.msg import Bool

            self._estop_pub = self.node.create_publisher(Bool, self.cfg["estop_topic"], 10)

        self._status_pub = self.node.create_publisher(
            _import_msg("std_msgs/msg/String"), self.cfg["status_topic"], 10
        )

        if self.cfg["reset_service"]:
            from std_srvs.srv import Trigger

            self.node.create_service(Trigger, self.cfg["reset_service"], self._srv_reset)

    # ------------------------------------------------------------------ callbacks
    def _on_imu(self, msg) -> None:
        self._imu_msg = msg
        self._imu_rx = time.monotonic()
        self._dirty = True

    def _on_joints(self, msg) -> None:
        self._joints_msg = msg
        self._joints_rx = time.monotonic()
        self._dirty = True

    def _on_state(self, msg) -> None:
        data = getattr(msg, "data", msg)
        self._feedback_state = str(data).strip()

    def _srv_reset(self, _request, response):
        if self.on_reset is None:
            response.success = False
            response.message = "no reset handler bound"
            return response
        self.on_reset(time.monotonic())
        response.success = True
        response.message = "manual reset requested"
        return response

    # ------------------------------------------------------------------ timing
    def tick(self) -> None:
        self._rclpy.spin_once(self.node, timeout_sec=float(self.cfg["spin_timeout_s"]))

    def spin(self, seconds: float = 0.2) -> None:
        """阻塞自旋指定时长（用于退出前把最后一条指令发出去）。"""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._rclpy.spin_once(self.node, timeout_sec=max(0.0, end - time.monotonic()))

    def now(self) -> float:
        return time.monotonic()

    def fixed_timestep(self) -> float | None:
        return None  # 实机按时间戳调度

    def wait_for_data(self, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.tick()
            if self.has_data():
                return True
        return self.has_data()

    def has_data(self) -> bool:
        return self._imu_msg is not None and self._joints_msg is not None

    def set_attitude_bias(self, roll: float, pitch: float) -> None:
        """静态零偏扣除（上电直立静置时采集的姿态均值）。"""
        self._bias = (float(roll), float(pitch))
        self._dirty = True

    def raw_attitude(self) -> tuple[float, float] | None:
        """未扣零偏的 IMU 姿态 (roll, pitch)，用于静态标定。"""
        if self._imu_msg is None:
            return None
        roll, pitch, _ = conv.imu_rpy(
            self._imu_msg, rpy_in_degrees=bool(self.cfg["imu_rpy_in_degrees"])
        )
        return roll, pitch

    # ------------------------------------------------------------------ sense
    def _build(self) -> Observation:
        if self._imu_msg is None or self._joints_msg is None:
            raise RuntimeError("no IMU/JOINTS data received yet; call wait_for_data() first")
        roll, pitch, yaw = conv.imu_rpy(
            self._imu_msg, rpy_in_degrees=bool(self.cfg["imu_rpy_in_degrees"])
        )
        roll -= self._bias[0]
        pitch -= self._bias[1]
        omega = conv.imu_omega(self._imu_msg)

        q = conv.reorder_joints(conv.joint_position(self._joints_msg), self.cfg["joint_order"])
        dq = conv.reorder_joints(conv.joint_velocity(self._joints_msg), self.cfg["joint_order"])
        if q.size != 12:
            raise ValueError(f"expected 12 joint positions, got {q.size}")

        height_override = None
        if self.cfg["height_source"] == "msg":
            raw = conv.first_field(self._joints_msg, "base_height", "height")
            height_override = float(raw) if raw is not None else None

        t = min(self._imu_rx or 0.0, self._joints_rx or 0.0)
        return conv.build_observation(
            t=t,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            omega=omega,
            joint_pos=q,
            joint_vel=dq,
            forces=conv.foot_forces(self._joints_msg),
            force_threshold_n=float(self.cfg["force_threshold_n"]),
            height_override=height_override,
        )

    def read(self) -> Observation:
        if self._dirty or self._cached is None:
            self._cached = self._build()
            self._dirty = False
        return self._cached

    # ------------------------------------------------------------------ modes
    def _publish_state(self, name: str) -> None:
        if self.dry_run:
            print(f"[dry-run] would publish state={name}", flush=True)
            return
        if self._cmd_pub is None:
            raise RuntimeError(
                "actuation requested but ros2.state_cmd_topic is empty; "
                "configure it or run with --dry-run"
            )
        msg = _import_msg(self.cfg["state_cmd_type"])()
        msg.data = name
        self._cmd_pub.publish(msg)

    def set_mode(self, mode: Mode) -> None:
        now = time.monotonic()
        changed = mode != self._sent_mode
        resend = now - self._sent_at >= float(self.cfg["cmd_resend_s"])
        self.mode = mode
        if not changed and not resend:
            return
        if self.dry_run and not changed:
            return  # 干跑时不按重发周期刷屏，只在模式变化时打印
        self._sent_mode = mode
        self._sent_at = now
        name = self.cfg["state_names"].get(mode.value, "")
        self._expected_state = name
        self._publish_state(name)

    def start_standup(self, t: float) -> None:
        self.set_mode(Mode.STANDUP)

    def start_liedown(self, t: float) -> None:
        self.set_mode(Mode.LIEDOWN)

    def reset(self) -> None:
        self._sent_mode = None
        self._feedback_state = None
        self._cached = None
        self._dirty = True

    def damping_ack(self) -> bool | None:
        if not self.cfg["state_feedback_topic"]:
            return None
        if self._expected_state is None:
            return True
        return self._feedback_state is not None and self._feedback_state == self._expected_state

    # ------------------------------------------------------------------ safety / io
    def emergency_stop(self) -> None:
        if self._estop_pub is None:
            print("[safety] EMERGENCY STOP requested but ros2.estop_topic is empty", flush=True)
            return
        from std_msgs.msg import Bool

        msg = Bool()
        msg.data = True
        self._estop_pub.publish(msg)
        print("[safety] EMERGENCY STOP published", flush=True)

    def publish_status(self, payload: dict) -> None:
        msg = _import_msg("std_msgs/msg/String")()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._status_pub.publish(msg)

    def shutdown(self) -> None:
        if self._owns_node:
            self.node.destroy_node()
