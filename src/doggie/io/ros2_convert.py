"""官方 Lite3 ROS2 消息 -> 通用 Observation 的解析（纯函数，不依赖 rclpy）。

不同版本的官方驱动字段名不完全一致，这里按「首个别名命中」的方式兼容：

- 姿态：`rpy` / `roll,pitch,yaw`（度）或 `orientation`（四元数）
- 角速度：`omega` / `angular_velocity` / `gyro`
- 关节：`position` / `joint_position` / `q`，速度 `velocity` / `joint_velocity` / `dq`
- 足端力：`foot_force` / `contact_force` / `forces`（四足法向力，N）

解析失败会抛出携带字段名的 `ValueError`，便于现场定位话题字段差异。
"""

from __future__ import annotations

import math

import numpy as np

from ..core import kinematics
from ..core.types import Observation


def first_field(msg, *names):
    for name in names:
        if "." in name:
            obj = msg
            for part in name.split("."):
                obj = getattr(obj, part, None)
                if obj is None:
                    break
            if obj is not None:
                return obj
        else:
            value = getattr(msg, name, None)
            if value is not None:
                return value
    return None


def quat_to_euler(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    """四元数 (w,x,y,z) -> (roll, pitch, yaw)，rad。"""
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-9:
        return 0.0, 0.0, 0.0
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def imu_rpy(msg, rpy_in_degrees: bool = True) -> tuple[float, float, float]:
    rpy = first_field(msg, "rpy", "RPY", "euler", "euler_angles")
    if rpy is not None and len(rpy) >= 2:
        roll, pitch = float(rpy[0]), float(rpy[1])
        yaw = float(rpy[2]) if len(rpy) > 2 else 0.0
    else:
        roll = first_field(msg, "roll")
        pitch = first_field(msg, "pitch")
        yaw = first_field(msg, "yaw")
        if roll is None or pitch is None:
            quat = first_field(msg, "orientation", "quaternion", "quat")
            if quat is None:
                raise ValueError(
                    "IMU message has neither rpy/roll-pitch-yaw nor orientation; "
                    "check ros2.imu_type / field names"
                )
            qx, qy, qz, qw = float(quat.x), float(quat.y), float(quat.z), float(quat.w)
            return quat_to_euler(qw, qx, qy, qz)
        roll, pitch = float(roll), float(pitch)
        yaw = float(yaw) if yaw is not None else 0.0
    if rpy_in_degrees:
        roll, pitch, yaw = math.radians(roll), math.radians(pitch), math.radians(yaw)
    return roll, pitch, yaw


def imu_omega(msg) -> np.ndarray:
    vec = first_field(msg, "omega", "angular_velocity", "gyro", "gyroscope", "angular_rate")
    if vec is None:
        raise ValueError("IMU message has no omega/angular_velocity field")
    if hasattr(vec, "x"):
        return np.array([float(vec.x), float(vec.y), float(vec.z)])
    return np.asarray(vec, dtype=float)[:3]


def joint_position(msg) -> np.ndarray:
    vec = first_field(msg, "position", "joint_position", "joint_pos", "q", "positions")
    if vec is None:
        raise ValueError("joint message has no position/joint_position field")
    return np.asarray(vec, dtype=float).ravel()


def joint_velocity(msg) -> np.ndarray:
    vec = first_field(msg, "velocity", "joint_velocity", "joint_vel", "dq", "velocities")
    if vec is None:
        return np.zeros(len(joint_position(msg)))
    return np.asarray(vec, dtype=float).ravel()


def joint_torque(msg) -> np.ndarray | None:
    vec = first_field(msg, "torque", "joint_torque", "tau", "effort")
    if vec is None:
        return None
    return np.asarray(vec, dtype=float).ravel()


def reorder_joints(vec: np.ndarray, msg_order: list[str]) -> np.ndarray:
    """把消息顺序的关节量重排成 canonical 顺序（kinematics.JOINT_NAMES）。"""
    if msg_order is None or len(msg_order) != len(vec) or list(msg_order) == list(
        kinematics.JOINT_NAMES
    ):
        return vec
    index = {name: i for i, name in enumerate(msg_order)}
    missing = [n for n in kinematics.JOINT_NAMES if n not in index]
    if missing:
        raise ValueError(f"ros2.joint_order is missing joints: {missing}")
    return np.array([vec[index[n]] for n in kinematics.JOINT_NAMES], dtype=float)


def foot_forces(msg) -> np.ndarray | None:
    vec = first_field(msg, "foot_force", "contact_force", "foot_forces", "forces")
    if vec is None:
        return None
    arr = np.asarray(vec, dtype=float).ravel()
    if arr.size == 12:  # 每腿三个关节力矩/力时，按腿求和
        arr = arr.reshape(4, 3).sum(axis=1)
    return arr[:4] if arr.size >= 4 else None


def build_observation(
    *,
    t: float,
    roll: float,
    pitch: float,
    yaw: float,
    omega: np.ndarray,
    joint_pos: np.ndarray,
    joint_vel: np.ndarray,
    forces: np.ndarray | None = None,
    force_threshold_n: float = 15.0,
    height_override: float | None = None,
) -> Observation:
    """把 At 的原始量组装成 Observation；高度/触地缺省用几何估计。"""
    if height_override is not None:
        height = float(height_override)
    else:
        height = kinematics.base_height_from_q(joint_pos, roll, pitch)

    if forces is not None:
        feet = int(np.count_nonzero(forces >= force_threshold_n))
    else:
        feet = kinematics.feet_contact_from_q(joint_pos, roll, pitch)

    return Observation(
        t=float(t),
        roll=roll,
        pitch=pitch,
        yaw=yaw,
        omega=np.asarray(omega, dtype=float),
        joint_pos=np.asarray(joint_pos, dtype=float),
        joint_vel=np.asarray(joint_vel, dtype=float),
        base_height=height,
        feet_contact=feet,
    )
