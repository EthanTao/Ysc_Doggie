"""Lite3 腿部几何与正运动学（纯 Python + NumPy，供实机与仿真共用）。

杆长引用 `core.params`（官方控制参数：THIGH=0.20 m、SHANK=0.21 m）；髋部安装
位置与关节轴取自官方 MJCF `sim/models/lite3/mjcf/Lite3.xml`：

    FL/HL_HIP pos      = (∓0.1745, +0.062, 0)
    FR/HR_HIP pos      = (∓0.1745, -0.062, 0)
    THIGH 相对 HIP 偏移 = (0, ±0.09735, 0)
    FOOT 球半径         = 0.022  m
    关节轴              = HipX(-x), HipY(-y), Knee(-y)

实机上没有躯干高度真值传感器，`base_height_from_q()` 用关节角正解出四个足端
在机身坐标系的位置，再按 IMU 姿态把地面法向投影到机身系，取最低足端的
垂直距离作为躯干离地高度。该方法在仿真中与 `data.qpos[2]` 的偏差见
`tests/test_kinematics.py`。
"""

from __future__ import annotations

import math

import numpy as np

from .params import SHANK, THIGH

LEGS = ("FL", "FR", "HL", "HR")
JOINT_PARTS = ("HipX", "HipY", "Knee")
JOINT_NAMES = tuple(f"{leg}_{part}_joint" for leg in LEGS for part in JOINT_PARTS)

HIP_POS = {
    "FL": (0.1745, 0.062, 0.0),
    "FR": (0.1745, -0.062, 0.0),
    "HL": (-0.1745, 0.062, 0.0),
    "HR": (-0.1745, -0.062, 0.0),
}
HIP_LATERAL = {"FL": 0.09735, "FR": -0.09735, "HL": 0.09735, "HR": -0.09735}
# 杆长统一引用 core.params（官方控制参数）；MJCF 中 SHANK 为 0.21012 m，
# 0.12 mm 的差异对高度估计与门限判定无影响。
THIGH_LEN = THIGH
SHANK_LEN = SHANK
FOOT_RADIUS = 0.022


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def foot_position(leg: str, q: np.ndarray) -> np.ndarray:
    """单腿足端球心在机身系的位置。q = [HipX, HipY, Knee]（rad）。"""
    q1, q2, q3 = (float(v) for v in q[:3])
    # HipX 轴为 -x，HipY/Knee 轴为 -y（见官方 MJCF）
    local = np.array([0.0, HIP_LATERAL[leg], 0.0]) + _ry(-q2) @ (
        np.array([0.0, 0.0, -THIGH_LEN]) + _ry(-q3) @ np.array([0.0, 0.0, -SHANK_LEN])
    )
    return np.array(HIP_POS[leg]) + _rx(-q1) @ local


def foot_positions(joint_pos: np.ndarray) -> np.ndarray:
    """12 关节角 -> (4, 3) 足端球心位置，行顺序同 LEGS。"""
    q = np.asarray(joint_pos, dtype=float).reshape(4, 3)
    return np.stack([foot_position(leg, q[i]) for i, leg in enumerate(LEGS)])


def up_axis_body(roll: float, pitch: float) -> np.ndarray:
    """世界竖直方向在机身系中的表示（忽略 yaw）：n_b = R_wb^T e_z。"""
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    return np.array([-sp, sr * cp, cr * cp])


def base_height_from_q(joint_pos: np.ndarray, roll: float, pitch: float) -> float:
    """躯干离地高度估计（米）：取离地最近的足端球心的垂直投影距离。

    与仿真约定一致（`qpos[2]` 也是到足端球心支撑面的距离），因此足端半径
    不再重复扣除；相对 0.10 m / 0.25 m 两个门限，0.022 m 的球半径不影响判定。
    """
    feet = foot_positions(joint_pos)
    n_b = up_axis_body(roll, pitch)
    return float(max(0.0, np.min(-(feet @ n_b))))


def feet_contact_from_q(
    joint_pos: np.ndarray, roll: float, pitch: float, tol: float = 0.03
) -> int:
    """无足端力传感器时的几何触地估计：足端低于机身支撑面 tol 内计为触地。"""
    feet = foot_positions(joint_pos)
    n_b = up_axis_body(roll, pitch)
    depth = -(feet @ n_b) - FOOT_RADIUS
    return int(np.count_nonzero(depth <= depth.min() + tol))
