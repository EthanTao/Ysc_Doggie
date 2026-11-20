"""平台无关的几何 / 姿态常量与工具函数（唯一来源）。

这些数值来自官方 Lite3 控制参数：
    pre_height = 0.12 m, stand_height = 0.33 m
    Q_PRE = [0, -1.35453, 2.54948]（每条腿 [HipX, HipY, Knee]）
    thigh = 0.20 m, shank = 0.21 m

仿真后端、恢复状态机、冒烟测试与关键帧脚本都必须从这里引用，
避免同一组参数在多处重复定义后发生漂移。
"""

from __future__ import annotations

import math

import numpy as np

THIGH = 0.20
SHANK = 0.21
PRE_HEIGHT = 0.12
STAND_HEIGHT = 0.33

# 官方预站立（蜷腿）姿态：每条腿 [HipX, HipY, Knee]，共 4 条腿。
Q_PRE = np.array([0.0, -1.35453, 2.54948] * 4, dtype=float)


def expand_pre_pose(values) -> np.ndarray:
    """把 3 维单腿姿态或 12 维四腿姿态统一成 12 维。"""
    arr = np.asarray(values, dtype=float)
    if arr.size == 3:
        return np.tile(arr, 4)
    if arr.size == 12:
        return arr.copy()
    raise ValueError(f"pre_pose_rad must have 3 or 12 values, got {arr.size}")


def hipy_by_height(h: float) -> float:
    return -math.acos((THIGH**2 + h**2 - SHANK**2) / (2.0 * h * THIGH))


def knee_by_height(h: float) -> float:
    return math.pi - math.acos((THIGH**2 + SHANK**2 - h**2) / (2.0 * THIGH * SHANK))


def q_stand(height: float = STAND_HEIGHT) -> np.ndarray:
    """按官方二连杆 IK 求站立关节角。"""
    return np.array([0.0, hipy_by_height(height), knee_by_height(height)] * 4)


def rpy_from_quat(q) -> tuple[float, float, float]:
    """四元数 (w, x, y, z) -> roll, pitch, yaw（rad）。"""
    w, x, y, z = np.asarray(q, dtype=float)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(float(np.clip(2.0 * (w * y - z * x), -1.0, 1.0)))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw
