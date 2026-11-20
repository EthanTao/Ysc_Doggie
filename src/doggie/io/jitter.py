"""翻倒注入的随机扰动（用于 stress 套件，默认关闭）。

正式 9 次矩阵保持确定性（`jitter=None`）；压力测试才给初始姿态、关节角与
角速度叠加小扰动，用来回答“9/9 是标称结果还是鲁棒结果”。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class FallJitter:
    """扰动幅度；0 表示不改动该通道。"""

    attitude_deg: float = 2.5
    joint_rad: float = 0.03
    rate_rel: float = 0.10

    @property
    def enabled(self) -> bool:
        return self.attitude_deg > 0.0 or self.joint_rad > 0.0 or self.rate_rel > 0.0


# 与 docs/08-feasibility.md 的 ±2° 扫描同量级，便于跨文档对照。
DEFAULT_STRESS = FallJitter(attitude_deg=2.5, joint_rad=0.03, rate_rel=0.10)


def quat_mul(a, b) -> np.ndarray:
    w1, x1, y1, z1 = (float(v) for v in a)
    w2, x2, y2, z2 = (float(v) for v in b)
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ]
    )


def quat_axis_angle(axis, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    half = 0.5 * angle
    return np.concatenate([[math.cos(half)], math.sin(half) * axis])


def perturb_attitude(rng: np.random.Generator, quat, attitude_deg: float) -> np.ndarray:
    """在世界系额外叠加绕 x/y 的小旋转（不改变 yaw）。"""
    if attitude_deg <= 0.0:
        return np.asarray(quat, dtype=float)
    ax, ay = rng.normal(0.0, math.radians(attitude_deg), 2)
    delta = quat_mul(quat_axis_angle((1.0, 0.0, 0.0), ax), quat_axis_angle((0.0, 1.0, 0.0), ay))
    return quat_mul(delta, quat)


def perturb_joints(rng: np.random.Generator, joints, joint_rad: float) -> np.ndarray:
    q = np.asarray(joints, dtype=float)
    if joint_rad <= 0.0:
        return q.copy()
    return q + rng.normal(0.0, joint_rad, q.size)


def perturb_rates(rng: np.random.Generator, rates, rate_rel: float) -> np.ndarray:
    w = np.asarray(rates, dtype=float)
    if rate_rel <= 0.0:
        return w.copy()
    return w * (1.0 + rng.normal(0.0, rate_rel, w.size))
