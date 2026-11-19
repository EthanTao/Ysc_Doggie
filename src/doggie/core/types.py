from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

import numpy as np


class Mode(str, Enum):
    """对齐官方状态机的五种控制模式（Idle / StandUp / LieDown / JointDamping / RLControl）。"""

    DAMPING = "damping"      # JointDamping：纯阻尼，安全状态
    LIEDOWN = "liedown"      # LieDown：官方 liedown_state.hpp（三次样条收腿到 Q_PRE + 阻尼贴地）
    STANDUP = "standup"      # StandUp：官方两阶段站立轨迹
    HOLD = "hold"            # 站立保持（官方 StandUp 结束后保持目标关节角）
    RL = "rl"                # RLControl：策略控制（误报测试用）


class FsmState(str, Enum):
    S0_MONITOR = "S0_MONITOR"
    S1_SAFE_DAMPING = "S1_SAFE_DAMPING"
    S2_ASSESS = "S2_ASSESS"
    S2B_PREPARE_LIEDOWN = "S2B_PREPARE_LIEDOWN"
    S3_STANDUP = "S3_STANDUP"
    S4_VERIFY = "S4_VERIFY"
    S5_SAFE_FAULT = "S5_SAFE_FAULT"


@dataclass
class Observation:
    """决策层唯一依赖的观测（仿真与实机都映射到这个结构）。"""

    t: float
    roll: float                      # rad
    pitch: float                     # rad
    yaw: float                       # rad
    omega: np.ndarray                # 体坐标系角速度 rad/s
    joint_pos: np.ndarray            # (12,)
    joint_vel: np.ndarray            # (12,)
    base_height: float               # m（仿真真值 / 实机估算）
    feet_contact: int                # 触地足数

    @property
    def roll_deg(self) -> float:
        return math.degrees(self.roll)

    @property
    def pitch_deg(self) -> float:
        return math.degrees(self.pitch)


@dataclass
class FallEvent:
    t: float
    kind: str
    cue: str
    roll_deg: float
    pitch_deg: float
    height: float
