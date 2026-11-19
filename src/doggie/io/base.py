from __future__ import annotations

from abc import ABC, abstractmethod

from ..core.types import Mode, Observation


class RobotBackend(ABC):
    """机器人硬件/仿真后端统一抽象接口。

    决策层（FallDetector + RecoveryFSM）只依赖本接口与 Observation：
      - 仿真由 SimBackend（MuJoCo）实现；
      - 实机/官方仿真联调由 Ros2Backend（或 SDK 桥接）实现。
    """

    @abstractmethod
    def read(self) -> Observation:
        """读取最新一帧传感器观测（姿态、角速度、12 关节位置/速度、高度/接触）。"""

    @abstractmethod
    def set_mode(self, mode: Mode) -> None:
        """设置底层执行器控制模式（DAMPING / LIEDOWN / STANDUP / HOLD / RL）。"""

    @abstractmethod
    def start_standup(self, t: float) -> None:
        """触发平台 StandUp 起身动作（记录起始关节角与时间戳）。"""

    @abstractmethod
    def start_liedown(self, t: float) -> None:
        """触发平台 LieDown 收腿整姿动作（记录起始关节角与时间戳）。"""

    @abstractmethod
    def reset(self) -> None:
        """重置后端内部状态。"""
