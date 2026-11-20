from __future__ import annotations

from abc import ABC, abstractmethod

from ..core.types import Mode, Observation


class RobotBackend(ABC):
    """机器人硬件/仿真后端统一抽象接口。

    决策层（FallDetector + RecoveryFSM）只依赖本接口与 Observation：
      - 仿真由 SimBackend（MuJoCo）实现；
      - 实机/官方仿真联调由 Ros2Backend（或 SDK 桥接）实现。

    时间语义：`now()` 与 `Observation.t` 必须同基准且单调；Supervisor 只通过
    `now()` 计时，不直接访问 MuJoCo 的 data/model，因此可同时驱动仿真与实机。
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

    # ------------------------------------------------------------ timing / safety
    def tick(self) -> None:
        """推进一个控制步：仿真积分一步；实机自旋等待新数据。默认无操作。"""

    def now(self) -> float:
        """决策层使用的单调时间（秒），与 Observation.t 同一基准。"""
        return float(self.read().t)

    def fixed_timestep(self) -> float | None:
        """固定步长（仿真返回 timestep）；实机返回 None，表示按时间戳调度。"""
        return None

    def damping_ack(self) -> bool | None:
        """阻尼指令确认状态：True=已确认，False=待确认，None=不可观测。"""
        return None

    def emergency_stop(self) -> None:
        """独立安全链：通信丢失时硬切发力。默认无操作。"""
