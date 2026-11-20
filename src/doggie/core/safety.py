"""通信与阻尼确认看门狗（与平台无关，仿真下恒不触发）。

对应 `config/recovery.yaml` 的 `safety` 段，也是报告 6.2 节承诺的双重看门狗：

- `backend_heartbeat_timeout_s`：传感器观测超过该时限没有更新 -> `E_SENSOR_STALE`；
- `damping_ack_timeout_s`：切入阻尼后超过该时限没有收到底层确认 -> `E_DAMPING_NO_ACK`；
  只有后端能观测到确认状态时才启用（无法观测时不误报，见 `RobotBackend.damping_ack`）。

任一判据触发后锁存，直到人工复位（`reset()`）。
"""

from __future__ import annotations

from .types import Mode


class SafetyMonitor:
    def __init__(self, cfg: dict) -> None:
        self.heartbeat_timeout = float(cfg.get("backend_heartbeat_timeout_s", 0.50))
        self.damping_ack_timeout = float(cfg.get("damping_ack_timeout_s", 0.50))
        self.estop_on_comm_loss = bool(cfg.get("estop_on_comm_loss", True))
        self.fault_hold_s = float(cfg.get("fault_hold_s", 2.0))
        self.reset()

    def reset(self) -> None:
        self.code: str | None = None
        self.t_fault: float | None = None
        self._damping_since: float | None = None

    def should_estop(self, code: str | None = None) -> bool:
        return self.estop_on_comm_loss and code == "E_SENSOR_STALE"

    def _latch(self, code: str, now: float) -> str:
        self.code = code
        self.t_fault = now
        return code

    def check(
        self,
        *,
        now: float,
        obs_t: float,
        mode: Mode,
        damping_ack: bool | None,
    ) -> str | None:
        """返回故障码或 None。`damping_ack=None` 表示后端不可观测确认状态。"""
        if self.code is not None:
            return self.code

        if now - obs_t > self.heartbeat_timeout:
            return self._latch("E_SENSOR_STALE", now)

        if mode == Mode.DAMPING and damping_ack is False:
            if self._damping_since is None:
                self._damping_since = now
            elif now - self._damping_since > self.damping_ack_timeout:
                return self._latch("E_DAMPING_NO_ACK", now)
        else:
            self._damping_since = None

        return None
