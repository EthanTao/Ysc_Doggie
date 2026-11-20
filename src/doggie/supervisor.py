"""决策闭环：把检测器、状态机、后端、看门狗串起来。

不依赖 MuJoCo：仿真后端提供固定步长（按步计数调度），实机后端提供时间戳
（按时间调度），两条路径共用同一套决策逻辑。
"""

from __future__ import annotations

from typing import Callable

from .core.detector import FallDetector
from .core.recovery_fsm import RecoveryFSM
from .core.safety import SafetyMonitor
from .core.types import FsmState as S
from .core.types import Mode
from .io.base import RobotBackend


class Supervisor:
    """把检测器、状态机、后端串成闭环。"""

    def __init__(
        self,
        backend: RobotBackend,
        detector: FallDetector,
        fsm: RecoveryFSM,
        logger=None,
        frame_cb=None,
        frame_fps: float = 25.0,
        safety: SafetyMonitor | None = None,
        status_cb: Callable[[dict], None] | None = None,
        status_hz: float = 20.0,
    ) -> None:
        self.backend = backend
        self.detector = detector
        self.fsm = fsm
        self.logger = logger
        self.frame_cb = frame_cb
        self.frame_fps = frame_fps
        self.safety = safety
        self.status_cb = status_cb
        self.status_hz = status_hz
        self.safety_code: str | None = None
        self.safety_fault_t: float | None = None

    @staticmethod
    def _period_steps(hz: float, fixed_dt: float | None) -> int | None:
        if fixed_dt is None:
            return None
        return max(1, int(round(1.0 / (hz * fixed_dt))))

    def _status(self, t: float, obs) -> dict:
        summary = self.fsm.summary()
        return {
            "t": round(t, 4),
            "state": self.fsm.state.value,
            "mode": self.fsm.mode.value,
            "fault_code": self.fsm.fault_code or self.safety_code or "",
            "attempts": self.fsm.attempts,
            "strategy": self.fsm.strategy,
            "detected": bool(self.detector.latched),
            "roll_deg": round(obs.roll_deg, 2),
            "pitch_deg": round(obs.pitch_deg, 2),
            "height_m": round(obs.base_height, 4),
            "feet": obs.feet_contact,
            "latency_s": summary.get("latency_s"),
            "detect_delay_s": summary.get("detect_delay_s"),
        }

    def run(
        self,
        max_time: float,
        det_hz: float = 200.0,
        log_hz: float = 100.0,
        continuous: bool = False,
    ) -> dict:
        """`continuous=True` 用于实机常驻监控：恢复/故障后不退出，直到 max_time。"""
        backend = self.backend
        fixed = backend.fixed_timestep()
        det_every = self._period_steps(det_hz, fixed)
        log_every = self._period_steps(log_hz, fixed)
        status_every = self._period_steps(self.status_hz, fixed)
        frame_every = self._period_steps(self.frame_fps, fixed)
        det_dt = 1.0 / det_hz
        log_dt = 1.0 / log_hz
        status_dt = 1.0 / self.status_hz
        frame_dt = 1.0 / self.frame_fps

        self.detector.reset()
        self.fsm.reset()
        self.safety_code = None
        self.safety_fault_t = None
        if self.safety is not None:
            self.safety.reset()

        t0 = backend.now()
        last_det = last_log = last_status = last_frame = t0
        i = 0
        while True:
            backend.tick()
            i += 1
            t = backend.now()

            do_det = (i % det_every == 0) if det_every else (t - last_det >= det_dt)

            if do_det:
                last_det = t
                obs = backend.read()

                # 1) 安全看门狗优先：通信丢失/阻尼无确认 -> 锁存 + 兜底阻尼
                if self.safety is not None and self.safety_code is None:
                    code = self.safety.check(
                        now=t,
                        obs_t=obs.t,
                        mode=self.fsm.mode,
                        damping_ack=backend.damping_ack(),
                    )
                    if code is not None:
                        self.safety_code = code
                        self.safety_fault_t = t
                        self.fsm.force_fault(t, code)
                        if self.safety.should_estop(code):
                            backend.emergency_stop()
                        backend.set_mode(Mode.DAMPING)

                # 2) 正常决策链路
                if self.safety_code is None:
                    self.detector.update(obs)
                    self.fsm.step(obs, backend)
                    backend.set_mode(self.fsm.mode)

                if self.status_cb is not None:
                    due = (
                        (i % status_every == 0)
                        if status_every
                        else (t - last_status >= status_dt)
                    )
                    if due:
                        last_status = t
                        self.status_cb(self._status(t, obs))

                if not continuous:
                    if (
                        self.fsm.state == S.S0_MONITOR
                        and self.fsm.m.get("t_recovered") is not None
                        and t - self.fsm.m["t_recovered"] > 1.0
                    ):
                        break
                    if (
                        self.fsm.state == S.S5_SAFE_FAULT
                        and t - self.fsm.m.get("t_fault", t) > 1.5
                    ):
                        break
                elif self.safety_code is not None and self.safety is not None:
                    # 实机：故障后保持阻尼一个观察窗口再退出进程
                    if t - (self.safety_fault_t or t) > self.safety.fault_hold_s:
                        break

                if t - t0 > max_time:
                    break

            if self.frame_cb is not None:
                due = (i % frame_every == 0) if frame_every else (t - last_frame >= frame_dt)
                if due:
                    last_frame = t
                    self.frame_cb(t)

            if self.logger is not None:
                due = (i % log_every == 0) if log_every else (t - last_log >= log_dt)
                if due:
                    last_log = t
                    obs = backend.read()
                    self.logger.row(
                        t=round(t, 4),
                        state=self.fsm.state.value,
                        mode=self.fsm.mode.value,
                        roll_deg=round(obs.roll_deg, 2),
                        pitch_deg=round(obs.pitch_deg, 2),
                        height_m=round(obs.base_height, 4),
                        feet=obs.feet_contact,
                        attempts=self.fsm.attempts,
                        fault=self.fsm.fault_code or "",
                        detected=self.detector.latched,
                        lp_roll_deg=round(self.detector.filtered_roll_deg, 2),
                        lp_pitch_deg=round(self.detector.filtered_pitch_deg, 2),
                    )

        return self._summary(max_time)

    def _summary(self, max_time: float) -> dict:
        m = dict(self.fsm.m)
        out = self.fsm.summary()
        out["transitions"] = [
            {"t": tr[0], "from": tr[1], "to": tr[2], "reason": tr[3]}
            for tr in self.fsm.transitions
        ]
        out["timeout"] = "t_recovered" not in m and self.fsm.state != S.S5_SAFE_FAULT
        out["recovered"] = self.fsm.state == S.S0_MONITOR and "t_recovered" in m
        out["safety_fault"] = self.safety_code
        return out
