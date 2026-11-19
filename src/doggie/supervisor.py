from __future__ import annotations

import numpy as np

from .core.detector import FallDetector
from .core.recovery_fsm import RecoveryFSM
from .core.types import FsmState as S
from .io.sim_backend import SimBackend


class Supervisor:
    """把检测器、状态机、仿真后端串成闭环。"""

    def __init__(
        self,
        backend: SimBackend,
        detector: FallDetector,
        fsm: RecoveryFSM,
        logger=None,
        frame_cb=None,
        frame_fps: float = 25.0,
    ) -> None:
        self.backend = backend
        self.detector = detector
        self.fsm = fsm
        self.logger = logger
        self.frame_cb = frame_cb
        self.frame_every = max(1, int(round(1.0 / (frame_fps * backend.model.opt.timestep))))

    def run(self, max_time: float, det_hz: float = 200.0, log_hz: float = 100.0) -> dict:
        dt = self.backend.model.opt.timestep
        det_every = max(1, int(round(1.0 / (det_hz * dt))))
        log_every = max(1, int(round(1.0 / (log_hz * dt))))

        self.detector.reset()
        self.fsm.reset()
        t0 = self.backend.data.time
        i = 0

        while True:
            self.backend.step()
            i += 1
            t = self.backend.data.time

            if i % det_every == 0:
                obs = self.backend.read()
                self.detector.update(obs)
                self.fsm.step(obs, self.backend)
                self.backend.set_mode(self.fsm.mode)

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

            if self.frame_cb is not None and i % self.frame_every == 0:
                self.frame_cb(t)

            if self.logger is not None and i % log_every == 0:
                obs = self.backend.read()
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

            if t - t0 > max_time:
                break

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
        return out
