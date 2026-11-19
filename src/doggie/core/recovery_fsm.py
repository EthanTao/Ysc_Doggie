from __future__ import annotations

import math

import numpy as np

from ..core.detector import FallDetector
from ..core.types import FsmState as S
from ..core.types import Mode, Observation


class RecoveryFSM:
    """恢复状态机（纯逻辑）。

    强制顺序：S0 --摔倒确认--> S1(阻尼，最短 settle_min_s) --> S2 --> S3(起身)
    --> S4(验证) --> S0。任何一次起身失败：有剩余次数则回 S1 重试，
    次数用尽进 S5(安全故障，锁存，只能人工复位)。
    所有切换都记录时间戳与原因，作为「先安全后起身」的证据。
    """

    def __init__(self, fsm_cfg: dict, detector: FallDetector) -> None:
        self.detector = detector
        self.settle_min = float(fsm_cfg.get("settle_min_s", 0.5))
        self.settle_max = float(fsm_cfg.get("settle_max_s", 3.0))
        self.settle_omega = float(fsm_cfg.get("settle_omega_rad_s", 0.15))
        self.settle_omega_body = float(fsm_cfg.get("settle_omega_body_rad_s", 0.15))
        self.quiet_sustain = float(fsm_cfg.get("quiet_sustain_s", 0.3))
        self.monitor_mode = Mode.DAMPING if fsm_cfg.get("monitor_mode", "damping") == "damping" else Mode.HOLD
        self.standup_traj = float(fsm_cfg.get("standup_traj_s", 4.0))
        self.standup_timeout = float(fsm_cfg.get("standup_timeout_s", 20.0))
        self.max_attempts = int(fsm_cfg.get("max_attempts", 2))
        self.backoff = float(fsm_cfg.get("backoff_s", 2.0))
        self.verify_s = float(fsm_cfg.get("verify_s", 1.0))
        self.verify_timeout = float(fsm_cfg.get("verify_timeout_s", 5.0))
        self.upright_roll = math.radians(float(fsm_cfg.get("upright_roll_deg", 10.0)))
        self.upright_pitch = math.radians(float(fsm_cfg.get("upright_pitch_deg", 10.0)))
        self.upright_height = float(fsm_cfg.get("upright_height_m", 0.25))
        self.max_rec_per_min = int(fsm_cfg.get("max_recoveries_per_min", 3))
        self.require_manual_reset = bool(fsm_cfg.get("require_manual_reset", True))
        self.reset()

    # ------------------------------------------------------------------ state
    def reset(self) -> None:
        self.state = S.S0_MONITOR
        self.mode = Mode.HOLD
        self.attempts = 0
        self.fault_code: str | None = None
        self.recoveries: list[float] = []
        self.transitions: list[tuple] = []
        self.m: dict = {}          # metrics / timestamps
        self._s1_t0 = 0.0
        self._s2_t0 = 0.0
        self._su_t0 = 0.0
        self._v_enter = 0.0
        self._v_ok_since: float | None = None
        self._quiet_since: float | None = None

    def _go(self, new: S, reason: str, t: float) -> None:
        self.transitions.append((round(t, 4), self.state.value, new.value, reason))
        self.state = new

    def _fault(self, t: float, code: str) -> None:
        self.fault_code = code
        self.m["t_fault"] = t
        self.m["fault_code"] = code
        self._go(S.S5_SAFE_FAULT, code, t)
        self.mode = Mode.DAMPING

    def _fail_attempt(self, t: float, code: str) -> None:
        self.m[f"t_attempt_{self.attempts}_end"] = t
        self.m[f"attempt_{self.attempts}_result"] = code
        if self.attempts >= self.max_attempts:
            self._fault(t, code)
        else:
            self._go(S.S1_SAFE_DAMPING, "retry_after_" + code, t)
            self._s1_t0 = t
            self._quiet_since = None
            self.mode = Mode.DAMPING

    def _upright(self, obs: Observation) -> bool:
        return (
            abs(obs.roll) < self.upright_roll
            and abs(obs.pitch) < self.upright_pitch
            and obs.base_height > self.upright_height
        )

    # ------------------------------------------------------------------ loop
    def step(self, obs: Observation, backend=None) -> None:
        t = obs.t

        if self.state == S.S0_MONITOR:
            self.mode = self.monitor_mode
            if self.detector.latched:
                ev = self.detector.event
                self.m["t_fall_confirm"] = ev.t if ev else t
                self.m["fall_kind"] = ev.kind if ev else "?"
                self.m["fall_cue"] = ev.cue if ev else "?"
                self._s1_t0 = t
                self.m["t_damping"] = t
                self._go(S.S1_SAFE_DAMPING, f"fall_confirmed:{self.m['fall_cue']}", t)
                self.mode = Mode.DAMPING

        elif self.state == S.S1_SAFE_DAMPING:
            self.mode = Mode.DAMPING
            quiet_now = (
                float(np.max(np.abs(obs.joint_vel))) < self.settle_omega
                and float(np.max(np.abs(obs.omega))) < self.settle_omega_body
            )
            if quiet_now:
                if self._quiet_since is None:
                    self._quiet_since = t
            else:
                self._quiet_since = None
            elapsed = t - self._s1_t0
            quiet = (
                self._quiet_since is not None
                and t - self._quiet_since >= self.quiet_sustain
            )
            if (elapsed >= self.settle_min and quiet) or elapsed >= self.settle_max:
                self._s2_t0 = t
                self.m["assess_pose"] = {
                    "roll_deg": round(obs.roll_deg, 1),
                    "pitch_deg": round(obs.pitch_deg, 1),
                    "height_m": round(obs.base_height, 3),
                    "feet": obs.feet_contact,
                }
                self._go(S.S2_ASSESS, "settled", t)

        elif self.state == S.S2_ASSESS:
            self.mode = Mode.DAMPING
            if self.attempts >= self.max_attempts:
                self._fault(t, "E_RETRY_EXHAUSTED")
                return
            wait = 0.0 if self.attempts == 0 else self.backoff
            if t - self._s2_t0 >= wait:
                self.attempts += 1
                self.m[f"t_standup_cmd_{self.attempts}"] = t
                if backend is not None:
                    backend.start_standup(t)
                self._su_t0 = t
                self._go(S.S3_STANDUP, f"attempt_{self.attempts}", t)
                self.mode = Mode.STANDUP

        elif self.state == S.S3_STANDUP:
            self.mode = Mode.STANDUP
            elapsed = t - self._su_t0
            if elapsed >= self.standup_traj + 0.2:
                # 官方语义：完整跑完两阶段轨迹后再判断结果，不在中途切目标
                if self._upright(obs):
                    self._v_enter = t
                    self._v_ok_since = None
                    self._go(S.S4_VERIFY, "upright", t)
                else:
                    self._fail_attempt(t, "E_STANDUP_NOT_UPRIGHT")
            elif elapsed > self.standup_timeout:
                self._fail_attempt(t, "E_STANDUP_TIMEOUT")

        elif self.state == S.S4_VERIFY:
            self.mode = Mode.HOLD
            if self._upright(obs):
                if self._v_ok_since is None:
                    self._v_ok_since = t
                if t - self._v_ok_since >= self.verify_s:
                    self.m["t_recovered"] = t
                    self.recoveries.append(t)
                    self.detector.clear()
                    self._go(S.S0_MONITOR, "recovered", t)
                    recent = [tr for tr in self.recoveries if t - tr <= 60.0]
                    if len(recent) > self.max_rec_per_min:
                        self._fault(t, "E_TOO_MANY_RECOVERIES")
            else:
                self._v_ok_since = None
                if t - self._v_enter >= self.verify_timeout:
                    self._fail_attempt(t, "E_VERIFY_TIMEOUT")

        elif self.state == S.S5_SAFE_FAULT:
            self.mode = Mode.DAMPING
            # 锁存：只能人工复位（require_manual_reset=True 时本状态机不会自动退出）

    # ------------------------------------------------------------------ report
    def summary(self) -> dict:
        t_standup = self.m.get("t_standup_cmd_1")
        t_damping = self.m.get("t_damping")
        out = {
            "final_state": self.state.value,
            "attempts": self.attempts,
            "fault_code": self.fault_code,
            "detected": self.detector.latched or "t_fall_confirm" in self.m,
            "fall_kind": self.m.get("fall_kind", ""),
            "fall_cue": self.m.get("fall_cue", ""),
            "order_safe_before_standup": (
                t_damping is not None and t_standup is not None and t_damping < t_standup
            ),
        }
        if "t_fall_confirm" in self.m:
            out["latency_s"] = round(self.m["t_fall_confirm"], 3)
        if "t_recovered" in self.m and t_standup is not None:
            out["standup_duration_s"] = round(self.m["t_recovered"] - t_standup, 2)
        return out
