from __future__ import annotations

import math

import numpy as np

from ..core.detector import FallDetector
from ..core.types import FsmState as S
from ..core.types import Mode, Observation

DEFAULT_Q_PRE = np.array([0.0, -1.35453, 2.54948] * 4, dtype=float)


class RecoveryFSM:
    """恢复状态机（纯逻辑）。

    强制顺序：
      S0 --摔倒确认--> S1(安全阻尼，最短 settle_min_s) --> S2(姿态评估分流)
      --> [S2B(官方 LieDown 收腿整姿，按需)] --> S3(官方 StandUp 起身)
      --> S4(直立验证) --> S0。

    S2_ASSESS 四路姿态分流（全部复用官方状态，无自定义底层轨迹）：
      1. unrecoverable_supine：仰翻/倒立（|roll|>=120° 或 |pitch|>=80°），超出
         平台 StandUp 恢复包络 -> 直接转 S5_SAFE_FAULT (E_UNRECOVERABLE_POSE)；
      2. side_prop_liedown：侧撑未倒平（35°<=max(|roll|,|pitch|)<120°）
         -> 先转 S2B 调用官方 LieDown 收腿撤掉侧撑支点，再转 S3_STANDUP；
      3. tuck_then_standup：机身已趴平（<35°），但四肢散开/不对称
         （max|q - q_pre| >= 0.35 rad）-> 先转 S2B 调用官方 LieDown 对齐四腿，
         再转 S3_STANDUP；
      4. direct_standup：标准趴地且四腿已接近 Q_PRE -> 直接转 S3_STANDUP。
    """

    def __init__(self, fsm_cfg: dict, detector: FallDetector) -> None:
        self.detector = detector
        self.settle_min = float(fsm_cfg.get("settle_min_s", 0.5))
        self.settle_max = float(fsm_cfg.get("settle_max_s", 3.0))
        self.settle_omega = float(fsm_cfg.get("settle_omega_rad_s", 0.15))
        self.settle_omega_body = float(fsm_cfg.get("settle_omega_body_rad_s", 0.15))
        self.quiet_sustain = float(fsm_cfg.get("quiet_sustain_s", 0.3))
        self.monitor_mode = (
            Mode.DAMPING if fsm_cfg.get("monitor_mode", "damping") == "damping" else Mode.HOLD
        )
        self.standup_traj = float(fsm_cfg.get("standup_traj_s", 3.0))
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

        # S2_ASSESS 姿态分流参数
        self.assess_prone_deg = float(fsm_cfg.get("assess_prone_deg", 35.0))
        self.assess_supine_roll_deg = float(fsm_cfg.get("assess_supine_roll_deg", 120.0))
        self.assess_supine_pitch_deg = float(fsm_cfg.get("assess_supine_pitch_deg", 80.0))
        self.assess_joint_tol = float(fsm_cfg.get("assess_joint_tol_rad", 0.35))
        self.liedown_traj = float(fsm_cfg.get("liedown_traj_s", 1.5))
        self.liedown_settle = float(fsm_cfg.get("liedown_settle_s", 0.3))
        pre = np.asarray(fsm_cfg.get("pre_pose_rad", DEFAULT_Q_PRE), dtype=float)
        if pre.size == 3:
            pre = np.tile(pre, 4)
        self.q_pre = pre

        self.reset()

    # ------------------------------------------------------------------ state
    def reset(self) -> None:
        self.state = S.S0_MONITOR
        self.mode = Mode.HOLD
        self.attempts = 0
        self.fault_code: str | None = None
        self.strategy: str = ""
        self.recoveries: list[float] = []
        self.transitions: list[tuple] = []
        self.m: dict = {}          # metrics / timestamps
        self._s1_t0 = 0.0
        self._s2_t0 = 0.0
        self._ld_t0 = 0.0
        self._su_t0 = 0.0
        self._v_enter = 0.0
        self._v_ok_since: float | None = None
        self._quiet_since: float | None = None

    def manual_reset(self, t: float = 0.0) -> bool:
        """人工复位接口（对应 /doggie/reset）：仅在 S5_SAFE_FAULT 下生效，清除故障锁存并回到 S0_MONITOR。"""
        if self.state != S.S5_SAFE_FAULT:
            return False
        self._go(S.S0_MONITOR, "manual_reset", t)
        self.mode = self.monitor_mode
        self.attempts = 0
        self.fault_code = None
        self.strategy = ""
        self.detector.clear()
        return True

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

    def select_strategy(self, obs: Observation) -> tuple[str, float]:
        """根据静止后的机体姿态与关节角偏差选择最合适的官方恢复路径。"""
        abs_r = abs(obs.roll_deg)
        abs_p = abs(obs.pitch_deg)
        q_err = float(np.max(np.abs(obs.joint_pos - self.q_pre)))

        if abs_r >= self.assess_supine_roll_deg or abs_p >= self.assess_supine_pitch_deg:
            return "unrecoverable_supine", q_err
        if abs_r >= self.assess_prone_deg or abs_p >= self.assess_prone_deg:
            return "side_prop_liedown", q_err
        if q_err >= self.assess_joint_tol:
            return "tuck_then_standup", q_err
        return "direct_standup", q_err

    def _enter_standup(self, t: float, backend, reason: str) -> None:
        self.m[f"t_standup_cmd_{self.attempts}"] = t
        if backend is not None and hasattr(backend, "start_standup"):
            backend.start_standup(t)
        self._su_t0 = t
        self._go(S.S3_STANDUP, reason, t)
        self.mode = Mode.STANDUP

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
                strategy, q_err = self.select_strategy(obs)
                self.strategy = strategy
                self.m["recovery_strategy"] = strategy
                self.m["assess_pose"] = {
                    "roll_deg": round(obs.roll_deg, 1),
                    "pitch_deg": round(obs.pitch_deg, 1),
                    "height_m": round(obs.base_height, 3),
                    "feet": obs.feet_contact,
                    "q_pre_max_err_rad": round(q_err, 3),
                    "strategy": strategy,
                }
                self._go(S.S2_ASSESS, f"settled:{strategy}", t)

        elif self.state == S.S2_ASSESS:
            self.mode = Mode.DAMPING
            if self.attempts >= self.max_attempts:
                self._fault(t, "E_RETRY_EXHAUSTED")
                return
            wait = 0.0 if self.attempts == 0 else self.backoff
            if t - self._s2_t0 < wait:
                return

            strategy, q_err = self.select_strategy(obs)
            self.strategy = strategy
            self.m["recovery_strategy"] = strategy
            self.m.setdefault("assess_pose", {})["strategy"] = strategy
            self.m["assess_pose"]["q_pre_max_err_rad"] = round(q_err, 3)

            if strategy == "unrecoverable_supine":
                # 四脚朝天/倒立：超出官方 StandUp 恢复包络，直接安全锁存并报警
                self._fault(t, "E_UNRECOVERABLE_POSE")
                return

            self.attempts += 1
            if strategy in ("side_prop_liedown", "tuck_then_standup"):
                self.m[f"t_liedown_cmd_{self.attempts}"] = t
                if backend is not None and hasattr(backend, "start_liedown"):
                    backend.start_liedown(t)
                self._ld_t0 = t
                self._go(S.S2B_PREPARE_LIEDOWN, f"attempt_{self.attempts}:{strategy}", t)
                self.mode = Mode.LIEDOWN
            else:
                self._enter_standup(t, backend, f"attempt_{self.attempts}:{strategy}")

        elif self.state == S.S2B_PREPARE_LIEDOWN:
            self.mode = Mode.LIEDOWN
            elapsed = t - self._ld_t0
            if elapsed >= self.liedown_traj + self.liedown_settle:
                # 官方 LieDown 收腿整姿完成：若意外翻成仰面则拒止，否则无缝转入官方 StandUp
                if (
                    abs(obs.roll_deg) >= self.assess_supine_roll_deg
                    or abs(obs.pitch_deg) >= self.assess_supine_pitch_deg
                ):
                    self._fault(t, "E_UNRECOVERABLE_POSE")
                else:
                    self._enter_standup(
                        t, backend, f"attempt_{self.attempts}:liedown_ready"
                    )

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
            "recovery_strategy": self.m.get("recovery_strategy", ""),
            "order_safe_before_standup": (
                t_damping is not None and t_standup is not None and t_damping < t_standup
            ),
        }
        if "t_fall_confirm" in self.m:
            out["latency_s"] = round(self.m["t_fall_confirm"], 3)
        if "t_recovered" in self.m and t_standup is not None:
            out["standup_duration_s"] = round(self.m["t_recovered"] - t_standup, 2)
        return out
