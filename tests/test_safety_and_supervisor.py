"""安全看门狗与「后端无关 Supervisor」的回归测试。

这些用例不加载 MuJoCo：用一个按时间推进的假后端证明决策闭环在实机语义
（无固定步长、无 MjModel、按时间戳调度）下同样可以跑通。
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from doggie.core.detector import FallDetector
from doggie.core.kinematics import JOINT_NAMES  # noqa: F401  (导入即校验常量)
from doggie.core.recovery_fsm import RecoveryFSM
from doggie.core.safety import SafetyMonitor
from doggie.core.types import FsmState as S, Mode, Observation
from doggie.io.base import RobotBackend
from doggie.io.sim_backend import Q_PRE, q_stand
from doggie.supervisor import Supervisor

DET_CFG = {
    "rate_hz": 200,
    "lowpass_cutoff_hz": 10.0,
    "roll_threshold_deg": 50.0,
    "pitch_threshold_deg": 50.0,
    "fast_path_deg": 75.0,
    "fast_path_dwell_s": 0.15,
    "dwell_s": 0.30,
    "height_min_m": 0.10,
    "height_dwell_s": 0.30,
}
FSM_CFG = {
    "monitor_mode": "damping",
    "settle_min_s": 0.5,
    "settle_max_s": 3.0,
    "settle_omega_rad_s": 0.15,
    "settle_omega_body_rad_s": 0.15,
    "quiet_sustain_s": 0.3,
    "standup_traj_s": 3.0,
    "standup_timeout_s": 20.0,
    "max_attempts": 2,
    "backoff_s": 2.0,
    "verify_s": 1.0,
    "verify_timeout_s": 5.0,
    "upright_roll_deg": 10.0,
    "upright_pitch_deg": 10.0,
    "upright_height_m": 0.25,
    "max_recoveries_per_min": 3,
    "assess_prone_deg": 35.0,
    "assess_supine_roll_deg": 120.0,
    "assess_supine_pitch_deg": 80.0,
    "assess_joint_tol_rad": 0.35,
    "liedown_traj_s": 1.5,
    "liedown_settle_s": 0.3,
}
SAFETY_CFG = {
    "backend_heartbeat_timeout_s": 0.5,
    "damping_ack_timeout_s": 0.5,
    "estop_on_comm_loss": True,
    "fault_hold_s": 1.0,
}


class ScriptedBackend(RobotBackend):
    """按时间戳推进的假实机后端：无 model、无固定步长。"""

    def __init__(self, dt: float = 0.005, fall_at: float = 0.4, obs_lag: float = 0.0):
        self.dt = dt
        self.fall_at = fall_at
        self.obs_lag = obs_lag
        self._t = 0.0
        self.mode = Mode.HOLD
        self.mode_log: list[tuple[float, Mode]] = []
        self.estop = False
        self._standup_t0: float | None = None
        self._liedown_t0: float | None = None
        self._upright = False

    # --- RobotBackend ---
    def tick(self) -> None:
        self._t += self.dt

    def now(self) -> float:
        return self._t

    def set_mode(self, mode: Mode) -> None:
        if mode != self.mode:
            self.mode_log.append((self._t, mode))
            if mode == Mode.STANDUP:
                self._standup_t0 = self._t
            if mode == Mode.LIEDOWN:
                self._liedown_t0 = self._t
        self.mode = mode

    def start_standup(self, t: float) -> None:
        self.set_mode(Mode.STANDUP)

    def start_liedown(self, t: float) -> None:
        self.set_mode(Mode.LIEDOWN)

    def reset(self) -> None:
        pass

    def emergency_stop(self) -> None:
        self.estop = True

    def _fallen(self) -> bool:
        if self._upright:
            return False
        if self._standup_t0 is not None and self._t - self._standup_t0 >= 3.2:
            self._upright = True
            return False
        return self._t >= self.fall_at

    def read(self) -> Observation:
        t = self._t - self.obs_lag
        fallen = self._fallen()
        if fallen:
            roll, pitch, h = math.radians(90.0), 0.0, 0.05
            q = Q_PRE.copy()
        else:
            roll, pitch, h = 0.0, 0.0, 0.33
            q = q_stand()
        return Observation(
            t=t,
            roll=roll,
            pitch=pitch,
            yaw=0.0,
            omega=np.zeros(3),
            joint_pos=q,
            joint_vel=np.zeros(12),
            base_height=h,
            feet_contact=4,
        )


def make_supervisor(backend, safety=None, status_cb=None):
    det = FallDetector(DET_CFG)
    fsm = RecoveryFSM(FSM_CFG, det)
    return Supervisor(backend, det, fsm, safety=safety, status_cb=status_cb), fsm


def test_supervisor_recovers_without_mujoco():
    backend = ScriptedBackend()
    sup, fsm = make_supervisor(backend)
    result = sup.run(max_time=25.0)

    assert result["detected"] is True
    assert result["order_safe_before_standup"] is True
    assert result["recovered"] is True
    assert result["final_state"] == S.S0_MONITOR.value
    # 确认时延按「首个判据越限 -> 确认」计，与运行了多久无关（实机必需）
    assert result["detect_delay_s"] == pytest.approx(0.155, abs=0.02)
    # 时间戳调度路径：确实发生过 DAMPING -> (LIEDOWN) -> STANDUP -> HOLD
    modes = [m for _, m in backend.mode_log]
    assert Mode.DAMPING in modes and Mode.STANDUP in modes


def test_supervisor_reports_status():
    seen: list[dict] = []
    backend = ScriptedBackend()
    sup, _ = make_supervisor(backend, status_cb=seen.append)
    sup.run(max_time=12.0)
    assert seen
    assert {"state", "mode", "fault_code", "latency_s"} <= set(seen[0])
    assert any(s["state"] == S.S3_STANDUP.value for s in seen)


def test_stale_sensor_triggers_safety_fault_and_estop():
    safety = SafetyMonitor(SAFETY_CFG)
    backend = ScriptedBackend(obs_lag=2.0)  # 观测时间永远落后 2 s
    sup, fsm = make_supervisor(backend, safety=safety)
    result = sup.run(max_time=10.0, continuous=True)

    assert result["safety_fault"] == "E_SENSOR_STALE"
    assert fsm.state == S.S5_SAFE_FAULT
    assert fsm.fault_code == "E_SENSOR_STALE"
    assert backend.estop is True
    assert fsm.mode == Mode.DAMPING


def test_damping_ack_timeout_only_when_observable():
    # 不可观测：不应误报
    safety = SafetyMonitor(SAFETY_CFG)
    for i in range(100):
        code = safety.check(
            now=0.01 * i, obs_t=0.01 * i, mode=Mode.DAMPING, damping_ack=None
        )
        assert code is None

    # 可观测但一直未确认：超过 0.5 s 报 E_DAMPING_NO_ACK
    safety = SafetyMonitor(SAFETY_CFG)
    code = None
    t = 0.0
    for _ in range(100):
        t += 0.01
        code = safety.check(now=t, obs_t=t, mode=Mode.DAMPING, damping_ack=False)
        if code:
            break
    assert code == "E_DAMPING_NO_ACK"
    assert t == pytest.approx(0.5, abs=0.02)

    # 已确认：不报
    safety = SafetyMonitor(SAFETY_CFG)
    t = 0.0
    for _ in range(100):
        t += 0.01
        assert safety.check(now=t, obs_t=t, mode=Mode.DAMPING, damping_ack=True) is None
