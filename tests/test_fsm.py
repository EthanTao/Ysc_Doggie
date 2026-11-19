"""RecoveryFSM 的纯逻辑单元测试：用假 backend 驱动完整/失败路径。"""

from __future__ import annotations

import math

import numpy as np
import pytest

from doggie.core.detector import FallDetector
from doggie.core.recovery_fsm import RecoveryFSM
from doggie.core.types import FsmState as S
from doggie.core.types import Observation

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
    "standup_traj_s": 4.0,
    "standup_timeout_s": 20.0,
    "max_attempts": 2,
    "backoff_s": 2.0,
    "verify_s": 1.0,
    "verify_timeout_s": 5.0,
    "upright_roll_deg": 10.0,
    "upright_pitch_deg": 10.0,
    "upright_height_m": 0.25,
    "max_recoveries_per_min": 3,
}
DT = 0.005


class FakeBackend:
    def __init__(self):
        self.standup_calls: list[float] = []

    def start_standup(self, t):
        self.standup_calls.append(t)


def make_obs(t, *, fallen=False, upright=False):
    if upright:
        roll, pitch, h = 0.0, 0.0, 0.33
    elif fallen:
        roll, pitch, h = 60.0, 0.0, 0.18
    else:
        roll, pitch, h = 0.0, 0.0, 0.33
    return Observation(
        t=t,
        roll=math.radians(roll),
        pitch=math.radians(pitch),
        yaw=0.0,
        omega=np.zeros(3),
        joint_pos=np.zeros(12),
        joint_vel=np.zeros(12),
        base_height=h,
        feet_contact=4,
    )


def drive(fsm, det, backend, t, *, fallen=False, upright=False):
    obs = make_obs(t, fallen=fallen, upright=upright)
    det.update(obs)
    fsm.step(obs, backend)


def test_full_recovery_path():
    det = FallDetector(DET_CFG)
    fsm = RecoveryFSM(FSM_CFG, det)
    backend = FakeBackend()

    t = 0.0
    # 1) 正常监测
    for _ in range(20):
        t += DT
        drive(fsm, det, backend, t)
    assert fsm.state == S.S0_MONITOR

    # 2) 摔倒确认 → 阻尼 → 稳定 → 起身（事件驱动，不假设固定时间轴）
    for _ in range(int(5.0 / DT)):
        t += DT
        drive(fsm, det, backend, t, fallen=True)
        if fsm.state == S.S3_STANDUP:
            break
    assert fsm.state == S.S3_STANDUP
    assert det.latched
    assert len(backend.standup_calls) == 1
    assert fsm.attempts == 1
    t_cmd = fsm.m["t_standup_cmd_1"]

    # 3) 起身轨迹大部分时间仍处于翻倒观测（留 0.2 s 余量给直立判定）
    while t - t_cmd < FSM_CFG["standup_traj_s"] - 0.2:
        t += DT
        drive(fsm, det, backend, t, fallen=True)
    assert fsm.state == S.S3_STANDUP

    # 4) 轨迹完成（>=4.2 s）且观测直立 → S4
    while t - t_cmd < FSM_CFG["standup_traj_s"] + 0.25:
        t += DT
        drive(fsm, det, backend, t, upright=True)
    assert fsm.state == S.S4_VERIFY

    # 5) 稳定 1 s → S0 恢复完成
    for _ in range(int(1.2 / DT)):
        t += DT
        drive(fsm, det, backend, t, upright=True)
    assert fsm.state == S.S0_MONITOR
    assert "t_recovered" in fsm.m

    summary = fsm.summary()
    assert summary["detected"]
    assert summary["attempts"] == 1
    assert summary["order_safe_before_standup"]
    assert summary["standup_duration_s"] == pytest.approx(
        round(fsm.m["t_recovered"] - fsm.m["t_standup_cmd_1"], 2)
    )


def test_blocked_path_retries_then_safe_fault():
    det = FallDetector(DET_CFG)
    fsm = RecoveryFSM(FSM_CFG, det)
    backend = FakeBackend()

    t = 0.0
    # 摔倒 → 阻尼 → 起身，但始终无法直立（被障碍挡住）
    for _ in range(int(60.0 / DT)):
        t += DT
        drive(fsm, det, backend, t, fallen=True)

    assert fsm.state == S.S5_SAFE_FAULT
    assert fsm.fault_code is not None
    assert fsm.fault_code.startswith("E_")
    # 不出现无限重试：两次尝试后就锁存
    assert fsm.attempts == FSM_CFG["max_attempts"]
    assert len(backend.standup_calls) == FSM_CFG["max_attempts"]
    # 每次重试之间有退避
    gap = backend.standup_calls[1] - backend.standup_calls[0]
    assert gap >= FSM_CFG["backoff_s"]


def test_order_is_always_safe_before_standup():
    det = FallDetector(DET_CFG)
    fsm = RecoveryFSM(FSM_CFG, det)
    backend = FakeBackend()
    t = 0.0
    for _ in range(int(50.0 / DT)):
        t += DT
        drive(fsm, det, backend, t, fallen=True)
    assert "t_damping" in fsm.m and "t_standup_cmd_1" in fsm.m
    assert fsm.m["t_damping"] < fsm.m["t_standup_cmd_1"]


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
