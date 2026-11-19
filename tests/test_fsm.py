"""RecoveryFSM 的纯逻辑单元测试：用假 backend 驱动完整/失败/四路姿态分流路径。"""

from __future__ import annotations

import math

import numpy as np
import pytest

from doggie.core.detector import FallDetector
from doggie.core.recovery_fsm import DEFAULT_Q_PRE, RecoveryFSM
from doggie.core.types import FsmState as S
from doggie.core.types import Mode, Observation

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
    "assess_prone_deg": 35.0,
    "assess_supine_roll_deg": 120.0,
    "assess_supine_pitch_deg": 80.0,
    "assess_joint_tol_rad": 0.35,
    "liedown_traj_s": 1.5,
    "liedown_settle_s": 0.3,
}
DT = 0.005


class FakeBackend:
    def __init__(self):
        self.standup_calls: list[float] = []
        self.liedown_calls: list[float] = []

    def start_standup(self, t):
        self.standup_calls.append(t)

    def start_liedown(self, t):
        self.liedown_calls.append(t)


def make_obs(
    t,
    *,
    fallen=False,
    upright=False,
    roll_deg: float | None = None,
    pitch_deg: float | None = None,
    height_m: float | None = None,
    joint_pos: np.ndarray | None = None,
):
    if upright:
        roll, pitch, h = 0.0, 0.0, 0.33
    elif fallen:
        roll, pitch, h = 60.0, 0.0, 0.18
    else:
        roll, pitch, h = 0.0, 0.0, 0.33
    if roll_deg is not None:
        roll = roll_deg
    if pitch_deg is not None:
        pitch = pitch_deg
    if height_m is not None:
        h = height_m
    q = DEFAULT_Q_PRE.copy() if joint_pos is None else np.asarray(joint_pos, dtype=float)
    return Observation(
        t=t,
        roll=math.radians(roll),
        pitch=math.radians(pitch),
        yaw=0.0,
        omega=np.zeros(3),
        joint_pos=q,
        joint_vel=np.zeros(3 * 4),
        base_height=h,
        feet_contact=4,
    )


def drive(fsm, det, backend, t, **kwargs):
    obs = make_obs(t, **kwargs)
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

    # 2) 摔倒（60°）→ 确认后机身在阻尼下落回标准趴地（roll=0, h=0.05, q=Q_PRE）→ 直接进 S3_STANDUP
    for _ in range(int(0.4 / DT)):
        t += DT
        drive(fsm, det, backend, t, fallen=True)
    assert det.latched

    for _ in range(int(5.0 / DT)):
        t += DT
        drive(fsm, det, backend, t, roll_deg=0.0, pitch_deg=0.0, height_m=0.05)
        if fsm.state == S.S3_STANDUP:
            break
    assert fsm.state == S.S3_STANDUP
    assert fsm.strategy == "direct_standup"
    assert len(backend.standup_calls) == 1
    assert len(backend.liedown_calls) == 0
    assert fsm.attempts == 1
    t_cmd = fsm.m["t_standup_cmd_1"]

    # 3) 起身轨迹大部分时间仍处于低高度观测（留 0.2 s 余量给直立判定）
    while t - t_cmd < FSM_CFG["standup_traj_s"] - 0.2:
        t += DT
        drive(fsm, det, backend, t, roll_deg=0.0, pitch_deg=0.0, height_m=0.05)
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
    assert summary["recovery_strategy"] == "direct_standup"
    assert summary["order_safe_before_standup"]
    assert summary["standup_duration_s"] == pytest.approx(
        round(fsm.m["t_recovered"] - fsm.m["t_standup_cmd_1"], 2)
    )


def test_assess_routes_sprawled_legs_via_official_liedown():
    """趴地但四肢散开时，S2_ASSESS 应先转 S2B 调用官方 LieDown 对齐四腿，再转 S3_STANDUP。"""
    det = FallDetector(DET_CFG)
    fsm = RecoveryFSM(FSM_CFG, det)
    backend = FakeBackend()

    t = 0.0
    q_sprawled = np.zeros(12)  # 与 Q_PRE 偏差约 2.55 rad >= 0.35 rad
    for _ in range(int(5.0 / DT)):
        t += DT
        drive(fsm, det, backend, t, roll_deg=5.0, pitch_deg=2.0, height_m=0.05, joint_pos=q_sprawled)
        if fsm.state == S.S2B_PREPARE_LIEDOWN:
            break
    assert fsm.state == S.S2B_PREPARE_LIEDOWN
    assert fsm.mode == Mode.LIEDOWN
    assert fsm.strategy == "tuck_then_standup"
    assert len(backend.liedown_calls) == 1

    # 官方 LieDown 完成后无缝进入 S3_STANDUP
    for _ in range(int(2.5 / DT)):
        t += DT
        drive(fsm, det, backend, t, roll_deg=2.0, pitch_deg=1.0, height_m=0.05, joint_pos=DEFAULT_Q_PRE)
        if fsm.state == S.S3_STANDUP:
            break
    assert fsm.state == S.S3_STANDUP
    assert len(backend.standup_calls) == 1


def test_assess_routes_side_prop_via_official_liedown():
    """侧撑卡滞（35°<=|roll|<120°）时，S2_ASSESS 应选 side_prop_liedown 先收腿撤掉侧撑支点。"""
    det = FallDetector(DET_CFG)
    fsm = RecoveryFSM(FSM_CFG, det)
    backend = FakeBackend()

    t = 0.0
    for _ in range(int(5.0 / DT)):
        t += DT
        drive(fsm, det, backend, t, fallen=True)  # roll=60°
        if fsm.state == S.S2B_PREPARE_LIEDOWN:
            break
    assert fsm.state == S.S2B_PREPARE_LIEDOWN
    assert fsm.strategy == "side_prop_liedown"
    assert len(backend.liedown_calls) == 1


def test_assess_rejects_unrecoverable_supine_to_safe_fault():
    """仰翻四脚朝天（|roll|>=120°）时，S2_ASSESS 应直接转 S5_SAFE_FAULT，不盲目触发 StandUp。"""
    det = FallDetector(DET_CFG)
    fsm = RecoveryFSM(FSM_CFG, det)
    backend = FakeBackend()

    t = 0.0
    for _ in range(int(5.0 / DT)):
        t += DT
        drive(fsm, det, backend, t, roll_deg=175.0, pitch_deg=0.0, height_m=0.07)
        if fsm.state == S.S5_SAFE_FAULT:
            break
    assert fsm.state == S.S5_SAFE_FAULT
    assert fsm.fault_code == "E_UNRECOVERABLE_POSE"
    assert fsm.strategy == "unrecoverable_supine"
    assert len(backend.standup_calls) == 0


def test_blocked_path_retries_then_safe_fault():
    det = FallDetector(DET_CFG)
    fsm = RecoveryFSM(FSM_CFG, det)
    backend = FakeBackend()

    t = 0.0
    # 摔倒 → 阻尼 → 起身，但始终无法直立（被障碍挡住）
    for _ in range(int(60.0 / DT)):
        t += DT
        drive(fsm, det, backend, t, roll_deg=0.0, pitch_deg=0.0, height_m=0.05)

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


def test_manual_reset_only_clears_safe_fault():
    det = FallDetector(DET_CFG)
    fsm = RecoveryFSM(FSM_CFG, det)
    backend = FakeBackend()
    t = 0.0
    assert not fsm.manual_reset(t)
    for _ in range(int(5.0 / DT)):
        t += DT
        drive(fsm, det, backend, t, roll_deg=175.0, pitch_deg=0.0, height_m=0.07)
        if fsm.state == S.S5_SAFE_FAULT:
            break
    assert fsm.state == S.S5_SAFE_FAULT
    assert fsm.manual_reset(t)
    assert fsm.state == S.S0_MONITOR
    assert fsm.fault_code is None
    assert not det.latched


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
