"""FallDetector 的合成数据单元测试：不依赖 MuJoCo。"""

from __future__ import annotations

import math

import numpy as np
import pytest

from doggie.core.detector import FallDetector
from doggie.core.types import Observation

CFG = {
    "rate_hz": 200,
    "lowpass_cutoff_hz": 10.0,
    "roll_threshold_deg": 50.0,
    "pitch_threshold_deg": 50.0,
    "fast_path_deg": 75.0,
    "fast_path_dwell_s": 0.15,
    "rate_fast_path_rad_s": 5.0,
    "rate_fast_dwell_s": 0.05,
    "dwell_s": 0.30,
    "height_min_m": 0.10,
    "height_dwell_s": 0.30,
    "fall_type_pitch_deg": 60.0,
    "fall_type_roll_deg": 60.0,
}

DT = 1.0 / CFG["rate_hz"]


def make_obs(
    t: float, roll_deg=0.0, pitch_deg=0.0, height=0.33, omega=(0.0, 0.0, 0.0)
) -> Observation:
    return Observation(
        t=t,
        roll=math.radians(roll_deg),
        pitch=math.radians(pitch_deg),
        yaw=0.0,
        omega=np.asarray(omega, dtype=float),
        joint_pos=np.zeros(12),
        joint_vel=np.zeros(12),
        base_height=height,
        feet_contact=4,
    )


def run(det: FallDetector, seq) -> bool:
    """seq: [(duration_s, roll, pitch, height)]，返回最后是否触发。"""
    t = 0.0
    fired = False
    for dur, roll, pitch, height in seq:
        for _ in range(int(round(dur / DT))):
            t += DT
            if det.update(make_obs(t, roll, pitch, height)):
                fired = True
    return fired


def test_upright_never_triggers():
    assert not run(FallDetector(CFG), [(5.0, 0, 0, 0.33)])


def test_trot_oscillation_no_false_positive():
    # 正常小跑：roll/pitch 包络 ±25°，快速振荡
    seq = [(0.02, 25 * s, 10 * s, 0.33) for s in (1, -1) for _ in range(125)]
    assert not run(FallDetector(CFG), seq)


def test_transient_spike_ignored():
    # 腾空/颠簸：roll 瞬时 60° 只持续 0.1 s，短于驻留 0.3 s
    assert not run(FallDetector(CFG), [(0.5, 0, 0, 0.33), (0.1, 60, 0, 0.20), (2.0, 0, 0, 0.33)])


def test_sustained_side_triggers():
    det = FallDetector(CFG)
    assert run(det, [(2.0, 65, 0, 0.18)])
    assert det.event is not None
    assert det.event.kind == "side_left"
    assert det.event.cue == "attitude"
    # 0.3 s 驻留 + 低通建立 → 应远小于 2 s
    assert det.event.t <= 0.5


def test_fast_path():
    det = FallDetector(CFG)
    assert run(det, [(1.0, 80, 0, 0.18)])
    assert det.event.cue == "fast_path"


def test_prone_triggers_by_height():
    # 趴地：姿态角≈0，必须靠高度判据
    det = FallDetector(CFG)
    assert run(det, [(2.0, 0, 0, 0.055)])
    assert det.event.cue == "height"
    assert det.event.kind == "flat"


def test_latch_and_clear():
    det = FallDetector(CFG)
    assert run(det, [(2.0, 60, 0, 0.18)])
    assert det.latched
    det.clear()
    assert not det.latched
    assert det.retriggers == 1
    # 清除后重新武装：直立不再触发
    assert not run(det, [(2.0, 0, 0, 0.33)])


def test_back_classification():
    det = FallDetector(CFG)
    run(det, [(2.0, 0, -90, 0.07)])
    assert det.event.kind == "back"


def test_high_rate_full_flip_fast_path():
    """完整 360° 翻转高速段可能只短暂越过 75°；角速度通道应快速确认。"""
    det = FallDetector(CFG)
    fired = False
    for i in range(60):  # 0.3 s
        t = (i + 1) * DT
        # 起始已越过 90°，绕 +y 高速前翻
        if det.update(make_obs(t, roll_deg=180.0, pitch_deg=70.0, height=0.45,
                               omega=(0.0, 16.0, 0.0))):
            fired = True
            break
    assert fired
    assert det.event.cue == "rate_fast"
    assert det.event.kind == "front"
    assert det.event.t <= 0.1


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
