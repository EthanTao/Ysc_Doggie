"""翻倒注入扰动：可复现性、幅度与闭环冒烟测试。"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from doggie.config import load_config
from doggie.io.jitter import FallJitter
from doggie.io.sim_backend import SimBackend
from doggie.trial import run_trial

ROOT = Path(__file__).resolve().parents[1]
CFG = load_config(ROOT / "config" / "recovery.yaml")
JITTER = FallJitter(attitude_deg=2.5, joint_rad=0.03, rate_rel=0.10)


def jittered(fall: str, seed: int) -> SimBackend:
    backend = SimBackend(CFG["sim"], scene="flat")
    backend.place_fall(fall, jitter=JITTER, rng=np.random.default_rng(seed))
    return backend


def test_same_seed_is_reproducible():
    a = jittered("side_left", 7)
    b = jittered("side_left", 7)
    assert np.array_equal(a.data.qpos, b.data.qpos)
    assert np.array_equal(a.data.qvel, b.data.qvel)


def test_different_seed_changes_pose():
    a = jittered("side_left", 1)
    b = jittered("side_left", 2)
    assert not np.allclose(a.data.qpos, b.data.qpos)


def test_jitter_stays_bounded():
    nominal = SimBackend(CFG["sim"], scene="flat")
    nominal.place_fall("front")
    pert = jittered("front", 3)

    qn = nominal.data.qpos[3:7]
    qp = pert.data.qpos[3:7]
    angle = 2.0 * math.acos(min(1.0, abs(float(np.dot(qn, qp)))))
    assert angle < math.radians(15.0)

    dq = np.abs(pert.data.qpos[7:] - CFG["sim"]["stand"]["pre_pose_rad"])
    assert dq.max() < 0.2
    assert np.all(np.isfinite(pert.data.qpos))


def test_jitter_requires_rng():
    backend = SimBackend(CFG["sim"], scene="flat")
    with pytest.raises(ValueError, match="Generator"):
        backend.place_fall("front", jitter=JITTER)


def test_zero_jitter_needs_no_rng():
    backend = SimBackend(CFG["sim"], scene="flat")
    backend.place_fall("front", jitter=FallJitter(0.0, 0.0, 0.0))
    assert np.all(np.isfinite(backend.data.qpos))


def test_stress_trial_reproducible_and_recovers():
    a = run_trial(CFG, "front", "flat", max_time=20.0, jitter=JITTER, seed=0)
    b = run_trial(CFG, "front", "flat", max_time=20.0, jitter=JITTER, seed=0)
    assert a["detected"] and a["recovered"]
    assert a["latency_s"] == b["latency_s"]
    assert a["standup_duration_s"] == b["standup_duration_s"]
