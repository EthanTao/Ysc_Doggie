"""配置单一来源与参数传播测试。"""

from __future__ import annotations

import copy
from pathlib import Path

import numpy as np
import pytest

from doggie.config import ConfigError, apply_profile, load_config, resolve_config
from doggie.core.params import hipy_by_height, q_stand
from doggie.io.sim_backend import SimBackend
from doggie.supervisor import Supervisor

ROOT = Path(__file__).resolve().parents[1]
CFG = load_config(ROOT / "config" / "recovery.yaml")


def test_pre_pose_is_single_source():
    assert CFG["fsm"]["pre_pose_rad"] == CFG["sim"]["stand"]["pre_pose_rad"]
    assert len(CFG["fsm"]["pre_pose_rad"]) == 12


def test_trajectory_durations_match_sim():
    phase = CFG["sim"]["stand"]["duration_per_phase_s"]
    assert CFG["fsm"]["standup_traj_s"] == pytest.approx(2.0 * phase)
    assert CFG["fsm"]["liedown_traj_s"] == pytest.approx(phase)


def test_stand_height_from_config_is_effective():
    cfg = copy.deepcopy(CFG)
    cfg["sim"]["stand"]["stand_height_m"] = 0.40
    cfg = resolve_config(cfg)
    backend = SimBackend(cfg["sim"])
    assert backend.stand_height == pytest.approx(0.40)
    assert backend.q_stand[4] == pytest.approx(hipy_by_height(0.40))
    assert np.allclose(backend.q_stand, q_stand(0.40))


def test_pre_height_from_config_is_effective():
    cfg = copy.deepcopy(CFG)
    cfg["sim"]["stand"]["pre_height_m"] = 0.20
    backend = SimBackend(resolve_config(cfg)["sim"])
    assert backend.pre_height == pytest.approx(0.20)


def test_nominal_is_default_profile():
    """主路径恢复为原来的 65–90° 摔倒；full_flip 只作为附加实验。"""
    assert CFG["active_profile"] == "nominal"
    assert CFG["fsm"]["settle_max_s"] == pytest.approx(3.0)
    inj = CFG["sim"]["fall_injection"]
    assert inj["front"]["deg"] == pytest.approx(90.0)
    assert abs(inj["side_left"]["deg"]) == pytest.approx(65.0)
    assert np.allclose(inj["front"]["rate"], 0.0)
    assert inj["prone"]["clearance_m"] == pytest.approx(0.005)


def test_full_flip_profile_remains_available_as_optional_experiment():
    full = apply_profile(CFG, "full_flip")
    assert full["active_profile"] == "full_flip"
    assert full["fsm"]["settle_max_s"] == pytest.approx(5.0)
    for kind in ("front", "back", "side_left", "side_right"):
        assert full["sim"]["fall_injection"][kind]["clearance_m"] > 0.0
        assert np.linalg.norm(full["sim"]["fall_injection"][kind]["rate"]) > 0.0
    # 不修改原配置
    assert CFG["active_profile"] == "nominal"


def test_unknown_profile_rejected():
    with pytest.raises(ConfigError, match="未知 profile"):
        apply_profile(CFG, "does_not_exist")


def test_missing_injection_rejected():
    cfg = copy.deepcopy(CFG)
    del cfg["profiles"]["full_flip"]["injections"]["back"]
    with pytest.raises(ConfigError, match="缺少"):
        resolve_config(cfg)


def test_mismatched_standup_duration_rejected():
    cfg = copy.deepcopy(CFG)
    cfg["fsm"]["standup_traj_s"] = 4.0
    with pytest.raises(ConfigError, match="standup_traj_s"):
        resolve_config(cfg)


def test_mismatched_liedown_duration_rejected():
    cfg = copy.deepcopy(CFG)
    cfg["fsm"]["liedown_traj_s"] = 1.0
    with pytest.raises(ConfigError, match="liedown_traj_s"):
        resolve_config(cfg)


def test_mismatched_pre_pose_rejected():
    cfg = copy.deepcopy(CFG)
    cfg["fsm"]["pre_pose_rad"] = [0.0, -1.0, 2.0] * 4
    with pytest.raises(ConfigError, match="pre_pose_rad"):
        resolve_config(cfg)


def test_non_integer_rate_rejected():
    cfg = copy.deepcopy(CFG)
    cfg["detector"]["rate_hz"] = 300  # 1 ms 步长下每帧 3.333 步
    with pytest.raises(ConfigError, match="rate_hz"):
        resolve_config(cfg)


def test_invalid_rl_guard_rejected():
    cfg = copy.deepcopy(CFG)
    cfg["sim"]["rl_guard"]["smooth_alpha"] = 0.0
    with pytest.raises(ConfigError, match="smooth_alpha"):
        resolve_config(cfg)


def test_resolve_config_does_not_mutate_input():
    cfg = copy.deepcopy(CFG)
    cfg["sim"]["stand"]["pre_pose_rad"] = [0.0, -1.35453, 2.54948]
    before = copy.deepcopy(cfg)
    resolved = resolve_config(cfg)
    assert cfg == before
    assert resolved is not cfg
    assert len(resolved["sim"]["stand"]["pre_pose_rad"]) == 12


def test_supervisor_period_steps_requires_integer():
    assert Supervisor._period_steps(200.0, 0.001) == 5
    assert Supervisor._period_steps(100.0, 0.001) == 10
    with pytest.raises(ValueError, match="不匹配"):
        Supervisor._period_steps(200.0, 0.003)


def test_stand_ik_matches_official_values():
    q = q_stand(0.33)
    assert q[1] == pytest.approx(-0.6535, abs=1e-4)
    assert q[2] == pytest.approx(1.2711, abs=1e-4)
    for h in (0.12, 0.25, 0.33, 0.40):
        assert np.all(np.isfinite(q_stand(h)))
