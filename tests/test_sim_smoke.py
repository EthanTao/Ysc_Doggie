"""MuJoCo 仿真后端与端到端闭环冒烟测试。"""

from __future__ import annotations

from pathlib import Path

import pytest

from doggie.config import apply_profile
from doggie.core.detector import FallDetector
from doggie.core.recovery_fsm import RecoveryFSM
from doggie.io.sim_backend import SimBackend
from doggie.supervisor import Supervisor
from doggie.utils.logger import load_config

ROOT = Path(__file__).resolve().parents[1]
CFG = load_config(ROOT / "config" / "recovery.yaml")


def test_sim_backend_standing():
    backend = SimBackend(CFG["sim"], scene="flat")
    backend.place_standing()
    for _ in range(500):  # 0.5 s
        backend.step()
    obs = backend.read()
    assert abs(obs.roll_deg) < 2.0
    assert abs(obs.pitch_deg) < 2.0
    assert obs.base_height > 0.30
    assert obs.feet_contact == 4


def test_scene_switch_hides_obstacle():
    """blocked 模型可在平地、受阻与绊脚场景间切换物理几何。"""
    backend = SimBackend(CFG["sim"], scene="blocked")
    assert backend._shelf_body >= 0
    assert backend._hurdle_body >= 0

    backend.set_scene("flat")
    assert backend.scene == "flat"
    assert backend.model.body_pos[backend._shelf_body][2] < -1.0
    assert backend.model.body_pos[backend._hurdle_body][2] < -1.0

    backend.set_scene("blocked")
    assert backend.scene == "blocked"
    assert backend.model.body_pos[backend._shelf_body][2] == backend._shelf_z0
    assert backend.model.body_pos[backend._hurdle_body][2] < -1.0

    backend.set_scene("trip")
    assert backend.scene == "trip"
    assert backend.model.body_pos[backend._shelf_body][2] < -1.0
    assert backend.model.body_pos[backend._hurdle_body][2] == backend._hurdle_pos0[2]


def test_scene_switch_flat_matches_native_flat():
    """切换得到的 flat 与原生 flat 模型物理一致（趴地静置高度相同）。"""
    def settle(backend):
        backend.place_fall("prone")
        for _ in range(2000):  # 2 s
            backend.step()
        return backend.read().base_height

    native = settle(SimBackend(CFG["sim"], scene="flat"))
    switched_backend = SimBackend(CFG["sim"], scene="blocked")
    switched_backend.set_scene("flat")
    switched = settle(switched_backend)
    assert abs(native - switched) < 0.01


def test_sim_end_to_end_front_recovery():
    backend = SimBackend(CFG["sim"], scene="flat")
    detector = FallDetector(CFG["detector"])
    fsm = RecoveryFSM(CFG["fsm"], detector)
    backend.place_fall("front")
    sup = Supervisor(backend, detector, fsm)
    res = sup.run(max_time=20.0)
    assert res["detected"]
    assert res["order_safe_before_standup"]
    assert res["recovered"]
    assert res["final_state"] == "S0_MONITOR"
    # 默认 nominal：旧版小幅前翻，不是 360° 全翻
    assert res["flip_rotation_deg"] < 360.0


@pytest.mark.parametrize("fall,axis", [
    ("front", 1),
    ("back", 1),
    ("side_left", 0),
    ("side_right", 0),
])
def test_full_flip_completes_360_and_lands_prone(fall, axis):
    """四类 full_flip 都必须实际转过 360°，且落回可恢复的趴地姿态。"""
    import math

    full_cfg = apply_profile(CFG, "full_flip")
    backend = SimBackend(full_cfg["sim"], scene="flat")
    backend.place_fall(fall)
    q0 = backend.data.qpos[3:7].copy()
    initial_deg = math.degrees(2.0 * math.acos(min(1.0, abs(float(q0[0])))))
    rotation_deg = 0.0
    dt = float(backend.model.opt.timestep)
    for _ in range(6000):
        rotation_deg += math.degrees(abs(float(backend.data.qvel[3 + axis]))) * dt
        backend.step()
    obs = backend.read()
    assert initial_deg + rotation_deg > 360.0, (fall, initial_deg, rotation_deg)
    assert abs(obs.roll_deg) < 35.0
    assert abs(obs.pitch_deg) < 35.0
    assert obs.base_height < CFG["detector"]["height_min_m"]


def test_trip_scene_falls_and_recovers_from_trot():
    """小跑撞上路沿触发摔倒，并按安全阻尼、整姿、起身流程恢复。"""
    from doggie.trial import run_trial

    result = run_trial(CFG, "trip", max_time=20.0)
    assert result["detected"]
    assert result["order_safe_before_standup"]
    assert result["recovered"]
    assert result["final_state"] == "S0_MONITOR"


def test_trip_contact_guard_suppresses_joint_velocity_spikes():
    """障碍接触瞬间的安全限幅应显著压低关节角速度峰值。"""
    import numpy as np

    backend = SimBackend(CFG["sim"], scene="trip")
    backend.place_trip()
    max_joint_vel = 0.0
    for _ in range(2500):
        backend.step()
        max_joint_vel = max(
            max_joint_vel, float(np.max(np.abs(backend.data.qvel[backend._vadr])))
        )

    assert max_joint_vel < 30.0
