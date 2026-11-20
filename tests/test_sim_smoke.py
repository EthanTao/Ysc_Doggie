"""MuJoCo 仿真后端与端到端闭环冒烟测试。"""

from __future__ import annotations

from pathlib import Path

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
    """blocked 模型上切到 flat：障碍架应移出场景且可无损切回。"""
    backend = SimBackend(CFG["sim"], scene="blocked")
    assert backend._shelf_body >= 0

    backend.set_scene("flat")
    assert backend.scene == "flat"
    assert backend.model.body_pos[backend._shelf_body][2] < -1.0

    backend.set_scene("blocked")
    assert backend.scene == "blocked"
    assert backend.model.body_pos[backend._shelf_body][2] == backend._shelf_z0


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
    res = sup.run(max_time=12.0)
    assert res["detected"]
    assert res["order_safe_before_standup"]
    assert res["recovered"]
    assert res["final_state"] == "S0_MONITOR"
