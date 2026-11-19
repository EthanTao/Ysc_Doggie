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
