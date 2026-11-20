"""腿部正运动学与实机高度估计的单元测试。

FK 结果直接与官方 MJCF 的 `mj_kinematics` 对拍；高度估计与仿真真值
`data.qpos[2]` 对拍（这是实机没有高度传感器时的替代量，必须先在仿真里校准）。
"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np
import pytest

from doggie.core import kinematics as K
from doggie.io.sim_backend import JOINT_NAMES, Q_PRE, SimBackend, q_stand
from doggie.utils.logger import load_config

ROOT = Path(__file__).resolve().parents[1]
CFG = load_config(ROOT / "config" / "recovery.yaml")
MODEL = ROOT / "sim" / "models" / "lite3" / "Lite3.xml"


@pytest.fixture(scope="module")
def mj():
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    return model, data


def test_fk_matches_official_mjcf(mj):
    model, data = mj
    rng = np.random.default_rng(0)
    qq = rng.uniform([-0.4, -2.4, 0.6], [0.4, 0.2, 2.7], size=(4, 3))
    for i, name in enumerate(JOINT_NAMES):
        data.qpos[model.joint(name).qposadr[0]] = qq[i // 3, i % 3]
    mujoco.mj_kinematics(model, data)

    feet = K.foot_positions(qq.ravel())
    for li, leg in enumerate(K.LEGS):
        truth = data.xpos[model.body(f"{leg}_FOOT").id] - data.qpos[:3]
        # 杆长引用官方控制参数（SHANK=0.21），MJCF 为 0.21012，差异 < 1 mm
        assert np.max(np.abs(feet[li] - truth)) < 1e-3


def test_standing_height_is_about_stand_height():
    q = q_stand()
    assert K.base_height_from_q(q, 0.0, 0.0) == pytest.approx(0.330, abs=0.005)
    assert K.feet_contact_from_q(q, 0.0, 0.0) == 4


def test_height_estimate_tracks_simulation_ground_truth():
    """仿真真值 vs 关节正解高度：误差应 < 3 cm，且趴地时远低于 10 cm 门限。"""
    backend = SimBackend(CFG["sim"], scene="flat")
    for fall in ("none", "prone", "side_left"):
        if fall == "none":
            backend.place_standing()
        else:
            backend.place_fall(fall)
        for _ in range(2500):
            backend.step()
        obs = backend.read()
        est = K.base_height_from_q(obs.joint_pos, obs.roll, obs.pitch)
        assert abs(est - obs.base_height) < 0.03, (fall, est, obs.base_height)
        if fall != "none":
            assert est < CFG["detector"]["height_min_m"]


def test_tucked_pose_is_not_reported_upright():
    """Q_PRE 全蜷腿（官方 LieDown 姿态）高度在 0.10–0.15 m 区间，不应被判直立。"""
    h = K.base_height_from_q(Q_PRE, 0.0, 0.0)
    assert h < CFG["fsm"]["upright_height_m"]
