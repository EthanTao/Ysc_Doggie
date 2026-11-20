"""ROS2 消息解析与 Observation 组装的单元测试（不需要 rclpy）。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np
import pytest

from doggie.core import kinematics as K
from doggie.io import ros2_convert as C


@dataclass
class Vec3:
    x: float
    y: float
    z: float


@dataclass
class Quat:
    x: float
    y: float
    z: float
    w: float


def test_quat_to_euler_known_angles():
    # roll 90°
    r, p, y = C.quat_to_euler(*[math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0, 0.0])
    assert r == pytest.approx(math.pi / 2, abs=1e-9)
    assert p == pytest.approx(0.0, abs=1e-9)
    # 归一化：未归一化四元数也应得到同样结果
    r2, _, _ = C.quat_to_euler(2 * math.cos(math.pi / 4), 2 * math.sin(math.pi / 4), 0.0, 0.0)
    assert r2 == pytest.approx(math.pi / 2, abs=1e-9)


def test_imu_rpy_degrees_and_quaternion():
    msg = SimpleNamespace(rpy=[10.0, -20.0, 30.0])
    r, p, y = C.imu_rpy(msg)
    assert r == pytest.approx(math.radians(10.0))
    assert p == pytest.approx(math.radians(-20.0))
    assert y == pytest.approx(math.radians(30.0))

    # 弧度输入
    r, p, _ = C.imu_rpy(SimpleNamespace(roll=0.5, pitch=-0.25, yaw=0.0), rpy_in_degrees=False)
    assert (r, p) == pytest.approx((0.5, -0.25))

    half = math.pi / 4
    q = Quat(x=math.sin(half), y=0.0, z=0.0, w=math.cos(half))
    r, p, _ = C.imu_rpy(SimpleNamespace(orientation=q))
    assert r == pytest.approx(math.pi / 2, abs=1e-9)
    assert p == pytest.approx(0.0, abs=1e-9)


def test_imu_omega_accepts_nested_and_array():
    assert C.imu_omega(SimpleNamespace(angular_velocity=Vec3(1.0, 2.0, 3.0))) == pytest.approx(
        [1.0, 2.0, 3.0]
    )
    assert C.imu_omega(SimpleNamespace(omega=np.array([4.0, 5.0, 6.0]))) == pytest.approx(
        [4.0, 5.0, 6.0]
    )
    with pytest.raises(ValueError):
        C.imu_omega(SimpleNamespace(foo=1))


def test_joint_reorder():
    canonical = list(K.JOINT_NAMES)
    shuffled = canonical[::-1]
    vec = np.arange(12, dtype=float)
    out = C.reorder_joints(vec, shuffled)
    # out[i] 应等于 canonical[i] 在 shuffled 中的位置上的值
    for i, name in enumerate(canonical):
        assert out[i] == vec[shuffled.index(name)]
    # 已经是 canonical 顺序时原样返回
    assert np.array_equal(C.reorder_joints(vec, canonical), vec)


def test_build_observation_height_and_contact():
    from doggie.io.sim_backend import q_stand

    q = q_stand()
    obs = C.build_observation(
        t=1.0,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
        omega=np.zeros(3),
        joint_pos=q,
        joint_vel=np.zeros(12),
    )
    assert obs.base_height == pytest.approx(0.33, abs=0.01)
    assert obs.feet_contact == 4

    # 足端力优先于几何估计
    obs2 = C.build_observation(
        t=1.0,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
        omega=np.zeros(3),
        joint_pos=q,
        joint_vel=np.zeros(12),
        forces=np.array([20.0, 20.0, 0.0, 0.0]),
        force_threshold_n=15.0,
    )
    assert obs2.feet_contact == 2


def test_foot_forces_shapes():
    assert C.foot_forces(SimpleNamespace(foot_force=[20.0, 0.0, 20.0, 0.0])) == pytest.approx(
        [20.0, 0.0, 20.0, 0.0]
    )
    twelve = np.array([[10, 10, 0]] * 4, dtype=float).ravel()
    assert C.foot_forces(SimpleNamespace(contact_force=twelve)) == pytest.approx([20, 20, 20, 20])
    assert C.foot_forces(SimpleNamespace(nothing=1)) is None
