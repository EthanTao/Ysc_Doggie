"""Ros2Backend 的离线测试：用假 rclpy / 假消息模块验证话题接线与安全语义。

这台机器没有 ROS2，但后端最容易出错的地方（字段探测、模式去重、阻尼确认、
状态广播、复位服务）都不需要真实 DDS，因此用最小 stub 覆盖。
"""

from __future__ import annotations

import json
import sys
import types
from types import SimpleNamespace

import pytest

from doggie.core.types import Mode


# --------------------------------------------------------------------- stubs
class _Msg:
    def __init__(self):
        self.data = None


class _Trigger:
    def __init__(self):
        self.success = False
        self.message = ""


class FakePublisher:
    def __init__(self, topic):
        self.topic = topic
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class FakeNode:
    def __init__(self, name="fake"):
        self.name = name
        self.subs = []
        self.pubs = {}
        self.services = {}

    def create_subscription(self, typ, topic, cb, qos):
        self.subs.append((topic, cb))
        return object()

    def create_publisher(self, typ, topic, qos):
        pub = FakePublisher(topic)
        self.pubs[topic] = pub
        return pub

    def create_service(self, typ, name, cb):
        self.services[name] = cb
        return object()

    def destroy_node(self):
        pass


@pytest.fixture(scope="module")
def ros_modules():
    rclpy = types.ModuleType("rclpy")
    rclpy.create_node = lambda name: FakeNode(name)
    rclpy.spin_once = lambda node, timeout_sec=0.0: None
    rclpy.try_shutdown = lambda: None

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Bool = _Msg
    std_msgs_msg.String = _Msg
    std_msgs.msg = std_msgs_msg

    std_srvs = types.ModuleType("std_srvs")
    std_srvs_srv = types.ModuleType("std_srvs.srv")
    std_srvs_srv.Trigger = _Trigger
    std_srvs.srv = std_srvs_srv

    drdds = types.ModuleType("drdds")
    drdds_msg = types.ModuleType("drdds.msg")
    drdds_msg.ImuData = type("ImuData", (), {})
    drdds_msg.JointsData = type("JointsData", (), {})
    drdds.msg = drdds_msg

    saved = {
        k: sys.modules.get(k)
        for k in (
            "rclpy",
            "std_msgs",
            "std_msgs.msg",
            "std_srvs",
            "std_srvs.srv",
            "drdds",
            "drdds.msg",
        )
    }
    sys.modules.update(
        {
            "rclpy": rclpy,
            "std_msgs": std_msgs,
            "std_msgs.msg": std_msgs_msg,
            "std_srvs": std_srvs,
            "std_srvs.srv": std_srvs_srv,
            "drdds": drdds,
            "drdds.msg": drdds_msg,
        }
    )
    yield
    for k, v in saved.items():
        if v is None:
            sys.modules.pop(k, None)
        else:
            sys.modules[k] = v


@pytest.fixture
def backend(ros_modules):
    from doggie.io.ros2_backend import Ros2Backend

    cfg = {
        "ros2": {
            "imu_topic": "/IMU_DATA",
            "joints_topic": "/JOINTS_DATA",
            "state_cmd_topic": "/TEST_STATE_CMD",
            "state_feedback_topic": "/TEST_STATE",
            "estop_topic": "/TEST_ESTOP",
            "status_topic": "/doggie/status",
            "reset_service": "/doggie/reset",
            "cmd_resend_s": 10.0,
        }
    }
    node = FakeNode()
    be = Ros2Backend(cfg, node=node, dry_run=False)
    return be


def feed_imu(be, roll_deg=0.0, pitch_deg=0.0):
    be._on_imu(SimpleNamespace(rpy=[roll_deg, pitch_deg, 0.0], omega=[0.0, 0.0, 0.0]))


def feed_joints(be, q):
    be._on_joints(SimpleNamespace(position=list(q), velocity=[0.0] * 12))


def test_topics_wired(backend):
    topics = {t for t, _ in backend.node.subs}
    assert {"/IMU_DATA", "/JOINTS_DATA", "/TEST_STATE"} <= topics
    assert "/TEST_STATE_CMD" in backend.node.pubs
    assert "/TEST_ESTOP" in backend.node.pubs
    assert "/doggie/reset" in backend.node.services


def test_read_builds_observation(backend):
    from doggie.io.sim_backend import q_stand

    feed_imu(backend, roll_deg=10.0, pitch_deg=-5.0)
    feed_joints(backend, q_stand())
    obs = backend.read()
    assert obs.roll == pytest.approx(0.1745, abs=1e-3)
    assert obs.pitch == pytest.approx(-0.0873, abs=1e-3)
    # 倾斜时按重力方向投影，高度会低于直立值，但仍应高于直立判据
    assert 0.25 < obs.base_height < 0.33
    assert 1 <= obs.feet_contact <= 4

    feed_imu(backend, roll_deg=0.0, pitch_deg=0.0)
    level = backend.read()
    assert level.base_height == pytest.approx(0.33, abs=0.01)
    assert level.feet_contact == 4


def test_attitude_bias_is_subtracted(backend):
    feed_imu(backend, roll_deg=2.0, pitch_deg=1.5)
    backend.set_attitude_bias(0.0349066, 0.0261799)  # 2°, 1.5°
    feed_joints(backend, [0.0, -0.65, 1.27] * 4)
    obs = backend.read()
    assert abs(obs.roll) < 1e-6 and abs(obs.pitch) < 1e-6


def test_set_mode_deduplicates_and_publishes(backend):
    backend.set_mode(Mode.DAMPING)
    backend.set_mode(Mode.DAMPING)
    pub = backend.node.pubs["/TEST_STATE_CMD"]
    assert len(pub.sent) == 1
    assert pub.sent[0].data == "kJointDamping"

    backend.set_mode(Mode.STANDUP)
    assert len(pub.sent) == 2
    assert pub.sent[1].data == "kStandUp"


def test_damping_ack_from_feedback(backend):
    assert backend.damping_ack() is True  # 尚未下发指令
    backend.set_mode(Mode.DAMPING)
    backend._on_state(SimpleNamespace(data="kJointDamping"))
    assert backend.damping_ack() is True
    backend._on_state(SimpleNamespace(data="kIdle"))
    assert backend.damping_ack() is False


def test_status_and_reset_and_estop(backend):
    backend.publish_status({"state": "S1_SAFE_DAMPING", "fault_code": ""})
    sent = backend.node.pubs["/doggie/status"].sent
    assert json.loads(sent[-1].data)["state"] == "S1_SAFE_DAMPING"

    called = []
    backend.on_reset = called.append
    resp = _Trigger()
    backend.node.services["/doggie/reset"](None, resp)
    assert resp.success and len(called) == 1

    backend.emergency_stop()
    assert backend.node.pubs["/TEST_ESTOP"].sent[-1].data is True


def test_dry_run_does_not_publish_commands(ros_modules):
    from doggie.io.ros2_backend import Ros2Backend

    cfg = {"ros2": {"state_cmd_topic": "/TEST_STATE_CMD"}}
    node = FakeNode()
    be = Ros2Backend(cfg, node=node, dry_run=True)
    be.set_mode(Mode.DAMPING)
    assert node.pubs["/TEST_STATE_CMD"].sent == []


def test_missing_data_raises(backend):
    with pytest.raises(RuntimeError):
        backend.read()
