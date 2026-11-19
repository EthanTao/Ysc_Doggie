from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np

from ..core.types import Mode, Observation

ROOT = Path(__file__).resolve().parents[3]
MODELS = ROOT / "sim" / "models" / "lite3"

LEGS = ("FL", "FR", "HL", "HR")
JOINT_NAMES = tuple(f"{leg}_{part}_joint" for leg in LEGS for part in ("HipX", "HipY", "Knee"))

# 官方 lite3_control_parameters.cpp
THIGH = 0.20
SHANK = 0.21
PRE_HEIGHT = 0.12
STAND_HEIGHT = 0.33

# 翻倒姿态注入（预研筛选出的「平台可恢复」构造，见 docs/08-feasibility.md）
Q_PRE = np.array([0.0, -1.35453, 2.54948] * 4)          # 官方预站立（蜷腿）
Q_STRAIGHT = np.array([0.0, -1.35, 0.6] * 4)            # 腿伸展（侧翻时脚有支点）
FALL_INJECTS = {
    "front":      dict(axis=(0.0, 1.0, 0.0), deg=90.0,  joints=Q_PRE),
    "back":       dict(axis=(0.0, 1.0, 0.0), deg=-90.0, joints=Q_PRE),
    # 侧翻 = 侧倾 65° + 前向翻转动量（行走中侧向失稳的典型构造）：
    # 落地后机体翻正成趴地姿态，平台 StandUp 可恢复。
    # 静态侧翻（无动量）会停在约 57° 的侧撑姿态，平台无法起身 —— 见 docs/08。
    "side_left":  dict(axis=(1.0, 0.0, 0.0), deg=65.0,  joints=Q_STRAIGHT, pitch_rate=-1.5),
    "side_right": dict(axis=(1.0, 0.0, 0.0), deg=-65.0, joints=Q_STRAIGHT, pitch_rate=-1.5),
    # 趴地姿态（受阻场景用：跌入矮障碍物下方，靠高度判据检出）
    "prone":      dict(axis=(1.0, 0.0, 0.0), deg=0.0,   joints=Q_PRE),
}


def hipy_by_height(h: float) -> float:
    return -math.acos((THIGH**2 + h**2 - SHANK**2) / (2.0 * h * THIGH))


def knee_by_height(h: float) -> float:
    return math.pi - math.acos((THIGH**2 + SHANK**2 - h**2) / (2.0 * THIGH * SHANK))


def q_stand() -> np.ndarray:
    return np.array([0.0, hipy_by_height(STAND_HEIGHT), knee_by_height(STAND_HEIGHT)] * 4)


class SimBackend:
    """MuJoCo 仿真后端：复现官方状态机的四种模式与站立轨迹。"""

    def __init__(self, sim_cfg: dict, scene: str = "flat") -> None:
        xml = MODELS / ("Lite3.xml" if scene == "flat" else f"Lite3_{scene}.xml")
        if not xml.is_file():
            raise FileNotFoundError(f"scene not found: {xml}")
        self.model = mujoco.MjModel.from_xml_path(str(xml))
        self.model.opt.timestep = float(sim_cfg.get("timestep_s", 0.001))
        self.data = mujoco.MjData(self.model)

        self._qadr = np.array([self.model.joint(n).qposadr[0] for n in JOINT_NAMES])
        self._vadr = np.array([self.model.joint(n).dofadr[0] for n in JOINT_NAMES])
        self._ctrl_lo = self.model.actuator_ctrlrange[:, 0].copy()
        self._ctrl_hi = self.model.actuator_ctrlrange[:, 1].copy()

        self._floor = self.model.geom("floor").id
        self._feet = {self.model.geom(f"{leg}_FOOT_collision").id for leg in LEGS}
        self._torso_body = self.model.body("TORSO").id

        st = sim_cfg.get("stand", {})
        self.kp = float(st.get("kp", 100.0))
        self.kd = float(st.get("kd", 2.5))
        self.damp_kd = float(sim_cfg.get("damping_kd", 2.5))
        self.T = float(st.get("duration_per_phase_s", 2.0))
        self.q_stand = q_stand()
        pre = np.asarray(st.get("pre_pose_rad", [0.0, -1.35453, 2.54948]), dtype=float)
        if pre.size == 3:  # 配置里只写一条腿时自动复制到四条腿
            pre = np.tile(pre, 4)
        self.q_pre = pre

        self.mode = Mode.HOLD
        self._su_t0: float | None = None
        self._su_q0: np.ndarray | None = None

    # ------------------------------------------------------------------ setup
    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.mode = Mode.HOLD
        self._su_t0 = None

    def place_standing(self) -> None:
        self.reset()
        self.data.qpos[2] = STAND_HEIGHT
        self.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qpos[7:] = self.q_stand
        mujoco.mj_forward(self.model, self.data)
        self.mode = Mode.HOLD

    def _robot_geoms(self) -> list[int]:
        """属于机器人本体（TORSO 子树）的 geom，排除地板与场景障碍物。"""
        out = []
        for g in range(self.model.ngeom):
            bid = self.model.geom_bodyid[g]
            while bid != 0:
                if bid == self._torso_body:
                    out.append(g)
                    break
                bid = int(self.model.body_parentid[bid])
        return out

    def place_fall(self, kind: str, clearance: float = 0.005) -> None:
        """按翻倒类型放置：最低接触点上方 clearance，零速度。"""
        if kind not in FALL_INJECTS:
            raise ValueError(f"unknown fall kind: {kind}")
        inj = FALL_INJECTS[kind]
        self.reset()
        a = math.radians(inj["deg"]) / 2.0
        axis = np.asarray(inj["axis"], dtype=float)
        self.data.qpos[2] = 0.5
        self.data.qpos[3:7] = np.concatenate([[math.cos(a)], math.sin(a) * axis])
        self.data.qpos[7:] = inj["joints"]
        mujoco.mj_forward(self.model, self.data)
        # 用精确 geom 距离（包围球对蜷腿姿态太保守，会把机器人放到半空）
        min_dist = min(
            mujoco.mj_geomDistance(self.model, self.data, g, self._floor, 10.0, None)
            for g in self._robot_geoms()
        )
        self.data.qpos[2] += clearance - min_dist
        mujoco.mj_forward(self.model, self.data)
        if inj.get("pitch_rate"):
            # 叠加前向翻转动量（行走中侧向失稳）
            self.data.qvel[3:6] = [0.0, float(inj["pitch_rate"]), 0.0]
        mujoco.mj_forward(self.model, self.data)
        # 静态翻倒注入：本体处于失控/阻尼状态，不能 PD 硬撑站立姿态
        self.mode = Mode.DAMPING

    # ------------------------------------------------------------------ modes
    def set_mode(self, mode: Mode) -> None:
        self.mode = mode

    def start_standup(self, t: float) -> None:
        self._su_t0 = t
        self._su_q0 = self.data.qpos[self._qadr].copy()

    def _spline(self, x0: float, xf: float, t: float, T: float) -> float:
        """官方 GetCubicSplinePos（v0=vf=0）。"""
        if t >= T:
            return xf
        a = (2.0 * x0 - 2.0 * xf) / T**3
        b = (3.0 * xf - 3.0 * x0) / T**2
        return a * t**3 + b * t**2 + x0

    def _standup_target(self, t: float) -> np.ndarray:
        el = t - (self._su_t0 or t)
        if el <= self.T:
            return np.array(
                [self._spline(self._su_q0[i], self.q_pre[i], el, self.T) for i in range(12)]
            )
        hh = self._spline(PRE_HEIGHT, STAND_HEIGHT, el - self.T, self.T)
        return np.array([0.0, hipy_by_height(hh), knee_by_height(hh)] * 4)

    def _pd(self, target: np.ndarray) -> None:
        q = self.data.qpos[self._qadr]
        dq = self.data.qvel[self._vadr]
        tau = self.kp * (target - q) - self.kd * dq
        self.data.ctrl[:] = np.clip(tau, self._ctrl_lo, self._ctrl_hi)

    def _apply_control(self) -> None:
        if self.mode == Mode.DAMPING:
            dq = self.data.qvel[self._vadr]
            self.data.ctrl[:] = np.clip(-self.damp_kd * dq, self._ctrl_lo, self._ctrl_hi)
        elif self.mode == Mode.STANDUP:
            self._pd(self._standup_target(self.data.time))
        elif self.mode == Mode.HOLD:
            self._pd(self.q_stand)
        else:  # RL 等未接入的模式：阻尼兜底
            dq = self.data.qvel[self._vadr]
            self.data.ctrl[:] = np.clip(-self.damp_kd * dq, self._ctrl_lo, self._ctrl_hi)

    # ------------------------------------------------------------------ run
    def step(self) -> None:
        self._apply_control()
        mujoco.mj_step(self.model, self.data)

    # ------------------------------------------------------------------ sense
    def read(self) -> Observation:
        d = self.data
        w, x, y, z = d.qpos[3:7]
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(float(np.clip(2.0 * (w * y - z * x), -1.0, 1.0)))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        feet = 0
        for i in range(d.ncon):
            c = d.contact[i]
            pair = {c.geom1, c.geom2}
            if self._floor in pair and pair & self._feet:
                feet += 1

        return Observation(
            t=d.time,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            omega=d.qvel[3:6].copy(),
            joint_pos=d.qpos[self._qadr].copy(),
            joint_vel=d.qvel[self._vadr].copy(),
            base_height=float(d.qpos[2]),
            feet_contact=feet,
        )
