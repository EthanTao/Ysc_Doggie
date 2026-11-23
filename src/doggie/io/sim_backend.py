from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np

from ..core.kinematics import JOINT_NAMES, LEGS
from ..core.params import (  # noqa: F401  (常量在此重新导出，兼容旧脚本)
    PRE_HEIGHT,
    Q_PRE,
    SHANK,
    STAND_HEIGHT,
    THIGH,
    expand_pre_pose,
    hipy_by_height,
    knee_by_height,
    q_stand,
    rpy_from_quat,
)
from ..core.types import Mode, Observation
from .base import RobotBackend
from .jitter import FallJitter, perturb_attitude, perturb_joints, perturb_rates

ROOT = Path(__file__).resolve().parents[3]
MODELS = ROOT / "sim" / "models" / "lite3"

# 翻倒姿态注入参数由 config/recovery.yaml 的 profiles.*.injections 提供：
# - nominal：前/后/侧 65–90° 常规翻倒；
# - full_flip：带角速度与离地高度的完整 360° 翻转。
# 关节角统一使用配置里的 sim.stand.pre_pose_rad（默认 Q_PRE）。
# 见 docs/08-feasibility.md。

# 扩展模型包含障碍架和绊脚路沿；被动窗口只能绑定一个 MjModel，
# 因此运行时通过平移未启用的 body 来切换场景，外观与碰撞会同时隐藏。
SHELF_BODY = "shelf"
SHELF_HIDDEN_DZ = -5.0
TRIP_HURDLE_BODY = "trip_hurdle"
TRIP_HURDLE_HIDDEN_DZ = -5.0


class SimBackend(RobotBackend):
    """MuJoCo 仿真后端：复现官方状态语义与站立轨迹结构。"""

    def __init__(self, sim_cfg: dict, scene: str = "flat") -> None:
        if scene not in ("flat", "blocked", "trip"):
            raise ValueError(f"unknown scene: {scene}")
        # blocked.xml 同时包含可切换的障碍架与绊脚路沿，
        # 避免 playlist 运行时重载模型。
        xml_name = "Lite3.xml" if scene == "flat" else "Lite3_blocked.xml"
        xml = MODELS / xml_name
        if not xml.is_file():
            raise FileNotFoundError(
                f"scene not found: {xml} (run: python scripts/build_scene.py)"
            )
        self.model = mujoco.MjModel.from_xml_path(str(xml))
        self.model.opt.timestep = float(sim_cfg.get("timestep_s", 0.001))
        self.data = mujoco.MjData(self.model)

        self.fall_profile = str(sim_cfg.get("fall_profile", "nominal"))
        injections = sim_cfg.get("fall_injection")
        if not isinstance(injections, dict) or not injections:
            raise ValueError(
                "sim.fall_injection 缺失：请用 doggie.config.load_config()/resolve_config() "
                "解析 profiles，而不是直接传原始 YAML sim 段"
            )
        self.fall_injection = injections

        # 场景切换用的障碍架（flat 模型没有该 body，id 为 -1）
        self.scene = scene
        self._shelf_body = int(
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, SHELF_BODY)
        )
        self._shelf_z0 = (
            float(self.model.body_pos[self._shelf_body][2])
            if self._shelf_body >= 0
            else 0.0
        )
        self._hurdle_body = int(
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, TRIP_HURDLE_BODY)
        )
        self._hurdle_pos0 = (
            self.model.body_pos[self._hurdle_body].copy()
            if self._hurdle_body >= 0
            else np.zeros(3)
        )
        self.set_scene(scene)

        self._qadr = np.array([self.model.joint(n).qposadr[0] for n in JOINT_NAMES])
        self._vadr = np.array([self.model.joint(n).dofadr[0] for n in JOINT_NAMES])
        self._ctrl_lo = self.model.actuator_ctrlrange[:, 0].copy()
        self._ctrl_hi = self.model.actuator_ctrlrange[:, 1].copy()

        self._floor = self.model.geom("floor").id
        self._feet = {
            self.model.geom(f"{leg}_FOOT_collision").id for leg in LEGS
        } | {
            self.model.geom(f"{leg}_SHANK_collision").id for leg in LEGS
        }
        self._torso_body = self.model.body("TORSO").id

        st = sim_cfg.get("stand", {})
        self.kp = float(st.get("kp", 100.0))
        self.kd = float(st.get("kd", 2.5))
        self.damp_kd = float(sim_cfg.get("damping_kd", 2.5))
        self.T = float(st.get("duration_per_phase_s", 1.5))
        self.pre_height = float(st.get("pre_height_m", PRE_HEIGHT))
        self.stand_height = float(st.get("stand_height_m", STAND_HEIGHT))
        self.q_stand = q_stand(self.stand_height)
        self.q_pre = expand_pre_pose(st.get("pre_pose_rad", Q_PRE))

        self.mode = Mode.HOLD
        self._su_t0: float | None = None
        self._su_q0: np.ndarray | None = None
        self._ld_t0: float | None = None
        self._ld_q0: np.ndarray | None = None

        # RL 小跑（官方 ONNX 策略，用于误报/正常数据采集）
        trot = sim_cfg.get("trot", {})
        self.policy_decimation = int(trot.get("policy_decimation", 12))
        self.cmd_max = np.array(
            [
                float(trot.get("vx_max_m_s", 0.8)),
                float(trot.get("vy_max_m_s", 0.8)),
                float(trot.get("wz_max_rad_s", 0.8)),
            ]
        )
        self.trip_vx = float(sim_cfg.get("challenge", {}).get("trip_vx", 0.75))
        guard = sim_cfg.get("rl_guard", {})
        self.rl_guard_enabled = bool(guard.get("enabled", True))
        self.rl_guard_alpha = float(guard.get("smooth_alpha", 0.35))
        self.rl_guard_clip = float(guard.get("vel_clip_rad_s", 10.0))
        self.rl_guard_kd = float(guard.get("kd", 2.0))
        self.rl_guard_hold = float(guard.get("hold_s", 0.40))
        self._impact_until = 0.0
        self.policy = None
        self.rl_target = self.q_stand.copy()
        self.cmd_vel = np.zeros(3)
        self._rl_count = 0

    # ------------------------------------------------------------------ setup
    def reset(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.mode = Mode.HOLD
        self._su_t0 = None
        self._ld_t0 = None
        self._impact_until = 0.0
        # 注入姿态相对直立的初始翻转角；用于报告“是否真正翻过去”。
        self.injected_rotation_deg = 0.0

    def place_standing(self) -> None:
        self.reset()
        self.data.qpos[2] = self.stand_height
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

    def _touching_hurdle(self) -> bool:
        """当前物理步机器人是否与绊脚路沿有接触。"""
        if self._hurdle_body < 0:
            return False
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            if (
                self.model.geom_bodyid[c.geom1] == self._hurdle_body
                or self.model.geom_bodyid[c.geom2] == self._hurdle_body
            ):
                return True
        return False

    def set_scene(self, scene: str) -> None:
        """在当前 model 上切换 flat / blocked / trip。

        blocked 与 trip 场景共享扩展模型：分别启用障碍架或绊脚路沿。
        将未使用的 body 平移到地面以下即可同时关闭外观与碰撞。
        """
        if scene not in ("flat", "blocked", "trip"):
            raise ValueError(f"unknown scene: {scene}")
        if self._shelf_body < 0 and scene == "blocked":
            raise RuntimeError(
                "blocked scene is unavailable in this model: "
                "rebuild with python scripts/build_scene.py"
            )
        if self._hurdle_body < 0 and scene == "trip":
            raise RuntimeError(
                "trip scene is unavailable in this model: "
                "rebuild with python scripts/build_scene.py"
            )
        if self._shelf_body >= 0:
            dz = 0.0 if scene == "blocked" else SHELF_HIDDEN_DZ
            self.model.body_pos[self._shelf_body][2] = self._shelf_z0 + dz
        if self._hurdle_body >= 0:
            self.model.body_pos[self._hurdle_body] = self._hurdle_pos0
            if scene != "trip":
                self.model.body_pos[self._hurdle_body][2] += TRIP_HURDLE_HIDDEN_DZ
        if self._shelf_body >= 0 or self._hurdle_body >= 0:
            mujoco.mj_forward(self.model, self.data)
        self.scene = scene

    def place_trip(self) -> None:
        """从正常站姿开始小跑，迎面撞上路沿并由接触动力学触发摔倒。"""
        if self._hurdle_body < 0:
            raise RuntimeError(
                "trip scene is unavailable in this model: "
                "construct SimBackend(scene='trip') and rebuild the scene if needed"
            )
        self.set_scene("trip")
        self.place_standing()
        self.start_rl()
        self.set_velocity(self.trip_vx, 0.0, 0.0)

    def place_fall(
        self,
        kind: str,
        clearance: float | None = None,
        *,
        jitter: FallJitter | None = None,
        rng: np.random.Generator | None = None,
    ) -> None:
        """按当前 profile 的注入参数放置：最低接触点上方留间隙。

        `clearance` 为空时使用 profile 的 `clearance_m`（全翻需要离地高度，
        常规翻倒通常为 5 mm）。`jitter` 为 None 时保持正式矩阵的确定性；
        传入 `FallJitter` 时只在姿态/关节角/角速度上叠加小扰动，仍按精确
        geom 距离重新留间隙。
        """
        if kind not in self.fall_injection:
            raise ValueError(f"unknown fall kind: {kind} (profile={self.fall_profile})")
        if jitter is not None and not jitter.enabled:
            jitter = None
        if jitter is not None and rng is None:
            raise ValueError("place_fall(jitter=...) requires an explicit numpy Generator")

        inj = self.fall_injection[kind]
        if clearance is None:
            clearance = float(inj.get("clearance_m", 0.005))
        self.reset()
        a = math.radians(float(inj["deg"])) / 2.0
        axis = np.asarray(inj["axis"], dtype=float)
        axis = axis / np.linalg.norm(axis)
        quat = np.concatenate([[math.cos(a)], math.sin(a) * axis])
        joints = self.q_pre.copy()
        if jitter is not None:
            quat = perturb_attitude(rng, quat, jitter.attitude_deg)
            # 关节扰动限制在标称姿态附近，避免越过机械限位。
            joints = np.clip(
                perturb_joints(rng, joints, jitter.joint_rad),
                joints - 0.2,
                joints + 0.2,
            )

        self.data.qpos[2] = 1.0
        self.data.qpos[3:7] = quat
        self.data.qpos[7:] = joints
        mujoco.mj_forward(self.model, self.data)
        # 用精确 geom 距离（包围球对蜷腿姿态太保守，会把机器人放到半空）
        min_dist = min(
            mujoco.mj_geomDistance(self.model, self.data, g, self._floor, 10.0, None)
            for g in self._robot_geoms()
        )
        self.data.qpos[2] += clearance - min_dist
        mujoco.mj_forward(self.model, self.data)
        # 角速度只在注入瞬间叠加，之后完全由重力/接触/阻尼演化。
        self.data.qvel[3:6] = np.asarray(inj.get("rate", [0.0, 0.0, 0.0]), dtype=float)
        if jitter is not None:
            self.data.qvel[3:6] = perturb_rates(rng, self.data.qvel[3:6], jitter.rate_rel)
        self.injected_rotation_deg = abs(float(inj["deg"]))
        # 静态翻倒注入：本体处于失控/阻尼状态，不能 PD 硬撑站立姿态
        self.mode = Mode.DAMPING

    # ------------------------------------------------------------------ modes
    def set_mode(self, mode: Mode) -> None:
        self.mode = mode

    # ------------------------------------------------------------ timing / safety
    def tick(self) -> None:
        self.step()

    def now(self) -> float:
        return float(self.data.time)

    def fixed_timestep(self) -> float | None:
        return float(self.model.opt.timestep)

    def damping_ack(self) -> bool | None:
        return True  # 仿真：模式切换即时生效

    def enable_policy(self, path: Path | None = None) -> None:
        from .policy import PolicyController

        self.policy = PolicyController(path or MODELS / "policy.onnx", max_cmd=self.cmd_max)

    def set_velocity(self, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> None:
        """设置归一化指令 [-1, 1]；实际速度上限见 sim.trot.*。"""
        self.cmd_vel[:] = np.clip([vx, vy, wz], -1.0, 1.0)

    def start_standup(self, t: float) -> None:
        self._su_t0 = t
        self._su_q0 = self.data.qpos[self._qadr].copy()

    def start_liedown(self, t: float) -> None:
        """对齐官方 LieDownState::OnEnter()：记录进入时的关节角与时间戳。"""
        self._ld_t0 = t
        self._ld_q0 = self.data.qpos[self._qadr].copy()

    def start_rl(self) -> None:
        self._rl_count = 0
        self.mode = Mode.RL

    def _spline(self, x0: float, xf: float, t: float, T: float) -> float:
        """按官方三次样条的位置公式生成目标（仿真中令端点速度为零）。"""
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
        hh = self._spline(self.pre_height, self.stand_height, el - self.T, self.T)
        return np.array([0.0, hipy_by_height(hh), knee_by_height(hh)] * 4)

    def _liedown_target(self, t: float) -> np.ndarray:
        """对齐官方 liedown_state.hpp 第一阶段（0 <= el <= T）：三次样条收腿到 Q_PRE。"""
        el = t - (self._ld_t0 or t)
        q0 = self._ld_q0 if self._ld_q0 is not None else self.data.qpos[self._qadr]
        return np.array(
            [self._spline(q0[i], self.q_pre[i], el, self.T) for i in range(12)]
        )

    def _pd(self, target: np.ndarray) -> None:
        q = self.data.qpos[self._qadr]
        dq = self.data.qvel[self._vadr]
        tau = self.kp * (target - q) - self.kd * dq
        self.data.ctrl[:] = np.clip(tau, self._ctrl_lo, self._ctrl_hi)

    def _apply_control(self) -> None:
        if self.mode == Mode.DAMPING:
            dq = self.data.qvel[self._vadr]
            self.data.ctrl[:] = np.clip(-self.damp_kd * dq, self._ctrl_lo, self._ctrl_hi)
        elif self.mode == Mode.LIEDOWN:
            # 官方 liedown_state.hpp：前 T 秒三次样条收腿到 Q_PRE，之后切纯阻尼 kd 贴地
            el = self.data.time - (self._ld_t0 or self.data.time)
            if el <= self.T:
                self._pd(self._liedown_target(self.data.time))
            else:
                dq = self.data.qvel[self._vadr]
                self.data.ctrl[:] = np.clip(-self.kd * dq, self._ctrl_lo, self._ctrl_hi)
        elif self.mode == Mode.STANDUP:
            self._pd(self._standup_target(self.data.time))
        elif self.mode == Mode.HOLD:
            self._pd(self.q_stand)
        elif self.mode == Mode.RL:
            from .policy import KD as RL_KD, KP as RL_KP

            if self.policy is None:
                self.enable_policy()
            if self._touching_hurdle():
                self._impact_until = max(
                    self._impact_until, float(self.data.time) + self.rl_guard_hold
                )
            guard = self.rl_guard_enabled and float(self.data.time) < self._impact_until
            if self._rl_count % self.policy_decimation == 0:
                omega_obs = self.data.qvel[3:6]
                joint_vel_obs = self.data.qvel[self._vadr]
                if guard:
                    clip = self.rl_guard_clip
                    omega_obs = np.clip(omega_obs, -clip, clip)
                    joint_vel_obs = np.clip(joint_vel_obs, -clip, clip)
                raw_target = self.policy.act(
                    self.data.qpos[3:7],
                    omega_obs,
                    self.data.qpos[self._qadr],
                    joint_vel_obs,
                    self.cmd_vel,
                )
                if guard:
                    alpha = self.rl_guard_alpha
                    self.rl_target = (
                        (1.0 - alpha) * self.rl_target + alpha * raw_target
                    )
                else:
                    self.rl_target = raw_target
            self._rl_count += 1
            q = self.data.qpos[self._qadr]
            dq = self.data.qvel[self._vadr]
            kd = self.rl_guard_kd if guard else RL_KD
            tau = RL_KP * (self.rl_target - q) - kd * dq
            self.data.ctrl[:] = np.clip(tau, self._ctrl_lo, self._ctrl_hi)

    # ------------------------------------------------------------------ run
    def step(self) -> None:
        self._apply_control()
        mujoco.mj_step(self.model, self.data)

    # ------------------------------------------------------------------ sense
    def read(self) -> Observation:
        d = self.data
        roll, pitch, yaw = rpy_from_quat(d.qpos[3:7])

        feet_legs = set()
        for i in range(d.ncon):
            c = d.contact[i]
            pair = {c.geom1, c.geom2}
            if self._floor in pair:
                for g in pair & self._feet:
                    gname = self.model.geom(g).name
                    feet_legs.add(gname[:2])
        feet = len(feet_legs)

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
