"""配置加载与参数一致性解析。

`config/recovery.yaml` 是唯一参数源。本模块在加载时做五件事：
1. 校验并展开 `profiles`，把选中 profile 的注入参数与 FSM 覆盖项写入 `sim` / `fsm`；
2. 把单腿 3 维 `pre_pose_rad` 展开为 12 维，并同步到 FSM（避免两处定义漂移）；
3. 校验 `standup_traj_s` / `liedown_traj_s` 与仿真每阶段时长一致；
4. 校验采样率与物理步长能整除，避免“配置写 200 Hz、实际按 100 Hz 跑”；
5. 为 trot 段补齐归一化指令上限的默认值。
"""

from __future__ import annotations

import copy
import math
from pathlib import Path

import numpy as np

from .core.params import expand_pre_pose


class ConfigError(ValueError):
    """配置自相矛盾时抛出，避免静默使用错误参数。"""


FALL_KINDS = ("front", "back", "side_left", "side_right", "prone")
DEFAULT_PROFILE = "nominal"


def _validate_injection(kind: str, data: dict, profile: str) -> dict:
    """校验单个翻倒注入并返回规范化副本（轴向量单位化）。"""
    if not isinstance(data, dict):
        raise ConfigError(f"profiles.{profile}.injections.{kind} 必须是字典")
    axis = data.get("axis", (0.0, 1.0, 0.0))
    try:
        axis = np.asarray(axis, dtype=float).reshape(3)
    except (TypeError, ValueError) as exc:
        raise ConfigError(
            f"profiles.{profile}.injections.{kind}.axis 必须是 3 维向量"
        ) from exc
    norm = float(np.linalg.norm(axis))
    if norm < 1e-9:
        raise ConfigError(f"profiles.{profile}.injections.{kind}.axis 不能为零向量")
    axis = axis / norm

    rate = data.get("rate", (0.0, 0.0, 0.0))
    try:
        rate = np.asarray(rate, dtype=float).reshape(3)
    except (TypeError, ValueError) as exc:
        raise ConfigError(
            f"profiles.{profile}.injections.{kind}.rate 必须是 3 维向量"
        ) from exc

    deg = float(data.get("deg", 0.0))
    clearance = float(data.get("clearance_m", 0.005))
    if clearance < 0.0:
        raise ConfigError(
            f"profiles.{profile}.injections.{kind}.clearance_m 必须非负"
        )
    return {
        "axis": axis.tolist(),
        "deg": deg,
        "rate": rate.tolist(),
        "clearance_m": clearance,
    }


def _resolve_profile(cfg: dict, profile: str | None) -> None:
    """把选中的 profile 注入到 sim/fsm，并校验所有 profile 的结构。"""
    profiles = cfg.get("profiles")
    if not isinstance(profiles, dict) or not profiles:
        raise ConfigError("缺少 profiles：至少需要 nominal / full_flip 之一")

    for name, node in profiles.items():
        if not isinstance(node, dict):
            raise ConfigError(f"profiles.{name} 必须是字典")
        injections = node.get("injections")
        if not isinstance(injections, dict):
            raise ConfigError(f"profiles.{name}.injections 必须是字典")
        missing = [kind for kind in FALL_KINDS if kind not in injections]
        if missing:
            raise ConfigError(
                f"profiles.{name}.injections 缺少：{', '.join(missing)}"
            )
        node["injections"] = {
            kind: _validate_injection(kind, injections[kind], name)
            for kind in FALL_KINDS
        }
        overrides = node.get("fsm") or {}
        if not isinstance(overrides, dict):
            raise ConfigError(f"profiles.{name}.fsm 必须是字典")
        node["fsm"] = overrides

    sim = cfg.setdefault("sim", {})
    selected = profile or sim.get("fall_profile") or DEFAULT_PROFILE
    if selected not in profiles:
        raise ConfigError(
            f"未知 profile: {selected}（可选：{', '.join(sorted(profiles))}）"
        )

    # profile 的 fsm 覆盖项优先于基础 fsm；用深拷贝避免污染 profiles。
    for key, value in profiles[selected]["fsm"].items():
        cfg.setdefault("fsm", {})[key] = copy.deepcopy(value)
    sim["fall_profile"] = selected
    sim["fall_injection"] = copy.deepcopy(profiles[selected]["injections"])
    cfg["active_profile"] = selected


def apply_profile(cfg: dict, profile: str | None = None) -> dict:
    """返回应用指定 profile 的新配置；不修改调用方 dict。"""
    return resolve_config(cfg, profile=profile)


def _check_rate(section: str, key: str, hz: float, dt: float) -> None:
    exact = 1.0 / (float(hz) * float(dt))
    steps = int(round(exact))
    if steps < 1 or abs(exact - steps) > 1e-6:
        raise ConfigError(
            f"{section}.{key}={hz} Hz 与 sim.timestep_s={dt} s 不匹配："
            f"每帧需要 {exact:.6g} 个物理步，必须是正整数"
        )


def _derive_or_check(node: dict, key: str, expected: float, hint: str) -> None:
    if key not in node or node[key] is None:
        node[key] = float(expected)
        return
    if not math.isclose(float(node[key]), float(expected), rel_tol=0.0, abs_tol=1e-6):
        raise ConfigError(
            f"{hint}：{key}={node[key]}，但按 sim.stand.duration_per_phase_s "
            f"应为 {expected}"
        )


def resolve_config(cfg: dict, profile: str | None = None) -> dict:
    """返回参数一致的深拷贝；不修改调用方传入的 dict。

    `profile` 为空时使用 `sim.fall_profile`（默认 `nominal`）。
    profile 同时决定翻倒注入参数（仿真）和 FSM 覆盖项（仿真/实机共用）。
    """
    cfg = copy.deepcopy(cfg)
    _resolve_profile(cfg, profile)
    sim = cfg.setdefault("sim", {})
    stand = sim.setdefault("stand", {})
    dt = sim.get("timestep_s")

    if dt is not None:
        for section, key in (("detector", "rate_hz"), ("logging", "rate_hz")):
            hz = (cfg.get(section) or {}).get(key)
            if hz is not None:
                _check_rate(section, key, float(hz), float(dt))

    pre = stand.get("pre_pose_rad")
    if pre is not None:
        pre12 = expand_pre_pose(pre)
        stand["pre_pose_rad"] = pre12.tolist()
        fsm = cfg.setdefault("fsm", {})
        fsm_pre = fsm.get("pre_pose_rad")
        if fsm_pre is not None and not np.allclose(expand_pre_pose(fsm_pre), pre12):
            raise ConfigError(
                "fsm.pre_pose_rad 与 sim.stand.pre_pose_rad 不一致；"
                "预站立姿态只能在 sim.stand.pre_pose_rad 定义一次"
            )
        fsm["pre_pose_rad"] = pre12.tolist()

    phase = stand.get("duration_per_phase_s")
    if phase is not None:
        fsm = cfg.setdefault("fsm", {})
        _derive_or_check(
            fsm,
            "standup_traj_s",
            2.0 * float(phase),
            "StandUp 总时长必须等于两个阶段",
        )
        _derive_or_check(
            fsm,
            "liedown_traj_s",
            float(phase),
            "LieDown 收腿时长必须与 StandUp 单阶段一致",
        )

    trot = sim.get("trot")
    if trot is not None:
        trot.setdefault("policy_decimation", 12)
        # 官方 ONNX 策略观测里的 cmd_vel 是归一化指令，再乘以下列上限。
        trot.setdefault("vx_max_m_s", 0.8)
        trot.setdefault("vy_max_m_s", 0.8)
        trot.setdefault("wz_max_rad_s", 0.8)

    challenge = sim.get("challenge")
    if challenge is not None:
        challenge.setdefault("trip_vx", 0.75)

    guard = sim.get("rl_guard")
    if guard is not None:
        guard.setdefault("enabled", True)
        guard.setdefault("smooth_alpha", 0.35)
        guard.setdefault("vel_clip_rad_s", 10.0)
        guard.setdefault("kd", 2.0)
        guard.setdefault("hold_s", 0.40)
        alpha = float(guard["smooth_alpha"])
        if not 0.0 < alpha <= 1.0:
            raise ConfigError("sim.rl_guard.smooth_alpha 必须在 (0, 1] 内")
        if float(guard["vel_clip_rad_s"]) <= 0.0:
            raise ConfigError("sim.rl_guard.vel_clip_rad_s 必须大于 0")
        if float(guard["kd"]) < 0.0:
            raise ConfigError("sim.rl_guard.kd 必须非负")
        if float(guard["hold_s"]) < 0.0:
            raise ConfigError("sim.rl_guard.hold_s 必须非负")

    return cfg


def load_config(path: Path) -> dict:
    import yaml

    with open(path) as f:
        return resolve_config(yaml.safe_load(f))
