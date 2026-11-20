"""配置加载与参数一致性解析。

`config/recovery.yaml` 是唯一参数源。本模块在加载时做四件事：
1. 把单腿 3 维 `pre_pose_rad` 展开为 12 维，并同步到 FSM（避免两处定义漂移）；
2. 校验 `standup_traj_s` / `liedown_traj_s` 与仿真每阶段时长一致；
3. 校验采样率与物理步长能整除，避免“配置写 200 Hz、实际按 100 Hz 跑”；
4. 为 trot 段补齐归一化指令上限的默认值。
"""

from __future__ import annotations

import copy
import math
from pathlib import Path

import numpy as np

from .core.params import expand_pre_pose


class ConfigError(ValueError):
    """配置自相矛盾时抛出，避免静默使用错误参数。"""


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


def resolve_config(cfg: dict) -> dict:
    """返回参数一致的深拷贝；不修改调用方传入的 dict。"""
    cfg = copy.deepcopy(cfg)
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

    return cfg


def load_config(path: Path) -> dict:
    import yaml

    with open(path) as f:
        return resolve_config(yaml.safe_load(f))
