#!/usr/bin/env python3
"""MuJoCo 冒烟测试：加载官方 Lite3 模型，PD 站立 5 s，打印姿态与高度。

用法：
    python scripts/sim_smoke_test.py
    MUJOCO_GL=egl python scripts/sim_smoke_test.py --render out.png
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "sim" / "models" / "lite3" / "Lite3.xml"

# 官方 lite3_control_parameters.cpp / standup_state.hpp
PRE_HEIGHT = 0.12
STAND_HEIGHT = 0.33
THIGH = 0.20
SHANK = 0.21
KP = 100.0
KD = 2.5


def hipy_by_height(h: float) -> float:
    return -math.acos((THIGH**2 + h**2 - SHANK**2) / (2.0 * h * THIGH))


def knee_by_height(h: float) -> float:
    return math.pi - math.acos((THIGH**2 + SHANK**2 - h**2) / (2.0 * THIGH * SHANK))


def stand_pose() -> np.ndarray:
    return np.array([0.0, hipy_by_height(STAND_HEIGHT), knee_by_height(STAND_HEIGHT)] * 4)


def rpy_from_quat(q: np.ndarray) -> tuple[float, float]:
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(float(np.clip(2 * (w * y - z * x), -1.0, 1.0)))
    return math.degrees(roll), math.degrees(pitch)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--render", type=Path, default=None, help="保存一帧 PNG")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(MODEL))
    model.opt.timestep = 0.001
    data = mujoco.MjData(model)

    target = stand_pose()
    mujoco.mj_resetData(model, data)
    data.qpos[2] = STAND_HEIGHT
    data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    data.qpos[7:] = target
    mujoco.mj_forward(model, data)

    for _ in range(int(args.seconds / model.opt.timestep)):
        data.ctrl[:] = np.clip(
            KP * (target - data.qpos[7:]) - KD * data.qvel[6:], -40.0, 40.0
        )
        mujoco.mj_step(model, data)

    roll, pitch = rpy_from_quat(data.qpos[3:7])
    print(f"model: {MODEL}")
    print(f"height: {data.qpos[2]:.4f} m (target {STAND_HEIGHT:.2f})")
    print(f"roll/pitch: {roll:.2f} / {pitch:.2f} deg")
    print(f"joint max error: {np.max(np.abs(data.qpos[7:] - target)):.4f} rad")

    if args.render is not None:
        import imageio.v3 as iio

        renderer = mujoco.Renderer(model, 480, 640)
        camera = mujoco.MjvCamera()
        camera.distance = 1.1
        camera.azimuth = 135
        camera.elevation = -15
        camera.lookat[:] = [0.0, 0.0, 0.25]
        renderer.update_scene(data, camera)
        iio.imwrite(args.render, renderer.render())
        print(f"saved: {args.render}")


if __name__ == "__main__":
    main()
