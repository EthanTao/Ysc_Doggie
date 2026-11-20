#!/usr/bin/env python3
"""MuJoCo 冒烟测试：加载官方 Lite3 MJCF 生成的场景，PD 站立 5 s，打印姿态与高度。

用法：
    python scripts/sim_smoke_test.py
    MUJOCO_GL=egl python scripts/sim_smoke_test.py --render out.png
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from doggie.core.params import (  # noqa: E402
    STAND_HEIGHT,
    q_stand,
    rpy_from_quat,
)

MODEL = ROOT / "sim" / "models" / "lite3" / "Lite3.xml"

# 官方 lite3_control_parameters.cpp / standup_state.hpp
KP = 100.0
KD = 2.5


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--render", type=Path, default=None, help="保存一帧 PNG")
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(MODEL))
    model.opt.timestep = 0.001
    data = mujoco.MjData(model)

    target = q_stand()
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

    roll, pitch, _ = (math.degrees(v) for v in rpy_from_quat(data.qpos[3:7]))
    print(f"model: {MODEL}")
    print(f"height: {data.qpos[2]:.4f} m (target {STAND_HEIGHT:.2f})")
    print(f"roll/pitch: {roll:.2f} / {pitch:.2f} deg")
    print(f"joint max error: {np.max(np.abs(data.qpos[7:] - target)):.4f} rad")

    if args.render is not None:
        import imageio.v3 as iio

        renderer = mujoco.Renderer(model, 720, 1280)
        option = mujoco.MjvOption()
        option.geomgroup[3] = 0  # 隐藏碰撞体，只留外观
        camera = mujoco.MjvCamera()
        camera.distance = 1.55
        camera.azimuth = 45
        camera.elevation = -18
        camera.lookat[:] = [0.03, 0.0, 0.25]
        renderer.update_scene(data, camera, scene_option=option)
        iio.imwrite(args.render, renderer.render())
        print(f"saved: {args.render}")


if __name__ == "__main__":
    main()
