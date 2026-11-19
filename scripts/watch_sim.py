#!/usr/bin/env python3
"""实时观看仿真：MuJoCo 窗口 + 状态机/检测器实况。

示例：
    python scripts/watch_sim.py --fall side_left --speed 0.5
    python scripts/watch_sim.py --fall back --speed 0.5
    python scripts/watch_sim.py --fall prone --scene blocked
    python scripts/watch_sim.py --fall front --speed 0.25

操作：鼠标左键旋转、右键平移、滚轮缩放；关闭窗口即退出。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from doggie.core.detector import FallDetector  # noqa: E402
from doggie.core.recovery_fsm import RecoveryFSM  # noqa: E402
from doggie.core.types import FsmState as S  # noqa: E402
from doggie.io.sim_backend import SimBackend  # noqa: E402
from doggie.utils.logger import load_config  # noqa: E402

FALLS = ["front", "back", "side_left", "side_right", "prone", "none"]


def set_overlay(viewer, fsm, obs, detector) -> None:
    font = mujoco.mjtFontScale.mjFONTSCALE_150
    grid = mujoco.mjtGridPos
    fault = fsm.fault_code or ""
    texts = [
        (font, grid.mjGRID_TOPLEFT, "Doggie | Lite3 fall recovery", ""),
        (font, grid.mjGRID_TOPRIGHT, fsm.state.value, f"mode: {fsm.mode.value}"),
        (
            font,
            grid.mjGRID_BOTTOMLEFT,
            f"roll {obs.roll_deg:+6.1f}   pitch {obs.pitch_deg:+6.1f}   h {obs.base_height:.3f} m",
            f"feet {obs.feet_contact}   detected: {detector.latched}",
        ),
        (
            font,
            grid.mjGRID_BOTTOMRIGHT,
            f"attempts {fsm.attempts}",
            fault or ("RECOVERED" if "t_recovered" in fsm.m else ""),
        ),
    ]
    viewer.set_texts(texts)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--fall", default="side_left", choices=FALLS)
    ap.add_argument("--scene", default="flat", choices=["flat", "blocked"])
    ap.add_argument("--speed", type=float, default=0.5, help="0.25=慢放，1.0=实时")
    ap.add_argument("--max-time", type=float, default=45.0)
    ap.add_argument("--hold", type=float, default=6.0, help="结束时停留秒数")
    args = ap.parse_args()

    cfg = load_config(ROOT / "config" / "recovery.yaml")
    backend = SimBackend(cfg["sim"], scene=args.scene)
    detector = FallDetector(cfg["detector"])
    fsm = RecoveryFSM(cfg["fsm"], detector)

    if args.fall in ("none", "stand"):
        backend.place_standing()
    else:
        backend.place_fall(args.fall)

    with mujoco.viewer.launch_passive(backend.model, backend.data) as viewer:
        viewer.cam.distance = 1.7
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -18
        viewer.cam.lookat[:] = [0.12, 0.0, 0.22]
        print("窗口已打开：鼠标旋转/滚轮缩放，关闭窗口退出。")

        dt = backend.model.opt.timestep
        det_every = 5          # 200 Hz 判定
        t_start = backend.data.time
        wall_start = time.perf_counter()
        last_sync = 0.0
        i = 0
        obs = backend.read()

        while viewer.is_running():
            backend.step()
            i += 1
            t = backend.data.time

            if i % det_every == 0:
                obs = backend.read()
                detector.update(obs)
                fsm.step(obs, backend)
                backend.set_mode(fsm.mode)

                if (
                    fsm.state == S.S0_MONITOR
                    and fsm.m.get("t_recovered") is not None
                    and t - fsm.m["t_recovered"] > 1.5
                ):
                    break
                if fsm.state == S.S5_SAFE_FAULT and t - fsm.m.get("t_fault", t) > 2.0:
                    break
                if t - t_start > args.max_time:
                    break

            # 按仿真时间实时/慢放
            target_wall = wall_start + (t - t_start) / args.speed
            delay = target_wall - time.perf_counter()
            if delay > 0:
                time.sleep(delay)

            now = time.perf_counter()
            if now - last_sync > 1 / 30:
                if viewer.is_running():
                    with viewer.lock():
                        set_overlay(viewer, fsm, obs, detector)
                    viewer.sync()
                last_sync = now

        # 结果
        summary = fsm.summary()
        print("\n===== 结果 =====")
        print(f"判定: {summary['detected']}  (时延 {summary.get('latency_s', '-')} s)")
        print(f"顺序: {'先安全后起身' if summary['order_safe_before_standup'] else '顺序错误'}")
        print(f"尝试次数: {summary['attempts']}  最终状态: {summary['final_state']}")
        print(f"起身成功: {summary.get('recovered')}  起身耗时: {summary.get('standup_duration_s', '-')} s")
        if summary.get("fault_code"):
            print(f"故障码: {summary['fault_code']}")

        # 停留，方便观察最终姿态
        end = time.perf_counter()
        while viewer.is_running() and time.perf_counter() - end < args.hold:
            time.sleep(1 / 60)
            with viewer.lock():
                set_overlay(viewer, fsm, obs, detector)
            viewer.sync()


if __name__ == "__main__":
    main()
