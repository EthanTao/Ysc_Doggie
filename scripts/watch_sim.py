#!/usr/bin/env python3
"""实时观看仿真：MuJoCo 窗口 + 状态机/检测器实况。

示例：
    python scripts/watch_sim.py --playlist all --speed 0.5          # 全部 5 类场景，自动切 flat/blocked
    python scripts/watch_sim.py --playlist all --speed 0.5 --loop   # 循环播放，关窗退出
    python scripts/watch_sim.py --fall side_left --speed 0.5 --loop
    python scripts/watch_sim.py --playlist side_left@flat,prone@blocked
    python scripts/watch_sim.py --fall front --speed 0.25

playlist 中每项可写成 `fall@scene` 单独指定场景（省略则用 --scene）。
操作：鼠标左键旋转、右键平移、滚轮缩放；关闭窗口即退出。
"""

from __future__ import annotations

import argparse
import os
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
from doggie.core.types import Mode  # noqa: E402
from doggie.io.sim_backend import SimBackend  # noqa: E402
from doggie.utils.logger import load_config  # noqa: E402

FALLS = ["front", "back", "side_left", "side_right", "prone", "none"]
SCENES = ["flat", "blocked"]

# `--playlist all` 的默认序列：与正式测试矩阵的 5 类场景一一对应，
# 受阻场景自动切到 blocked（障碍架场景）。
DEMO_SEQUENCE = [
    ("side_left", "flat"),
    ("side_right", "flat"),
    ("front", "flat"),
    ("back", "flat"),
    ("prone", "blocked"),
]


def parse_playlist(args) -> list[tuple[str, str]]:
    """把 --playlist 解析成 [(fall, scene), ...]，支持 fall@scene 语法。"""
    raw = [x.strip() for x in args.playlist.split(",") if x.strip()]
    if not raw:
        raw = [args.fall]
    if len(raw) == 1 and raw[0] in ("all", "demo"):
        return list(DEMO_SEQUENCE)

    items: list[tuple[str, str]] = []
    for token in raw:
        fall, _, scene = token.partition("@")
        fall, scene = fall.strip(), scene.strip() or args.scene
        if fall not in FALLS:
            raise SystemExit(f"unknown fall in playlist: {fall} (choices: {', '.join(FALLS)})")
        if scene not in SCENES:
            raise SystemExit(f"unknown scene in playlist: {scene} (choices: {', '.join(SCENES)})")
        items.append((fall, scene))
    return items


def set_overlay(viewer, fsm, obs, detector, scene: str = "") -> None:
    font = mujoco.mjtFontScale.mjFONTSCALE_150
    grid = mujoco.mjtGridPos
    fault = fsm.fault_code or ""
    strat = f" | {fsm.strategy}" if getattr(fsm, "strategy", "") else ""
    texts = [
        (font, grid.mjGRID_TOPLEFT, f"Doggie | Lite3 fall recovery | scene: {scene or '-'}", ""),
        (font, grid.mjGRID_TOPRIGHT, fsm.state.value, f"mode: {fsm.mode.value}{strat}"),
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


def run_once(viewer, backend, detector, fsm, args, fall: str, scene: str, label: str) -> None:
    print(f"\n===== {label}：{fall} @ {scene} =====")
    sys.stdout.flush()
    backend.set_scene(scene)
    if fall in ("none", "stand"):
        backend.place_standing()
    else:
        backend.place_fall(fall)
    detector.reset()
    fsm.reset()
    # 观看正常站立时保持 HOLD；翻倒场景才用阻尼，避免机体自己塌下去。
    fsm.monitor_mode = Mode.HOLD if fall in ("none", "stand") else Mode.DAMPING
    # 注意：不要在这里用 fsm.mode（reset 后是 HOLD）覆盖 backend 的初始模式。
    # 静止翻倒注入应保持 DAMPING，否则 5 ms 的站立 PD 会扰动翻倒过程。
    backend.set_mode(fsm.monitor_mode)

    dt = backend.model.opt.timestep
    det_every = 5  # 200 Hz
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

        target_wall = wall_start + (t - t_start) / args.speed
        delay = target_wall - time.perf_counter()
        if delay > 0:
            time.sleep(delay)

        now = time.perf_counter()
        if now - last_sync > 1 / 30:
            if viewer.is_running():
                with viewer.lock():
                    set_overlay(viewer, fsm, obs, detector, backend.scene)
                viewer.sync()
            last_sync = now

    summary = fsm.summary()
    print("===== 结果 =====")
    print(f"判定: {summary['detected']}  (时延 {summary.get('latency_s', '-')} s)")
    if summary["detected"]:
        order = "先安全后起身" if summary["order_safe_before_standup"] else "顺序错误"
    else:
        order = "—（未检测到摔倒）"
    print(f"顺序: {order}")
    print(f"尝试次数: {summary['attempts']}  最终状态: {summary['final_state']}")
    if summary["detected"]:
        recovered = "t_recovered" in fsm.m
        print(f"起身成功: {recovered}  起身耗时: {summary.get('standup_duration_s', '-')} s")
    if summary.get("fault_code"):
        print(f"故障码: {summary['fault_code']}")
    sys.stdout.flush()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--fall", default="side_left", choices=FALLS)
    ap.add_argument(
        "--playlist",
        default="",
        help="逗号分隔的场景序列，如 side_left,front,back 或 side_left@flat,prone@blocked；"
        "填 all 播放全部 5 类场景（自动切 flat/blocked）",
    )
    ap.add_argument("--scene", default="flat", choices=SCENES, help="播放项未指定场景时的默认场景")
    ap.add_argument("--speed", type=float, default=0.5, help="0.25=慢放，1.0=实时")
    ap.add_argument("--max-time", type=float, default=45.0)
    ap.add_argument("--hold", type=float, default=6.0, help="每次结束后停留秒数")
    ap.add_argument("--loop", action="store_true", help="循环播放，关闭窗口才退出")
    args = ap.parse_args()

    cfg = load_config(ROOT / "config" / "recovery.yaml")
    detector = FallDetector(cfg["detector"])
    fsm = RecoveryFSM(cfg["fsm"], detector)

    playlist = parse_playlist(args)
    # blocked 模型 = flat + 障碍架，因此只要序列里出现 blocked 就加载它，
    # 其余场景运行时把障碍架移到地面以下即可（见 SimBackend.set_scene）。
    initial_scene = "blocked" if any(s == "blocked" for _, s in playlist) else "flat"
    backend = SimBackend(cfg["sim"], scene=initial_scene)

    desc = " -> ".join(f"{f}@{s}" for f, s in playlist)
    with mujoco.viewer.launch_passive(backend.model, backend.data) as viewer:
        viewer.opt.geomgroup[3] = 0  # 隐藏碰撞体，只留外观
        viewer.cam.distance = 1.55
        viewer.cam.azimuth = 45
        viewer.cam.elevation = -18
        viewer.cam.lookat[:] = [0.03, 0.0, 0.25]
        print(f"窗口已打开：{desc}，speed={args.speed}，loop={args.loop}")
        print("鼠标旋转/滚轮缩放；关闭窗口退出。")
        sys.stdout.flush()

        idx = 0
        total = len(playlist)
        while viewer.is_running():
            fall, scene = playlist[idx % total]
            idx += 1
            label = f"[{idx}/{total}]" if not args.loop else f"[{(idx - 1) % total + 1}/{total}]"
            run_once(viewer, backend, detector, fsm, args, fall, scene, label)

            # 停留观察最终姿态
            end = time.perf_counter()
            while viewer.is_running() and time.perf_counter() - end < args.hold:
                time.sleep(1 / 60)
                with viewer.lock():
                    set_overlay(viewer, fsm, backend.read(), detector, backend.scene)
                viewer.sync()

            if not args.loop and idx >= total:
                break

    # 部分 NVIDIA/GLX 环境下被动窗口在析构阶段会崩溃，直接退出规避
    os._exit(0)


if __name__ == "__main__":
    main()
