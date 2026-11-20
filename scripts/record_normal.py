#!/usr/bin/env python3
"""正常运动数据采集：官方 ONNX 策略小跑/转向，统计检测器误报与姿态包络。

示例：
    python scripts/record_normal.py --vx 0.4 --seconds 60
    python scripts/record_normal.py --vx 0.6 --seconds 60
    python scripts/record_normal.py --wz 0.5 --seconds 60 --run-id normal_turn
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from doggie.core.detector import FallDetector  # noqa: E402
from doggie.core.types import Mode  # noqa: E402
from doggie.io.sim_backend import SimBackend  # noqa: E402
from doggie.utils.logger import load_config  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--vx", type=float, default=0.4, help="归一化指令 [-1,1]（实际速度 = vx × trot.vx_max_m_s）")
    ap.add_argument("--vy", type=float, default=0.0, help="归一化指令 [-1,1]")
    ap.add_argument("--wz", type=float, default=0.0, help="归一化指令 [-1,1]")
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--run-id", default="normal_vx0.4")
    args = ap.parse_args()

    cfg = load_config(ROOT / "config" / "recovery.yaml")
    backend = SimBackend(cfg["sim"])
    # 日志频率同时也是检测器更新频率；低通滤波系数必须按实际更新频率计算。
    log_hz = float(cfg["logging"]["rate_hz"])
    det_cfg = dict(cfg["detector"])
    det_cfg["rate_hz"] = log_hz
    detector = FallDetector(det_cfg)
    every = max(1, int(round(1.0 / (log_hz * backend.model.opt.timestep))))
    backend.place_standing()
    backend.start_rl()
    backend.set_velocity(args.vx, args.vy, args.wz)

    rows = []
    t0 = time.perf_counter()
    trigger_t = None
    max_roll = max_pitch = 0.0
    min_h = 9.9

    print(f"[run] {args.run_id}: vx={args.vx} vy={args.vy} wz={args.wz} for {args.seconds}s")
    step = 0
    while backend.data.time < args.seconds:
        backend.step()
        step += 1
        if step % every == 0:
            obs = backend.read()
            detector.update(obs)
            max_roll = max(max_roll, abs(obs.roll_deg))
            max_pitch = max(max_pitch, abs(obs.pitch_deg))
            min_h = min(min_h, obs.base_height)
            if trigger_t is None and detector.latched:
                trigger_t = obs.t
            rows.append(
                {
                    "t": round(obs.t, 4),
                    "roll_deg": round(obs.roll_deg, 3),
                    "pitch_deg": round(obs.pitch_deg, 3),
                    "height_m": round(obs.base_height, 4),
                    "feet": obs.feet_contact,
                    "detected": int(detector.latched),
                    "lp_roll_deg": round(detector.filtered_roll_deg, 3),
                    "lp_pitch_deg": round(detector.filtered_pitch_deg, 3),
                }
            )

    run_dir = ROOT / "data" / "raw" / args.run_id
    run_dir.mkdir(parents=True, exist_ok=True)
    with open(run_dir / "timeseries.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    duration = float(backend.data.time)
    distance = float(backend.data.qpos[0])
    result = {
        "run_id": args.run_id,
        "cmd_normalized": {"vx": args.vx, "vy": args.vy, "wz": args.wz},
        "cmd_max": cfg["sim"]["trot"],
        "duration_s": round(duration, 2),
        "false_positives": int(detector.latched),
        "first_trigger_s": trigger_t,
        "max_abs_roll_deg": round(max_roll, 2),
        "max_abs_pitch_deg": round(max_pitch, 2),
        "min_height_m": round(min_h, 4),
        "distance_x_m": round(distance, 3),
        "mean_speed_x_m_s": round(distance / duration, 4) if duration > 0 else 0.0,
        "wall_time_s": round(time.perf_counter() - t0, 1),
        "config": {"detector": det_cfg, "trot": cfg["sim"]["trot"]},
    }
    (run_dir / "meta.json").write_text(json.dumps(result, indent=2, ensure_ascii=False))
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
