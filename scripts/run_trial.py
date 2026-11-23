#!/usr/bin/env python3
"""单次恢复试验：注入翻倒姿态 → 判定 → 阻尼 → 起身 → 验证 / 安全故障。

示例：
    python scripts/run_trial.py --fall side_left --video videos/demo.mp4
    python scripts/run_trial.py --fall front --scene blocked --run-id blocked_demo
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from doggie.trial import run_trial  # noqa: E402
from doggie.utils.logger import load_config  # noqa: E402

FALLS = ["front", "back", "side_left", "side_right", "prone", "trip", "none"]
SCENES = ["flat", "blocked", "trip"]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--fall", default="side_left", choices=FALLS)
    ap.add_argument("--scene", default="flat", choices=SCENES,
                    help="trip 场景会在 --fall trip 时自动选择")
    ap.add_argument("--max-time", type=float, default=60.0)
    ap.add_argument("--run-id", default=None)
    ap.add_argument("--video", default=None, help="输出 mp4 路径")
    ap.add_argument(
        "--profile",
        default=None,
        help="翻倒/恢复 profile（默认用配置的 sim.fall_profile；可选 nominal/full_flip）",
    )
    args = ap.parse_args()

    cfg = load_config(ROOT / "config" / "recovery.yaml")
    if args.profile is not None and args.profile not in cfg.get("profiles", {}):
        ap.error(f"未知 profile: {args.profile}")
    scene = "trip" if args.fall == "trip" and args.scene == "flat" else args.scene
    run_id = args.run_id or f"trial_{args.fall}_{scene}"
    run_dir = ROOT / "data" / "raw" / run_id
    video = Path(args.video) if args.video else None

    result = run_trial(cfg, args.fall, scene, run_dir=run_dir, video=video,
                       max_time=args.max_time, profile=args.profile)

    print(json.dumps(result, indent=2, ensure_ascii=False))
    print(f"\nlogs : {run_dir}")
    if video is not None:
        print(f"video: {video}")


if __name__ == "__main__":
    main()
