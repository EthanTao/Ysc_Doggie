#!/usr/bin/env python3
"""批量测试矩阵：正式 9 次（前/后/侧各 3）+ 起身受阻 + 场景触发挑战。

示例：
    python scripts/run_matrix.py                    # 默认 full_flip：完整 360° 翻转
    python scripts/run_matrix.py --profile nominal  # 旧版 65–90° 小幅注入
    python scripts/run_matrix.py --suite formal
    python scripts/run_matrix.py --video-key front_1,side_1,back_1,blocked_1
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from datetime import date
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from doggie.config import apply_profile  # noqa: E402
from doggie.trial import run_trial  # noqa: E402
from doggie.utils.logger import load_config  # noqa: E402

# 正式 9 次：前翻 3、侧翻 3（左右交替）、后翻 3。
# full_flip profile 下三者都要求完整翻过 360° 后仍落回趴地，再走官方起身。
FORMAL = [
    ("front_1", "front", "flat"),
    ("front_2", "front", "flat"),
    ("front_3", "front", "flat"),
    ("side_1", "side_left", "flat"),
    ("side_2", "side_right", "flat"),
    ("side_3", "side_left", "flat"),
    ("back_1", "back", "flat"),
    ("back_2", "back", "flat"),
    ("back_3", "back", "flat"),
]
# 起身受阻 3 次：趴地 + 上方矮障碍（卡在低矮空间下，验证超时/重试/安全故障）
BLOCKED = [
    ("blocked_1", "prone", "blocked"),
    ("blocked_2", "prone", "blocked"),
    ("blocked_3", "prone", "blocked"),
]
CHALLENGE = [
    ("trip_1", "trip", "trip"),
]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--suite", default="all", choices=["formal", "blocked", "challenge", "all"])
    ap.add_argument("--date", default="2026-11-19", help="测试日期前缀（默认固定 2026-11-19）")
    ap.add_argument("--video-key", default="", help="为指定 run 录视频，逗号分隔 key")
    ap.add_argument("--max-time", type=float, default=60.0)
    ap.add_argument(
        "--profile",
        default=None,
        help="翻倒/恢复 profile（默认用 sim.fall_profile；nominal=旧版小幅，full_flip=完整360°）",
    )
    args = ap.parse_args()

    cfg = load_config(ROOT / "config" / "recovery.yaml")
    if args.profile is not None and args.profile not in cfg.get("profiles", {}):
        ap.error(f"未知 profile: {args.profile}")
    cfg = apply_profile(cfg, args.profile)
    today = args.date

    specs = []
    if args.suite in ("formal", "all"):
        specs += FORMAL
    if args.suite in ("blocked", "all"):
        specs += BLOCKED
    if args.suite in ("challenge", "all"):
        specs += CHALLENGE
    video_keys = {k.strip() for k in args.video_key.split(",") if k.strip()}

    rows = []
    for key, fall, scene in specs:
        run_id = f"{today}_{key}"
        video = ROOT / "videos" / f"{run_id}.mp4" if key in video_keys else None
        print(f"[run] {run_id:24s} fall={fall:10s} scene={scene}")
        res = run_trial(cfg, fall, scene, run_dir=ROOT / "data" / "raw" / run_id,
                        video=video, max_time=args.max_time,
                        profile=cfg.get("active_profile"))
        row = {
            "run_id": run_id,
            "fall": fall,
            "scene": scene,
            "profile": cfg.get("active_profile", ""),
            "detected": res["detected"],
            "fall_kind": res.get("fall_kind", ""),
            "latency_s": res.get("latency_s", ""),
            "detect_delay_s": res.get("detect_delay_s", ""),
            "order_ok": res["order_safe_before_standup"],
            "strategy": res.get("recovery_strategy", ""),
            "attempts": res["attempts"],
            "recovered": res["recovered"],
            "standup_s": res.get("standup_duration_s", ""),
            "flip_rotation_deg": res.get("flip_rotation_deg", ""),
            "final_state": res["final_state"],
            "fault": res["fault_code"] or "",
        }
        rows.append(row)
        print("      ->", {k: v for k, v in row.items() if k not in ("run_id", "fall", "scene")})

    # 汇总
    out_dir = ROOT / "data" / "processed"
    suite_only = args.suite != "all"
    out_csv = out_dir / (
        f"matrix_{args.suite}_{today}.csv" if suite_only else f"matrix_{today}.csv"
    )
    summary_path = out_dir / (
        f"summary_{args.suite}_{today}.md" if suite_only else "summary.md"
    )
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with open(out_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    normal = []
    for mf in sorted((ROOT / "data" / "raw").glob("normal_*/meta.json")):
        normal.append(json.loads(mf.read_text()))

    formal_rows = [r for r in rows if r["scene"] == "flat"]
    blocked_rows = [r for r in rows if r["scene"] == "blocked"]
    challenge_rows = [r for r in rows if r["scene"] == "trip"]
    n_formal = len(formal_rows)
    det_ok = sum(1 for r in formal_rows if r["detected"])
    order_ok = sum(1 for r in formal_rows if r["order_ok"])
    recovered = sum(1 for r in formal_rows if r["recovered"])
    lat = [float(r["latency_s"]) for r in formal_rows if r["latency_s"] != ""]
    dur = [float(r["standup_s"]) for r in formal_rows if r["standup_s"] != ""]
    fault_ok = sum(1 for r in blocked_rows if r["final_state"] == "S5_SAFE_FAULT")

    rotations = [
        float(r["flip_rotation_deg"])
        for r in formal_rows
        if r["flip_rotation_deg"] != ""
    ]
    lines = [
        f"# 任务二测试矩阵汇总（{today}）",
        f"- profile：`{cfg.get('active_profile', '')}`",
        "",
    ]
    if args.suite in ("formal", "all"):
        lines += [
            "## 正式 9 次（前/后/侧各 3）",
            f"- 摔倒判定：{det_ok}/{n_formal}",
            f"- 判定时延：min {min(lat):.3f}s / max {max(lat):.3f}s"
            if lat else "- 判定时延：-",
            f"- 先安全后起身（顺序正确）：{order_ok}/{n_formal}",
            f"- 起身成功：{recovered}/{n_formal}"
            + (f"（平均耗时 {sum(dur)/len(dur):.1f}s）" if dur else ""),
            (
                f"- 累计翻转角度（注入+运行）：{min(rotations):.0f}–{max(rotations):.0f}°"
                if rotations else "- 全翻累计角度：-"
            ),
            "",
        ]
    if args.suite in ("blocked", "all"):
        lines += [
            "## 起身受阻 3 次",
            f"- 停在安全状态并给出提示：{fault_ok}/{len(blocked_rows)}",
            f"- 重试上限（应全部 ≤2）："
            + ", ".join(str(r["attempts"]) for r in blocked_rows),
            "",
        ]
    if challenge_rows:
        challenge_detected = sum(bool(r["detected"]) for r in challenge_rows)
        challenge_recovered = sum(bool(r["recovered"]) for r in challenge_rows)
        challenge_safe = sum(r["final_state"] == "S5_SAFE_FAULT" for r in challenge_rows)
        challenge_delays = [
            float(r["detect_delay_s"])
            for r in challenge_rows
            if r["detect_delay_s"] != ""
        ]
        lines += [
            "## 场景触发挑战",
            f"- 小跑撞上 11 cm 路沿：检测 {challenge_detected}/{len(challenge_rows)}",
            (
                f"- 首个判据越限至确认：{min(challenge_delays):.3f}–"
                f"{max(challenge_delays):.3f} s"
                if challenge_delays else "- 首个判据越限至确认：-"
            ),
            f"- 恢复成功：{challenge_recovered}/{len(challenge_rows)}；"
            f"安全锁存：{challenge_safe}/{len(challenge_rows)}",
            "",
        ]
    if normal:
        lines += [
            "## 正常小跑（误报测试）",
            f"- {len(normal)} 段，合计 {sum(n['duration_s'] for n in normal):.0f} s",
            f"- 误报：{sum(n['false_positives'] for n in normal)} 次",
            f"- 正常姿态包络：|roll| ≤ {max(n['max_abs_roll_deg'] for n in normal):.2f}°"
            f"，|pitch| ≤ {max(n['max_abs_pitch_deg'] for n in normal):.2f}°"
            f"，高度 ≥ {min(n['min_height_m'] for n in normal):.3f} m",
            "",
        ]
    lines += [
        "## 明细",
        "| run | fall | scene | profile | detected | kind | latency_s | detect_delay_s | order_ok | strategy | attempts | recovered | standup_s | flip_deg | final | fault |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for r in rows:
        lines.append(
            f"| {r['run_id']} | {r['fall']} | {r['scene']} | {r['profile']} | "
            f"{r['detected']} | {r['fall_kind']} | {r['latency_s']} | {r['detect_delay_s']} | "
            f"{r['order_ok']} | {r['strategy']} | {r['attempts']} | "
            f"{r['recovered']} | {r['standup_s']} | {r['flip_rotation_deg']} | "
            f"{r['final_state']} | {r['fault']} |"
        )
    summary_path.write_text("\n".join(lines) + "\n")

    print(f"\nsummary: {summary_path.relative_to(ROOT)}")
    print(f"csv    : {out_csv}")


if __name__ == "__main__":
    main()
