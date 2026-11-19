#!/usr/bin/env python3
"""批量测试矩阵：正式 9 次（前/后/侧各 3）+ 起身受阻 3 次。

示例：
    python scripts/run_matrix.py                    # 跑 formal + blocked，写 data/processed/
    python scripts/run_matrix.py --suite formal
    python scripts/run_matrix.py --video-key front_1,side_1,back_1,blocked_1
"""

from __future__ import annotations

import argparse
import csv
import sys
from datetime import date
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from doggie.trial import run_trial  # noqa: E402
from doggie.utils.logger import load_config  # noqa: E402

# 正式 9 次：前翻 3、侧翻 3（左右交替）、后翻 3（平台无法自恢复 → 验证失败路径）
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


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--suite", default="all", choices=["formal", "blocked", "all"])
    ap.add_argument("--video-key", default="", help="为指定 run 录视频，逗号分隔 key")
    ap.add_argument("--max-time", type=float, default=60.0)
    args = ap.parse_args()

    cfg = load_config(ROOT / "config" / "recovery.yaml")
    today = date.today().isoformat()

    specs = []
    if args.suite in ("formal", "all"):
        specs += FORMAL
    if args.suite in ("blocked", "all"):
        specs += BLOCKED
    video_keys = {k.strip() for k in args.video_key.split(",") if k.strip()}

    rows = []
    for key, fall, scene in specs:
        run_id = f"{today}_{key}"
        video = ROOT / "videos" / f"{run_id}.mp4" if key in video_keys else None
        print(f"[run] {run_id:24s} fall={fall:10s} scene={scene}")
        res = run_trial(cfg, fall, scene, run_dir=ROOT / "data" / "raw" / run_id,
                        video=video, max_time=args.max_time)
        row = {
            "run_id": run_id,
            "fall": fall,
            "scene": scene,
            "detected": res["detected"],
            "fall_kind": res.get("fall_kind", ""),
            "latency_s": res.get("latency_s", ""),
            "order_ok": res["order_safe_before_standup"],
            "attempts": res["attempts"],
            "recovered": res["recovered"],
            "standup_s": res.get("standup_duration_s", ""),
            "final_state": res["final_state"],
            "fault": res["fault_code"] or "",
        }
        rows.append(row)
        print("      ->", {k: v for k, v in row.items() if k not in ("run_id", "fall", "scene")})

    # 汇总
    out_csv = ROOT / "data" / "processed" / f"matrix_{today}.csv"
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with open(out_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    formal_rows = [r for r in rows if r["scene"] == "flat"]
    blocked_rows = [r for r in rows if r["scene"] == "blocked"]
    n_formal = len(formal_rows)
    det_ok = sum(1 for r in formal_rows if r["detected"])
    order_ok = sum(1 for r in formal_rows if r["order_ok"])
    recovered = sum(1 for r in formal_rows if r["recovered"])
    lat = [float(r["latency_s"]) for r in formal_rows if r["latency_s"] != ""]
    dur = [float(r["standup_s"]) for r in formal_rows if r["standup_s"] != ""]
    fault_ok = sum(1 for r in blocked_rows if r["final_state"] == "S5_SAFE_FAULT")

    lines = [
        f"# 任务二测试矩阵汇总（{today}）",
        "",
        "## 正式 9 次（前/后/侧各 3）",
        f"- 摔倒判定：{det_ok}/{n_formal}",
        f"- 判定时延：min {min(lat):.2f}s / max {max(lat):.2f}s" if lat else "- 判定时延：-",
        f"- 先安全后起身（顺序正确）：{order_ok}/{n_formal}",
        f"- 起身成功：{recovered}/{n_formal}"
        + (f"（平均耗时 {sum(dur)/len(dur):.1f}s）" if dur else ""),
        "",
        "## 起身受阻 3 次",
        f"- 停在安全状态并给出提示：{fault_ok}/{len(blocked_rows)}",
        f"- 重试上限（应全部 ≤2）："
        + ", ".join(str(r['attempts']) for r in blocked_rows),
        "",
        "## 明细",
        "| run | fall | scene | detected | kind | latency_s | order_ok | attempts | recovered | standup_s | final | fault |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for r in rows:
        lines.append(
            f"| {r['run_id']} | {r['fall']} | {r['scene']} | {r['detected']} | "
            f"{r['fall_kind']} | {r['latency_s']} | {r['order_ok']} | {r['attempts']} | "
            f"{r['recovered']} | {r['standup_s']} | {r['final_state']} | {r['fault']} |"
        )
    (ROOT / "data" / "processed" / "summary.md").write_text("\n".join(lines) + "\n")

    print(f"\nsummary: data/processed/summary.md")
    print(f"csv    : {out_csv}")


if __name__ == "__main__":
    main()
