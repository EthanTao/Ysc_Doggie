#!/usr/bin/env python3
"""随机化压力测试：在正式 9 次矩阵之外，统计带扰动时的成功率与分布。

正式矩阵（scripts/run_matrix.py）保持确定性 9/9；本脚本给初始姿态、关节角与
角速度叠加小扰动，用固定 seed 保证每次试验可复现，输出 CSV + Markdown 汇总。

示例：
    python scripts/run_stress.py --repeats 10 --seed 0
    python scripts/run_stress.py --repeats 20 --falls front,side_left,side_right,back
    python scripts/run_stress.py --repeats 5 --falls prone --scene blocked
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from datetime import date
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from doggie.io.jitter import FallJitter  # noqa: E402
from doggie.trial import run_trial  # noqa: E402
from doggie.utils.logger import load_config  # noqa: E402

DEFAULT_FALLS = "front,side_left,side_right,back"


def _mean_std(values: list[float]) -> tuple[float, float]:
    if not values:
        return float("nan"), float("nan")
    if len(values) == 1:
        return values[0], 0.0
    return statistics.mean(values), statistics.stdev(values)


def _fmt(value: float, digits: int = 3) -> str:
    return "—" if value is None or (isinstance(value, float) and math.isnan(value)) else f"{value:.{digits}f}"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeats", type=int, default=10, help="每类翻倒的随机试验次数")
    ap.add_argument("--seed", type=int, default=0, help="基础种子；同一 (fall, repeat) 可复现")
    ap.add_argument("--falls", default=DEFAULT_FALLS, help="逗号分隔：front,side_left,side_right,back,prone")
    ap.add_argument("--scene", default="flat", choices=["flat", "blocked"])
    ap.add_argument("--attitude-deg", type=float, default=2.5, help="初始姿态扰动标准差（度）")
    ap.add_argument("--joint-rad", type=float, default=0.03, help="关节角扰动标准差（rad）")
    ap.add_argument("--rate-rel", type=float, default=0.10, help="初始角速度相对扰动标准差")
    ap.add_argument("--max-time", type=float, default=40.0)
    ap.add_argument("--date", default=date.today().isoformat())
    ap.add_argument("--out-dir", type=Path, default=ROOT / "data" / "processed")
    args = ap.parse_args()

    falls = [f.strip() for f in args.falls.split(",") if f.strip()]
    if not falls:
        ap.error("--falls 不能为空")
    if args.repeats < 1:
        ap.error("--repeats 必须 >= 1")
    jitter = FallJitter(
        attitude_deg=args.attitude_deg,
        joint_rad=args.joint_rad,
        rate_rel=args.rate_rel,
    )
    cfg = load_config(ROOT / "config" / "recovery.yaml")

    rows: list[dict] = []
    for fall_idx, fall in enumerate(falls):
        for i in range(args.repeats):
            seed = args.seed + fall_idx * 1000 + i
            print(f"[stress] {fall:10s} trial={i + 1:02d}/{args.repeats} seed={seed}")
            try:
                res = run_trial(
                    cfg,
                    fall,
                    args.scene,
                    run_dir=None,
                    video=None,
                    max_time=args.max_time,
                    jitter=jitter,
                    seed=seed,
                )
                row = {
                    "fall": fall,
                    "trial": i + 1,
                    "seed": seed,
                    "detected": res["detected"],
                    "latency_s": res.get("latency_s", ""),
                    "strategy": res.get("recovery_strategy", ""),
                    "attempts": res["attempts"],
                    "recovered": res["recovered"],
                    "standup_s": res.get("standup_duration_s", ""),
                    "final_state": res["final_state"],
                    "fault": res["fault_code"] or "",
                    "error": "",
                }
            except Exception as exc:  # noqa: BLE001
                row = {
                    "fall": fall,
                    "trial": i + 1,
                    "seed": seed,
                    "detected": False,
                    "latency_s": "",
                    "strategy": "",
                    "attempts": 0,
                    "recovered": False,
                    "standup_s": "",
                    "final_state": "ERROR",
                    "fault": "",
                    "error": f"{type(exc).__name__}: {exc}",
                }
            rows.append(row)

    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / f"stress_{args.date}.csv"
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    # ---------------------------------------------------------------- summary
    per_fall = []
    for fall in falls:
        group = [r for r in rows if r["fall"] == fall]
        detected = [r for r in group if r["detected"]]
        recovered = [r for r in group if r["recovered"]]
        lat = [float(r["latency_s"]) for r in detected if r["latency_s"] != ""]
        dur = [float(r["standup_s"]) for r in recovered if r["standup_s"] != ""]
        lat_m, lat_s = _mean_std(lat)
        dur_m, dur_s = _mean_std(dur)
        strategies: dict[str, int] = {}
        for r in group:
            key = r["strategy"] or "—"
            strategies[key] = strategies.get(key, 0) + 1
        failures = [r for r in group if not r["recovered"]]
        per_fall.append(
            {
                "fall": fall,
                "n": len(group),
                "success": len(recovered),
                "rate": len(recovered) / len(group) if group else 0.0,
                "latency_mean": lat_m,
                "latency_std": lat_s,
                "duration_mean": dur_m,
                "duration_std": dur_s,
                "strategies": strategies,
                "failures": failures,
            }
        )

    total = len(rows)
    total_ok = sum(1 for r in rows if r["recovered"])
    lines = [
        f"# 随机化压力测试（{args.date}）",
        "",
        "> 与正式 9 次矩阵分开统计：正式矩阵保持确定性 nominal，本表为带扰动 stress。",
        f"> 扰动：姿态 σ={args.attitude_deg}°，关节 σ={args.joint_rad} rad，"
        f"角速度 σ={args.rate_rel:.0%}；每类 {args.repeats} 次，seed 从 {args.seed} 起。",
        f"> 场景：{args.scene}；结束条件 max_time={args.max_time:.0f} s。",
        "",
        "## 总览",
        "",
        f"- 总成功率：**{total_ok}/{total}（{100.0 * total_ok / total:.1f}%）**",
        "",
        "| 翻倒类型 | 成功 | 成功率 | 时延均值/s | 时延标准差/s | 起身均值/s | 起身标准差/s | 策略分布 |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for item in per_fall:
        strat = "，".join(f"{k}×{v}" for k, v in sorted(item["strategies"].items()))
        lines.append(
            f"| {item['fall']} | {item['success']}/{item['n']} | {100.0 * item['rate']:.1f}% | "
            f"{_fmt(item['latency_mean'])} | {_fmt(item['latency_std'])} | "
            f"{_fmt(item['duration_mean'], 2)} | {_fmt(item['duration_std'], 2)} | {strat} |"
        )

    lines += ["", "## 失败明细", ""]
    any_failure = False
    for item in per_fall:
        for r in item["failures"]:
            any_failure = True
            detail = r["error"] or r["fault"] or r["final_state"]
            lines.append(
                f"- `{r['fall']}` trial={r['trial']} seed={r['seed']}："
                f"strategy={r['strategy'] or '—'}，{detail}"
            )
    if not any_failure:
        lines.append("- 无失败样本。")

    lines += [
        "",
        "## 结果解读",
        "",
        "- 成功率低于 100% 不代表正式矩阵失效：正式 9 次仍是零扰动 nominal 结果；",
        "- 压力测试用于暴露临界工况（例如侧翻落成侧撑后 `LieDown` 无法撤掉支点）；",
        "- 若要提升 stress 成功率，应改恢复策略或注入构造，而不是继续加大 `settle_max_s`。",
        "",
    ]
    md_path = out_dir / f"stress_{args.date}.md"
    md_path.write_text("\n".join(lines))

    print("\n" + "\n".join(lines))
    print(f"\ncsv: {csv_path}")
    print(f"md : {md_path}")


if __name__ == "__main__":
    main()
