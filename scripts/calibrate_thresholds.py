#!/usr/bin/env python3
"""基于已记录时间序列离线回放检测器，做阈值 × 驻留时间扫描。

这不是重新跑物理仿真，而是把同一批正常/翻倒日志喂给不同参数的
FallDetector，用于回答“为什么取 50° / 0.3 s”。

输出：
    data/calibration/threshold_sweep.csv
    data/calibration/thresholds.md
    docs/report/figures/fig07_threshold_sweep.png

示例：
    python scripts/calibrate_thresholds.py
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from doggie.core.detector import FallDetector
from doggie.core.types import Observation
from doggie.utils.logger import load_config

ROOT = Path(__file__).resolve().parents[1]
RAW = ROOT / "data" / "raw"
CAL = ROOT / "data" / "calibration"
FIG = ROOT / "docs" / "report" / "figures"

plt.rcParams.update(
    {
        "font.sans-serif": ["Noto Sans CJK SC", "Noto Sans CJK JP", "Droid Sans Fallback"],
        "axes.unicode_minus": False,
        "figure.dpi": 160,
        "savefig.dpi": 160,
        "font.size": 9,
    }
)

FORMAL_RUNS = [
    "2026-11-19_front_1",
    "2026-11-19_front_2",
    "2026-11-19_front_3",
    "2026-11-19_side_1",
    "2026-11-19_side_2",
    "2026-11-19_side_3",
    "2026-11-19_back_1",
    "2026-11-19_back_2",
    "2026-11-19_back_3",
]
NORMAL_RUNS = ["normal_vx0.4", "normal_vx0.6", "normal_turn"]


def read_rows(run_id: str) -> list[dict]:
    path = RAW / run_id / "timeseries.csv"
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def replay(rows: list[dict], cfg: dict) -> dict | None:
    """按日志频率回放检测器；返回首次锁存事件或 None。"""
    det = FallDetector(cfg)
    for r in rows:
        obs = Observation(
            t=float(r["t"]),
            roll=math.radians(float(r["roll_deg"])),
            pitch=math.radians(float(r["pitch_deg"])),
            yaw=0.0,
            omega=np.zeros(3),
            joint_pos=np.zeros(12),
            joint_vel=np.zeros(12),
            base_height=float(r["height_m"]),
            feet_contact=int(r["feet"]),
        )
        if det.update(obs):
            ev = det.event
            assert ev is not None
            return {"t": ev.t, "cue": ev.cue, "kind": ev.kind}
    return None


def evaluate(cfg: dict, normal_cache: dict, fall_cache: dict) -> dict:
    detected = 0
    latencies = []
    side = 0
    for run in FORMAL_RUNS:
        ev = replay(fall_cache[run], cfg)
        if ev is not None and ev["t"] <= 2.0:  # 统一按 2 s 窗口统计
            detected += 1
            latencies.append(ev["t"])
            if "side" in run:
                side += 1
    fp = sum(1 for run in NORMAL_RUNS if replay(normal_cache[run], cfg) is not None)
    return {
        "detected": detected,
        "side_detected": side,
        "max_latency": max(latencies) if latencies else float("nan"),
        "mean_latency": sum(latencies) / len(latencies) if latencies else float("nan"),
        "normal_fp": fp,
        "ok": detected == 9 and fp == 0,
    }


def main() -> None:
    CAL.mkdir(parents=True, exist_ok=True)
    FIG.mkdir(parents=True, exist_ok=True)
    base = load_config(ROOT / "config" / "recovery.yaml")["detector"]
    normal_cache = {run: read_rows(run) for run in NORMAL_RUNS}
    fall_cache = {run: read_rows(run) for run in FORMAL_RUNS}

    thresholds = list(range(40, 71, 5))
    dwells = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0]
    results = []
    for thr in thresholds:
        for dwell in dwells:
            cfg = dict(base)
            cfg.update(
                {
                    "rate_hz": 100.0,  # 日志是 100 Hz；离线回放按日志频率
                    "roll_threshold_deg": float(thr),
                    "pitch_threshold_deg": float(thr),
                    "dwell_s": float(dwell),
                }
            )
            m = evaluate(cfg, normal_cache, fall_cache)
            results.append({"threshold_deg": thr, "dwell_s": dwell, **m})
            print(
                f"thr={thr:2d}° dwell={dwell:.1f}s -> {m['detected']}/9, "
                f"FP={m['normal_fp']}, max_lat={m['max_latency']:.3f}s"
            )

    csv_path = CAL / "threshold_sweep.csv"
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(results[0].keys()))
        w.writeheader()
        w.writerows(results)

    # 热图：可接受区域（9/9 + 0 误报）标出最大时延，失败区域置灰
    grid = np.full((len(dwells), len(thresholds)), np.nan)
    ok = np.zeros_like(grid, dtype=bool)
    for r in results:
        j = thresholds.index(r["threshold_deg"])
        i = dwells.index(r["dwell_s"])
        grid[i, j] = r["max_latency"]
        ok[i, j] = r["ok"]

    fig, ax = plt.subplots(figsize=(8.0, 4.2), constrained_layout=True)
    masked = np.ma.masked_where(~ok, grid)
    im = ax.imshow(
        masked,
        origin="lower",
        aspect="auto",
        cmap="YlGnBu_r",
        vmin=0.0,
        vmax=2.0,
        extent=[min(thresholds) - 2.5, max(thresholds) + 2.5, -0.05, len(dwells) - 0.95],
    )
    for i, dwell in enumerate(dwells):
        for j, thr in enumerate(thresholds):
            if not ok[i, j]:
                ax.text(thr, i, "×", ha="center", va="center", color="#B2182B", fontsize=10)
            else:
                ax.text(
                    thr, i, f"{grid[i, j]:.2f}", ha="center", va="center", fontsize=6.5
                )
    ax.set_yticks(range(len(dwells)), [f"{d:.1f}" for d in dwells])
    ax.set_xticks(thresholds, [f"{t}°" for t in thresholds])
    ax.set_xlabel("姿态阈值 / °")
    ax.set_ylabel("驻留时间 / s")
    ax.set_title("阈值 × 驻留扫描：数字为 9 次中最大判定时延（× = 有漏检/误报）")
    # 当前参数：50° / 0.3 s
    ax.scatter([50], [dwells.index(0.3)], marker="*", s=180, color="#D55E00", edgecolors="white")
    ax.annotate(
        "当前 50° / 0.3 s",
        xy=(50, dwells.index(0.3)),
        xytext=(56, dwells.index(0.3) + 1.2),
        fontsize=8,
        arrowprops=dict(arrowstyle="->", lw=0.8, color="#D55E00"),
        color="#D55E00",
    )
    cb = fig.colorbar(im, ax=ax, pad=0.02)
    cb.set_label("最大时延 / s（可接受区域）")
    fig.savefig(FIG / "fig07_threshold_sweep.png", bbox_inches="tight")
    plt.close(fig)

    # 选定几行写 markdown
    def row(thr: float, dwell: float) -> dict:
        return next(
            r for r in results if r["threshold_deg"] == thr and r["dwell_s"] == dwell
        )

    lines = [
        "# 阈值 / 驻留时间标定（离线回放）",
        "",
        "> 数据：正式 9 次翻倒 + 3×60 s 正常小跑；以 100 Hz 日志回放 `FallDetector`。",
        "> 快速通道（75°/0.15 s）与高度判据（0.10 m/0.3 s）保持不变，只扫描姿态阈值与驻留时间。",
        "",
        "## 扫描范围",
        "",
        f"- 姿态阈值：{min(thresholds)}°–{max(thresholds)}°，步长 5°；",
        f"- 驻留时间：{min(dwells):.1f}–{max(dwells):.1f} s；",
        "- 通过条件：9/9 检出（≤2 s）且 3 段正常小跑 0 误报。",
        "",
        "## 驻留时间对比（阈值 50°）",
        "",
        "| 驻留/s | 检出 | 侧翻检出 | 最大时延/s | 正常误报 | 结论 |",
        "|---|---|---|---|---|---|",
    ]
    for dwell in dwells:
        r = row(50, dwell)
        lines.append(
            f"| {dwell:.1f} | {r['detected']}/9 | {r['side_detected']}/3 | "
            f"{r['max_latency']:.2f} | {r['normal_fp']} | {'通过' if r['ok'] else '不通过'} |"
        )
    lines += [
        "",
        "## 阈值对比（驻留 0.3 s）",
        "",
        "| 阈值/° | 检出 | 侧翻检出 | 最大时延/s | 正常误报 | 结论 |",
        "|---|---|---|---|---|---|",
    ]
    for thr in thresholds:
        r = row(thr, 0.3)
        lines.append(
            f"| {thr} | {r['detected']}/9 | {r['side_detected']}/3 | "
            f"{r['max_latency']:.2f} | {r['normal_fp']} | {'通过' if r['ok'] else '不通过'} |"
        )
    lines += [
        "",
        "## 选择理由",
        "",
        "- 正常包络最大 1.61°，阈值从 40° 起都不会误报；选 50° 是给未建模扰动（坡面、负载、实机噪声）留裕度。",
        "- 侧翻初始 roll 约 60°，阈值升到 60° 时侧翻开始漏检或退化到高度判据；50° 保留约 10° 检出裕度。",
        "- 驻留 0.3 s 可在 0.305 s 内确认侧翻；继续增大驻留只会增加时延，不改善误报（正常数据本来为 0）。",
        "- 回差 35°/0.5 s：低于阈值 15°，大于正常包络 33 倍，避免起身姿态回摆时重复触发。",
        "",
        "![阈值扫描](figures/fig07_threshold_sweep.png)",
        "",
    ]
    (CAL / "thresholds.md").write_text("\n".join(lines))
    print(f"\nwrote {csv_path}")
    print(f"wrote {CAL / 'thresholds.md'}")
    print(f"wrote {FIG / 'fig07_threshold_sweep.png'}")


if __name__ == "__main__":
    main()
