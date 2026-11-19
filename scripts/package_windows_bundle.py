#!/usr/bin/env python3
"""将报告、PPT提示词、核心统计数据、7张分析图、仿真画面与4段演示视频打包为 Windows 资料包。"""

from __future__ import annotations

import shutil
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PKG_NAME = "Doggie_Windows_Report_Pack"
STAGE_DIR = ROOT / "dist" / PKG_NAME
ZIP_PATH = ROOT / f"{PKG_NAME}.zip"

README_WIN = """# Doggie（Lite3 恢复运控）Windows 报告与 PPT 制作资料包

本资料包已将 Linux 仿真端产出的所有**必要数据、7 张统计标定图、MuJoCo 仿真关键帧长图、4 段未剪辑演示视频、技术报告全文以及 AI 提示词**按目录整理好，方便在 Windows 下直接制作 Word 报告与 PowerPoint 答辩 PPT。

---

## 目录结构说明

### 1. `01_报告与PPT提示词/`
- `06-report.md`：**《四足机器人 Lite3 恢复运控（任务二）技术报告》完整正文**（含摘要、算法公式、状态转移表、实验数据表、参数标定分析与实机迁移方案；图片相对路径已调整为匹配本资料包，可直接用 Typora / VS Code / Pandoc 打开或导出 Word/PDF）。
- `07-slides.md`：**14 页答辩 PPT 逐页稿 + Windows AI 一键生成提示词（Prompt A / Prompt B）+ 图表/视频插入对照表**。
- `data-pack.md`：一页式核心指标与表格速查（做 PPT 贴表格时直接从这里复制最快）。
- `00-requirements-traceability.md`：任务书评分标准（表 7）与交付物逐项映射表。

### 2. `02_核心数据表/`
- `summary.md` / `matrix_2026-11-19.csv`：正式 9 次翻倒 + 3 次起身受阻测试矩阵汇总表（Excel 可直接打开 CSV）。
- `thresholds.md` / `threshold_sweep.csv`：姿态阈值（40°–70°）× 驻留时间（0.1–1.0 s）共 56 组网格扫描标定数据。
- `report_metrics.json`：所有核心指标机器可读汇总（检出率、最小/最大/平均时延、起身耗时、正常小跑包络）。
- `recovery.yaml`：系统唯一参数配置文件（含检测器、状态机、仿真与安全参数）。

### 3. `03_数据图表_7张/`（300 DPI 高清论文级配图）
- `fig01_detection_latency.png`：正式 9 次摔倒判定时延柱状图（0.155–0.305 s vs 优秀档 2.0 s 上限）
- `fig02_attitude_timeline.png`：前翻 / 侧翻 / 后翻 姿态角（roll/pitch）与状态机区间时间线
- `fig03_normal_envelope.png`：官方 ONNX 策略 3×60 s（180 s）正常小跑姿态包络（0 误报，|roll|≤1.06°, |pitch|≤1.71°）
- `fig04_normal_vs_fall.png`：正常小跑包络 vs 翻倒姿态分布散点图（直观展示 50° 阈值裕度）
- `fig05_state_timeline.png`：恢复状态机甘特图（证明 100% 满足 `t_damping < t_standup_cmd` 先安全后起身）
- `fig06_standup.png`：两阶段起身（1.5s + 1.5s）+ 1.0s 站稳验证的躯干高度与俯仰角曲线
- `fig07_threshold_sweep.png`：姿态阈值 × 驻留时间 56 组网格标定热力图（星号标出 50° / 0.3 s 最优工作点）

### 4. `04_仿真关键帧画面/`（MuJoCo 官方 MJCF 渲染，带顶部状态徽章与底部遥测栏）
- **推荐直接插进 Word 报告与 PPT 的四阶段横向长图（`strip_*.png`）**：
  - `strip_front.png`：前翻（+90°）→ 0.155s 快速通道检出 → S1 阻尼落稳 → S3 起身恢复
  - `strip_side_left.png`：左侧翻（+65°）→ 0.305s 检出 → S1 阻尼 → **S2B 调用官方 LieDown 收腿整姿** → S3 起身恢复
  - `strip_back.png`：后翻（-90°+前向回摆）→ 0.305s 检出 → S1 阻尼落稳趴地 → S3 起身恢复
  - `strip_blocked.png`：跌入矮架受阻 → 0.320s 高度判据检出 → 第 1 次受阻 → 退避 2s 第 2 次尝试 → **S5_SAFE_FAULT 安全锁存**
  - `strip_supine_boundary.png`：纯仰翻（四脚朝天 180°）能力边界 → **S2_ASSESS 直接识别不可恢复并锁存 `E_UNRECOVERABLE_POSE`（0 次盲目起身）**
  - `grid_overview.png`：四大场景全流程总览大图（适合放 PPT 架构/实验总览页）
- **`single_frames/` 子目录**：上述每个阶段对应的 20 张 1280×720 独立单帧高清大图（适合在 PPT 里自由排版）。

### 5. `05_演示视频_4段/`（1280×720 未剪辑 MP4，可直接插入 PPT 播放）
- `demo_mjcf_front.mp4`：前翻恢复完整视频
- `demo_mjcf_side.mp4`：侧翻恢复完整视频（含 S2B 收腿整姿 + StandUp 起身）
- `demo_mjcf_back.mp4`：后翻回摆恢复完整视频
- `demo_mjcf_blocked.mp4`：矮架受阻 2 次尝试后进入 `S5_SAFE_FAULT` 安全故障锁存完整视频（满足任务书“包含一次异常处理”要求）

---

## Windows 下快速上手步骤

1. **一键生成 PPT 初稿**：
   打开 `01_报告与PPT提示词/07-slides.md`，滑到**第二部分 Prompt A**，整段复制粘贴到 **Kimi PPT助手 / Gamma.app / WPS AI / Microsoft Copilot**，即可自动生成 14 页结构化 PPT；随后按照文末的《素材插入对照表》把 `03_数据图表_7张/`、`04_仿真关键帧画面/` 和 `05_演示视频_4段/` 拖进对应幻灯片即可。
2. **生成 Word / PDF 技术报告**：
   在 Windows 上使用 **Typora** 或 **VS Code (Markdown PDF 插件)** 打开 `01_报告与PPT提示词/06-report.md`，所有图片均会自动加载显示，点击“导出为 Word (.docx) / PDF”即可得到排版完整的正式报告。
"""


def main() -> None:
    if STAGE_DIR.exists():
        shutil.rmtree(STAGE_DIR)
    STAGE_DIR.mkdir(parents=True, exist_ok=True)

    d1 = STAGE_DIR / "01_报告与PPT提示词"
    d2 = STAGE_DIR / "02_核心数据表"
    d3 = STAGE_DIR / "03_数据图表_7张"
    d4 = STAGE_DIR / "04_仿真关键帧画面"
    d4_single = d4 / "single_frames"
    d5 = STAGE_DIR / "05_演示视频_4段"

    for d in (d1, d2, d3, d4, d4_single, d5):
        d.mkdir(parents=True, exist_ok=True)

    # Top-level README
    (STAGE_DIR / "README_Windows使用说明.md").write_text(README_WIN, encoding="utf-8")

    # 1. Docs & Prompts (rewrite image paths in copies inside 01_报告与PPT提示词 so Typora on Windows renders images directly)
    for doc_name in ("06-report.md", "07-slides.md", "00-requirements-traceability.md"):
        src = ROOT / "docs" / doc_name
        text = src.read_text(encoding="utf-8")
        text_win = (
            text.replace("report/figures/", "../03_数据图表_7张/")
            .replace("report/sim_shots/", "../04_仿真关键帧画面/")
            .replace("figures/", "../03_数据图表_7张/")
            .replace("sim_shots/", "../04_仿真关键帧画面/")
        )
        (d1 / doc_name).write_text(text_win, encoding="utf-8")

    dp_text = (ROOT / "docs" / "report" / "data-pack.md").read_text(encoding="utf-8")
    dp_text = dp_text.replace("figures/", "../03_数据图表_7张/")
    (d1 / "data-pack.md").write_text(dp_text, encoding="utf-8")

    # 2. Data tables & config
    shutil.copy2(ROOT / "data" / "processed" / "summary.md", d2 / "summary.md")
    shutil.copy2(ROOT / "data" / "processed" / "matrix_2026-11-19.csv", d2 / "matrix_2026-11-19.csv")
    shutil.copy2(ROOT / "data" / "calibration" / "threshold_sweep.csv", d2 / "threshold_sweep.csv")
    thr_text = (ROOT / "data" / "calibration" / "thresholds.md").read_text(encoding="utf-8")
    thr_text = thr_text.replace("figures/", "../03_数据图表_7张/")
    (d2 / "thresholds.md").write_text(thr_text, encoding="utf-8")
    shutil.copy2(ROOT / "docs" / "report" / "report_metrics.json", d2 / "report_metrics.json")
    shutil.copy2(ROOT / "config" / "recovery.yaml", d2 / "recovery.yaml")

    # 3. Figures (7 charts)
    for p in sorted((ROOT / "docs" / "report" / "figures").glob("*.png")):
        shutil.copy2(p, d3 / p.name)

    # 4. Simulation keyframes (strips at root of 04, individual frames in single_frames/)
    for p in sorted((ROOT / "docs" / "report" / "sim_shots").glob("*.png")):
        if p.name.startswith(("strip_", "grid_")):
            shutil.copy2(p, d4 / p.name)
        else:
            shutil.copy2(p, d4_single / p.name)
    for p in sorted((ROOT / "docs" / "assets").glob("*.png")):
        shutil.copy2(p, d4_single / p.name)

    # 5. Unedited Demo Videos (4 MP4s)
    for p in sorted((ROOT / "videos").glob("demo_mjcf_*.mp4")):
        shutil.copy2(p, d5 / p.name)

    # Build ZIP archive
    if ZIP_PATH.exists():
        ZIP_PATH.unlink()
    with zipfile.ZipFile(ZIP_PATH, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for file_path in sorted(STAGE_DIR.rglob("*")):
            if file_path.is_file():
                arcname = file_path.relative_to(STAGE_DIR.parent)
                zf.write(file_path, arcname)

    size_mb = ZIP_PATH.stat().st_size / (1024 * 1024)
    print(f"Created package: {ZIP_PATH} ({size_mb:.2f} MB)")


if __name__ == "__main__":
    main()
