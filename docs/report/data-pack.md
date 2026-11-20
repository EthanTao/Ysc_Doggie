# 汇报数据包（仿真）

> 生成时间：2026-11-20T20:28:12  
> 数据源：`data/processed/matrix_2026-11-19.csv`、`data/raw/*`  
> 配置：`config/recovery.yaml`（sha256 前 12 位 `99257fbee6d8`）  
> 代码：`615d7b0`

## 1. 核心指标

| 评分项 | 优秀档要求 | 实测 | 结论 |
|---|---|---|---|
| 摔倒检测 | 9/9，误报 0，时延 ≤2 s | 判定 9/9，时延 0.155–0.305 s（均值 0.255 s）；正常小跑 0/180 s 误报 | 达标 |
| 恢复流程 | 9/9 先安全后起身 | 顺序正确 9/9，每次 `t_damping < t_standup` | 达标 |
| 起身完成 | 9/9，平均 ≤15 s | 9/9；轨迹 3.20 s + 验证 1.00 s = 4.21 s | 达标 |
| 失败与超时 | 3/3 停安全状态，无无限重试 | 3/3 进入 `S5_SAFE_FAULT`；尝试次数 2, 2, 2 | 达标 |
| 参数依据 | 回差与驻留时间有标定数据 | 正常包络 max 1.71°，距 50° 阈值余量 48.3°；翻倒姿态 ≥60° | 见第 4 节 |

## 2. 正式 9 次明细

| # | 场景 | 判定 | 类型 | 时延/s | 先安全后起身 | 恢复策略 | 尝试 | 起身/s | 最终状态 |
|---|---|---|---|---|---|---|---|---|---|
| 1 | 前翻 | ✅ | front | 0.155 | ✅ | `direct_standup` | 1 | 4.21 | `S0_MONITOR` |
| 2 | 前翻 | ✅ | front | 0.155 | ✅ | `direct_standup` | 1 | 4.21 | `S0_MONITOR` |
| 3 | 前翻 | ✅ | front | 0.155 | ✅ | `direct_standup` | 1 | 4.21 | `S0_MONITOR` |
| 4 | 左侧翻 | ✅ | side_left | 0.305 | ✅ | `tuck_then_standup` | 1 | 4.21 | `S0_MONITOR` |
| 5 | 右侧翻 | ✅ | side_right | 0.305 | ✅ | `tuck_then_standup` | 1 | 4.21 | `S0_MONITOR` |
| 6 | 左侧翻 | ✅ | side_left | 0.305 | ✅ | `tuck_then_standup` | 1 | 4.21 | `S0_MONITOR` |
| 7 | 后翻 | ✅ | back | 0.305 | ✅ | `direct_standup` | 1 | 4.21 | `S0_MONITOR` |
| 8 | 后翻 | ✅ | back | 0.305 | ✅ | `direct_standup` | 1 | 4.21 | `S0_MONITOR` |
| 9 | 后翻 | ✅ | back | 0.305 | ✅ | `direct_standup` | 1 | 4.21 | `S0_MONITOR` |

## 3. 起身受阻 3 次

| run | 判定 cue | 时延/s | 尝试 | 最终状态 | 故障码 |
|---|---|---|---|---|---|
| 2026-11-19_blocked_1 | height | 0.320 | 2 | `S5_SAFE_FAULT` | `E_STANDUP_NOT_UPRIGHT` |
| 2026-11-19_blocked_2 | height | 0.320 | 2 | `S5_SAFE_FAULT` | `E_STANDUP_NOT_UPRIGHT` |
| 2026-11-19_blocked_3 | height | 0.320 | 2 | `S5_SAFE_FAULT` | `E_STANDUP_NOT_UPRIGHT` |

## 4. 正常小跑与阈值依据

| run | 指令 | 时长/s | 误报 | max abs roll/° | max abs pitch/° | min height/m |
|---|---|---|---|---|---|---|
| normal_turn | vx=0.3, wz=0.5 | 60 | 0 | 0.62 | 1.56 | 0.319 |
| normal_vx0.4 | vx=0.4, wz=0.0 | 60 | 0 | 1.06 | 1.41 | 0.320 |
| normal_vx0.6 | vx=0.6, wz=0.0 | 60 | 0 | 0.93 | 1.71 | 0.321 |

- 正常姿态包络：**max roll 1.06° / max pitch 1.71°**，相对 50° 阈值余量约 **48.3°**。
- 翻倒姿态：前/后翻 pitch≈90°，侧翻 roll≈60°；阈值 50° 正好位于 1.6° 正常包络与 60° 翻倒姿态之间。
- 回差 35°、驻留 0.3 s、快速通道 75°/0.15 s；时间线见 `fig02`，状态机顺序见 `fig05`。

## 5. 阈值 / 驻留标定（离线回放）

用正式 9 次 + 3×60 s 正常日志回放检测器，扫描姿态阈值 40°–70°、驻留 0.1–1.0 s：

| 配置 | 检出 | 最大时延/s | 正常误报 | 说明 |
|---|---|---|---|---|
| 50° / 0.3 s（当前） | 9/9 | 0.31 | 0 | 侧翻由姿态判据在 0.305 s 确认 |
| 50° / 1.0 s | 9/9 | 0.98 | 0 | 驻留过长，退化到高度判据 |
| 60° / 0.3 s | 9/9 | 0.31 | 0 | 侧翻 roll≈65° 临界检出，裕度仅 5° |
| 70° / 0.3 s | 9/9 | 0.98 | 0 | 侧翻姿态不再超限，靠高度兜底 |

完整扫描见 `data/calibration/threshold_sweep.csv` 与 `data/calibration/thresholds.md`；热图见 `figures/fig07_threshold_sweep.png`。

## 6. 图表索引

| 文件 | 用途 |
|---|---|
| `figures/fig01_detection_latency.png` | 9 次判定时延与 2 s 优秀档上限 |
| `figures/fig02_attitude_timeline.png` | 前/侧/后翻姿态、阈值与状态区间 |
| `figures/fig03_normal_envelope.png` | 3×60 s 官方 ONNX 小跑姿态包络 |
| `figures/fig04_normal_vs_fall.png` | 正常/翻倒姿态分离与阈值裕度 |
| `figures/fig05_state_timeline.png` | 典型试验的状态机时间线 |
| `figures/fig06_standup.png` | 起身高度/俯仰轨迹与 3 s + 1 s 时序 |
| `figures/fig07_threshold_sweep.png` | 阈值 × 驻留扫描热图 |

> 说明：仿真中每阶段站立轨迹 1.5 s（共 3.0 s），与官方 `StandUpState` 默认时长一致；“起身/s”列为 `standup_duration_s`，含 1 s 站稳验证。
