# 任务二测试矩阵汇总（2026-11-19）
- profile：`nominal`

## 正式 9 次（前/后/侧各 3）
- 摔倒判定：9/9
- 判定时延：min 0.155s / max 0.305s
- 先安全后起身（顺序正确）：9/9
- 起身成功：9/9（平均耗时 4.2s）
- 累计翻转角度（注入+运行）：183–211°

## 起身受阻 3 次
- 停在安全状态并给出提示：3/3
- 重试上限（应全部 ≤2）：2, 2, 2

## 场景触发挑战
- 小跑撞上 11 cm 路沿：检测 1/1
- 首个判据越限至确认：0.305–0.305 s
- 恢复成功：1/1；安全锁存：0/1

## 正常小跑（误报测试）
- 3 段，合计 180 s
- 误报：0 次
- 正常姿态包络：|roll| ≤ 1.06°，|pitch| ≤ 1.71°，高度 ≥ 0.319 m

## 明细
| run | fall | scene | profile | detected | kind | latency_s | detect_delay_s | order_ok | strategy | attempts | recovered | standup_s | flip_deg | final | fault |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 2026-11-19_front_1 | front | flat | nominal | True | front | 0.155 | 0.15 | True | direct_standup | 1 | True | 4.21 | 200.8 | S0_MONITOR |  |
| 2026-11-19_front_2 | front | flat | nominal | True | front | 0.155 | 0.15 | True | direct_standup | 1 | True | 4.21 | 200.8 | S0_MONITOR |  |
| 2026-11-19_front_3 | front | flat | nominal | True | front | 0.155 | 0.15 | True | direct_standup | 1 | True | 4.21 | 200.8 | S0_MONITOR |  |
| 2026-11-19_side_1 | side_left | flat | nominal | True | side_left | 0.305 | 0.3 | True | tuck_then_standup | 1 | True | 4.21 | 183.1 | S0_MONITOR |  |
| 2026-11-19_side_2 | side_right | flat | nominal | True | side_right | 0.305 | 0.3 | True | tuck_then_standup | 1 | True | 4.21 | 185.4 | S0_MONITOR |  |
| 2026-11-19_side_3 | side_left | flat | nominal | True | side_left | 0.305 | 0.3 | True | tuck_then_standup | 1 | True | 4.21 | 183.1 | S0_MONITOR |  |
| 2026-11-19_back_1 | back | flat | nominal | True | back | 0.305 | 0.3 | True | direct_standup | 1 | True | 4.21 | 211.0 | S0_MONITOR |  |
| 2026-11-19_back_2 | back | flat | nominal | True | back | 0.305 | 0.3 | True | direct_standup | 1 | True | 4.21 | 211.0 | S0_MONITOR |  |
| 2026-11-19_back_3 | back | flat | nominal | True | back | 0.305 | 0.3 | True | direct_standup | 1 | True | 4.21 | 211.0 | S0_MONITOR |  |
| 2026-11-19_blocked_1 | prone | blocked | nominal | True | flat | 0.32 | 0.3 | True | tuck_then_standup | 2 | False |  | 46.7 | S5_SAFE_FAULT | E_STANDUP_NOT_UPRIGHT |
| 2026-11-19_blocked_2 | prone | blocked | nominal | True | flat | 0.32 | 0.3 | True | tuck_then_standup | 2 | False |  | 46.7 | S5_SAFE_FAULT | E_STANDUP_NOT_UPRIGHT |
| 2026-11-19_blocked_3 | prone | blocked | nominal | True | flat | 0.32 | 0.3 | True | tuck_then_standup | 2 | False |  | 46.7 | S5_SAFE_FAULT | E_STANDUP_NOT_UPRIGHT |
| 2026-11-19_trip_1 | trip | trip | nominal | True | flat | 2.695 | 0.305 | True | tuck_then_standup | 1 | True | 4.21 | 374.0 | S0_MONITOR |  |
