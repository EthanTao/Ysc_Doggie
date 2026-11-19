# 任务二测试矩阵汇总（2026-11-19）

## 正式 9 次（前/后/侧各 3）
- 摔倒判定：9/9
- 判定时延：min 0.15s / max 0.30s
- 先安全后起身（顺序正确）：9/9
- 起身成功：9/9（平均耗时 5.2s）

## 起身受阻 3 次
- 停在安全状态并给出提示：3/3
- 重试上限（应全部 ≤2）：2, 2, 2

## 明细
| run | fall | scene | detected | kind | latency_s | order_ok | attempts | recovered | standup_s | final | fault |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 2026-11-19_front_1 | front | flat | True | front | 0.155 | True | 1 | True | 5.21 | S0_MONITOR |  |
| 2026-11-19_front_2 | front | flat | True | front | 0.155 | True | 1 | True | 5.21 | S0_MONITOR |  |
| 2026-11-19_front_3 | front | flat | True | front | 0.155 | True | 1 | True | 5.21 | S0_MONITOR |  |
| 2026-11-19_side_1 | side_left | flat | True | side_left | 0.305 | True | 1 | True | 5.21 | S0_MONITOR |  |
| 2026-11-19_side_2 | side_right | flat | True | side_right | 0.305 | True | 1 | True | 5.21 | S0_MONITOR |  |
| 2026-11-19_side_3 | side_left | flat | True | side_left | 0.305 | True | 1 | True | 5.21 | S0_MONITOR |  |
| 2026-11-19_back_1 | back | flat | True | back | 0.305 | True | 1 | True | 5.21 | S0_MONITOR |  |
| 2026-11-19_back_2 | back | flat | True | back | 0.305 | True | 1 | True | 5.21 | S0_MONITOR |  |
| 2026-11-19_back_3 | back | flat | True | back | 0.305 | True | 1 | True | 5.21 | S0_MONITOR |  |
| 2026-11-19_blocked_1 | prone | blocked | True | flat | 0.32 | True | 2 | False |  | S5_SAFE_FAULT | E_STANDUP_NOT_UPRIGHT |
| 2026-11-19_blocked_2 | prone | blocked | True | flat | 0.32 | True | 2 | False |  | S5_SAFE_FAULT | E_STANDUP_NOT_UPRIGHT |
| 2026-11-19_blocked_3 | prone | blocked | True | flat | 0.32 | True | 2 | False |  | S5_SAFE_FAULT | E_STANDUP_NOT_UPRIGHT |
