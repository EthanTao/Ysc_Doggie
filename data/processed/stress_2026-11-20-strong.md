# 随机化压力测试（2026-11-20-strong）

> 与正式 9 次矩阵分开统计：正式矩阵保持确定性 nominal，本表为带扰动 stress。
> 扰动：姿态 σ=5.0°，关节 σ=0.06 rad，角速度 σ=20%；每类 10 次，seed 从 100 起。
> 场景：flat；结束条件 max_time=40 s。

## 总览

- 总成功率：**34/40（85.0%）**

| 翻倒类型 | 成功 | 成功率 | 时延均值/s | 时延标准差/s | 起身均值/s | 起身标准差/s | 策略分布 |
|---|---|---|---|---|---|---|---|
| front | 9/10 | 90.0% | 0.155 | 0.000 | 4.21 | 0.00 | direct_standup×7，tuck_then_standup×2，unrecoverable_supine×1 |
| side_left | 9/10 | 90.0% | 0.296 | 0.019 | 4.21 | 0.00 | side_prop_liedown×3，tuck_then_standup×7 |
| side_right | 6/10 | 60.0% | 0.283 | 0.047 | 4.21 | 0.00 | side_prop_liedown×3，tuck_then_standup×6，unrecoverable_supine×1 |
| back | 10/10 | 100.0% | 0.670 | 0.465 | 4.21 | 0.00 | direct_standup×4，tuck_then_standup×6 |

## 失败明细

- `front` trial=3 seed=102：strategy=unrecoverable_supine，E_UNRECOVERABLE_POSE
- `side_left` trial=8 seed=1107：strategy=side_prop_liedown，E_STANDUP_NOT_UPRIGHT
- `side_right` trial=4 seed=2103：strategy=unrecoverable_supine，E_UNRECOVERABLE_POSE
- `side_right` trial=8 seed=2107：strategy=side_prop_liedown，E_STANDUP_NOT_UPRIGHT
- `side_right` trial=9 seed=2108：strategy=side_prop_liedown，E_STANDUP_NOT_UPRIGHT
- `side_right` trial=10 seed=2109：strategy=side_prop_liedown，E_STANDUP_NOT_UPRIGHT

## 结果解读

- 成功率低于 100% 不代表正式矩阵失效：正式 9 次仍是零扰动 nominal 结果；
- 压力测试用于暴露临界工况（例如侧翻落成侧撑后 `LieDown` 无法撤掉支点）；
- 若要提升 stress 成功率，应改恢复策略或注入构造，而不是继续加大 `settle_max_s`。
