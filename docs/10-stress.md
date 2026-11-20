# 参数单一来源与随机化压力测试（P0 / P1）

> 日期：2026-11-20
> 范围：`config/recovery.yaml` 参数治理 + 正式矩阵之外的 stress 套件
> 结论：正式 9 次仍为确定性 nominal **9/9**；带扰动 stress 为 **117/120（97.5%）**，失败集中在右侧翻的侧撑工况。

## 1. 为什么做这两件事

正式矩阵的 9 次试验是确定性注入：固定姿态、零初速、无噪声。`md5sum` 显示
`front_1/2/3`、`back_1/2/3`、`blocked_1/2/3` 的时间序列完全相同，`side_1` 与
`side_3` 也完全相同。因此“3 次重复”不是独立样本，不能支撑鲁棒性结论。

同时，`config/recovery.yaml` 虽然标注“唯一参数源”，但有多处参数未接线或重复定义：

- `sim.stand.pre_height_m` / `stand_height_m` 未生效，实际使用模块常量；
- 预站立姿态 `Q_PRE` 在 `sim_backend.py` / `recovery_fsm.py` / `render_sim_keyframes.py` 三处重复；
- `Supervisor` 硬编码 200 Hz / 100 Hz，不读 `detector.rate_hz` / `logging.rate_hz`；
- `standup_traj_s`、`liedown_traj_s` 与 `duration_per_phase_s` 需手工保持一致；
- `sim.trot.*` 未接线，`policy.py` 硬编码 `DECIMATION` / `MAX_CMD`。

P0 修参数一致性，P1 增加可复现的随机化压力测试；两者都不改变正式 9 次矩阵。

## 2. P0：参数单一来源

| 改动 | 结果 |
|---|---|
| 新增 `src/doggie/core/params.py` | `Q_PRE`、`PRE_HEIGHT`、`STAND_HEIGHT`、二连杆 IK、RPY 转换只定义一次 |
| 新增 `src/doggie/config.py` | `resolve_config()` 加载时校验并展开参数 |
| `sim_backend.py` | `pre_height_m` / `stand_height_m` / `pre_pose_rad` 真正生效；RL 的 decimation 与指令上限从 `sim.trot` 读取 |
| `recovery_fsm.py` | 从 `core.params` 引用 `Q_PRE`；FSM 的 `q_pre` 由 `sim.stand.pre_pose_rad` 注入 |
| `supervisor.py` / `trial.py` | 采样率由配置驱动；检测与记录同帧时复用同一帧观测 |
| `scripts/sim_smoke_test.py` | 删除重复的 IK / 常量 / RPY 实现 |
| `scripts/watch_sim.py` / `render_sim_keyframes.py` | 检测频率由 `detector.rate_hz` 推导 |
| `scripts/record_normal.py` | 检测器更新频率与日志频率统一为 `logging.rate_hz`（保持 100 Hz 行为） |
| 配置注释 | 明确标注 `use_contact`、`quiet_omega_rad_s`、`resume`、`safety` 等为预留项 |

`resolve_config()` 会在以下情况直接报错，避免“改了配置但不生效”：

- `standup_traj_s != 2 × duration_per_phase_s`；
- `liedown_traj_s != duration_per_phase_s`；
- `fsm.pre_pose_rad` 与 `sim.stand.pre_pose_rad` 不一致；
- `rate_hz × timestep` 不是整数步。

核心回归测试见 `tests/test_params.py`（改 `stand_height_m` 后站立目标确实变化、配置不一致会报错等）。

## 3. P1：随机化压力测试

### 3.1 方法

- 在 `place_fall()` 的初始姿态、关节角、角速度上叠加高斯扰动；
- 扰动后仍用 `mj_geomDistance` 重新留 5 mm 间隙，避免初始穿透；
- `jitter=None` 时完全保持正式矩阵的确定性行为；
- 每个 `(fall, repeat)` 使用固定 seed，单次试验可复现；
- 输出 `data/processed/stress_<date>.csv` 与 `.md`，不覆盖正式 `summary.md`。

命令：

```bash
# 常规扰动：σ_att=2.5°, σ_joint=0.03 rad, σ_rate=10%
python scripts/run_stress.py --repeats 30 --seed 0

# 强扰动：σ_att=5°, σ_joint=0.06 rad, σ_rate=20%
python scripts/run_stress.py --repeats 10 --seed 100 \
  --attitude-deg 5 --joint-rad 0.06 --rate-rel 0.2 \
  --date 2026-11-20-strong
```

### 3.2 常规扰动结果（30 次/类，共 120 次）

| 翻倒类型 | 成功 | 成功率 | 时延均值 ± 标准差 / s | 起身均值 / s |
|---|---|---|---|---|
| front | 30/30 | 100.0% | 0.155 ± 0.000 | 4.21 |
| side_left | 30/30 | 100.0% | 0.304 ± 0.003 | 4.21 |
| side_right | **27/30** | **90.0%** | 0.304 ± 0.006 | 4.21 |
| back | 30/30 | 100.0% | 0.510 ± 0.275 | 4.21 |
| **合计** | **117/120** | **97.5%** | — | 4.21 |

失败明细：

- `side_right` seed=2016 / 2019 / 2025；
- 策略均为 `side_prop_liedown`，故障码 `E_STANDUP_NOT_UPRIGHT`；
- 现象：扰动后机体停在侧撑（约 57°），`LieDown` 以触地脚为支点无法把机身带回趴地，两次尝试后安全锁存。

`back` 的时延标准差较大（0.510 ± 0.275 s），是因为部分扰动样本落到高度通道（0.98 s）而非姿态通道（0.305 s），但全部仍小于 2 s。

### 3.3 强扰动结果（10 次/类，共 40 次）

| 翻倒类型 | 成功 | 成功率 | 时延均值 ± 标准差 / s |
|---|---|---|---|
| front | 9/10 | 90.0% | 0.155 ± 0.000 |
| side_left | 9/10 | 90.0% | 0.296 ± 0.019 |
| side_right | **6/10** | **60.0%** | 0.283 ± 0.047 |
| back | 10/10 | 100.0% | 0.670 ± 0.465 |
| **合计** | **34/40** | **85.0%** | — |

强扰动下出现两类失败：

1. `side_prop_liedown`：侧撑无法撤掉支点（主要失败模式）；
2. `unrecoverable_supine`：扰动把机身推成仰翻/倒立，超出平台 `StandUp` 包络，直接安全拒止。

## 4. 结论与后续

- 正式矩阵仍应报告为 **nominal 9/9**，并在报告中同时给出 stress 成功率，避免“三份相同数据”被质疑；
- 当前系统的主要鲁棒性瓶颈是**侧翻落成侧撑**，不是检出阈值或起身时长；
- 继续增大 `settle_max_s` 无效（侧撑不会自行回翻）；有效方向是：
  - 侧撑时增加一次绕 roll 的脉冲/重心转移；
  - 或改用带前向动量的侧翻注入（与 `docs/08-feasibility.md` 的 60° + 3.0 rad/s 一致）；
- 时延与起身时间已远低于优秀档上限（≤2 s / ≤15 s），不建议为分数继续压低 `dwell_s` 或 `verify_s`。
