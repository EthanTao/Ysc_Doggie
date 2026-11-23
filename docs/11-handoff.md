# 交接文档：背部着地自翻正（路线 2）与真机接入

> 日期：2026-11-23  
> 交接基线：当前工作区（含未提交改动）。建议接手人先跑回归，再打 tag/提交，避免与主路径改动混在一起。  
> 当前主路径：`nominal`，即原来的前/后/侧 65–90° 摔倒场景 + 官方 `JointDamping / LieDown / StandUp`。  
> 本文交接两条独立路线：
> 1. **RL 自翻正策略（路线 2）**：目标是从背部静止着地（仰面）自己翻正；
> 2. **真机接入**：把当前检测/恢复主路径安全地接到 Lite3。

---

## 0. 当前结论与边界（先看这一段）

- 主路径 `nominal` 的正式矩阵为 **9/9 检出、9/9 先安全后起身、9/9 起身**，受阻 3/3，动态路沿绊倒 1/1。
- `full_flip` 是**可选附加实验**：完整翻过 360° 后落回趴地，9/9 起身；默认配置已切回 `nominal`。
- **背部静止着地（四脚朝天、仰面）目前不能自恢复**。已实测：
  - 官方 `StandUp`：不动；
  - 官方 `LieDown → StandUp`：回到仰面；
  - 612 组不同仰面腿姿 + `StandUp`：0 成功；
  - 官方 ONNX 行走策略从仰面“乱蹬”：只能到约 121° 侧撑，随后卡住。
- 结论：背部着地自翻正**必须新增一个非官方的自翻正原语**（RL 策略或自定义关节轨迹）。路线 2 是明确的附加实验，不能混入主路径的 9/9 指标，也不能默认在真机上启用。

---

# 路线 2：RL 自翻正策略交接

## 1. 目标与验收

**目标**：输入背部着地/仰面姿态，输出 12 关节目标（或残差），在仿真中把机器人翻回趴地或可直接接 `StandUp` 的姿态；随后交回官方 `LieDown / StandUp` 完成起身。

**建议验收标准**：

| 指标 | 建议门槛 |
|---|---|
| 静态仰面（roll≈180°）成功率 | ≥90%（≥100 个随机仰面姿态） |
| 从 `full_flip`/背部着地动态过程接入 | ≥80% |
| 翻正后接官方 `StandUp` 成功率 | ≥90% |
| 翻正耗时 | ≤5 s |
| 主路径回归 | `nominal` 9/9 不受影响 |
| 推理延迟 | ≤5 ms/次（12 关节） |
| 安全 | 力矩 ±30 N·m、关节限位、无自碰撞、可随时中止 |

**注意**：不要把 RL 策略的成功率混入原任务书要求的 9/9；它应单独列为“附加实验”，并在报告中明确标注超出官方 `StandUp` 包络。

## 2. 推荐实现方式

### 2.1 训练环境

- 物理模型：官方 `sim/models/lite3/Lite3.xml`（不要改质量/惯量/关节/执行器）。
- 步长：1 ms；策略分频建议 10–20 ms（50–100 Hz）。
- 观测（建议）：
  - 重力方向在机体系的投影（3）；
  - 基座角速度（3）；
  - 12 关节位置（12）；
  - 12 关节速度（12）；
  - 上一步动作（12）；
  - 可选：足端接触/高度估计（4+1）。
- 动作：12 维关节目标增量或绝对目标，经 PD（kp=100, kd=2.5）和 ±30 N·m 限幅执行。
- Reset 分布：
  - roll 150–210°（重点 180°），pitch −30–30°；
  - 关节角在官方限位内随机（HipX ±0.52、HipY −2.67–0.31、Knee 0.52–2.79）；
  - 初始高度 0.05–0.10 m，零/小角速度；
  - 后续加入 `full_flip` 落背瞬间的状态，做动态接入。
- 奖励（建议）：
  - 机身 up 向量与目标（趴地时 z 向上）对齐；
  - 高度/姿态进入“可接 StandUp”包络（|roll|,|pitch|<35°，h>0.10 m）；
  - 关节速度/力矩/动作变化惩罚；
  - 成功终止奖励；超时/自碰撞/越界惩罚。
- 训练算法：PPO（建议 16–64 个并行 MuJoCo 环境），课程学习从“侧撑→仰面→动态落背”。
- 域随机化：质量 ±10%、摩擦 ±20%、IMU 噪声、执行器延迟 0–10 ms、初始关节扰动。

### 2.2 代码接入点（建议，不改主路径）

推荐把实验代码放在独立目录，例如：

```
experiments/self_right/
├── train_ppo.py          # 训练入口
├── env.py                # Gymnasium/MuJoCo 环境封装
├── eval.py               # 批量评估
├── policy.onnx           # 训练产物
└── README.md
```

主仓库只增加一个**默认关闭**的配置块和可选模式，例如：

```yaml
experimental:
  self_right:
    enabled: false
    policy_path: "experiments/self_right/policy.onnx"
    max_time_s: 5.0
    # 只有显式开启 + 独立实验脚本才允许使用
```

推荐接入方式（二选一）：

1. **独立实验 Supervisor（更安全）**：新建 `experiments/self_right/run_self_right.py`，复用 `SimBackend` 的模型/步进与 `RecoveryFSM` 的 `S1/S2B/S3`，只在 `S2` 的 `unrecoverable_supine` 分支插入 RL 翻正；不修改 `src/doggie/core/recovery_fsm.py` 的主状态表。
2. **新增实验模式**：在 `Mode` 中加入 `SELF_RIGHT`，在 `RobotBackend` 增加 `start_self_right()`；`SimBackend` 加载 ONNX；`Ros2Backend` 默认拒绝该模式，只有显式实验配置才映射到自定义底层接口。

**不要**直接把 RL 策略塞进 `S3_STANDUP`；它会污染“复用官方 StandUp”的主路径结论。

### 2.3 与官方动作的衔接

建议顺序：

```
S1 JointDamping（等静止）
  → [实验] RL 自翻正（≤5 s）
  → 重新评估：若 |roll|,|pitch|<35° 且 h>0.10 m
  → 官方 LieDown（按需整姿）
  → 官方 StandUp
  → 官方 S4_VERIFY
```

如果 RL 翻正失败：回到 `S1` 或直接 `S5_SAFE_FAULT`，不要无限重试。

## 3. 现有可复用资产

| 资产 | 路径 | 说明 |
|---|---|---|
| 官方 MJCF 场景 | `sim/models/lite3/Lite3.xml` | 直接作为训练环境基础 |
| 仿真后端 | `src/doggie/io/sim_backend.py` | 1 ms 步进、PD、`place_fall`、`full_flip`、`place_trip` |
| 翻倒 profile | `config/recovery.yaml` → `profiles` | `nominal` 主路径；`full_flip` 可生成“经过仰面”的动态轨迹；仰面静置 reset 需自行采样 |
| 检测/FSM | `src/doggie/core/detector.py`、`recovery_fsm.py` | 主路径不要改；实验可在 `S2` 分支接入 |
| 压力测试 | `scripts/run_stress.py` | 可复用 jitter/seed/CSV 汇总 |
| 失败证据 | `docs/08-feasibility.md` | 官方 StandUp 包络与 612 组仰面失败记录 |
| 可选全翻数据 | `data/processed/stress_2026-11-23.*` | full_flip 32 次压力测试，31/32 |

## 4. 已知风险

- 官方 `RLControl` 在实机有 `PostureUnsafeCheck()`（大倾角会切阻尼），**不能**假设实机允许在仰面直接跑官方行走策略；路线 2 若要上实机，需要新增专用底层接口，属于额外工程/安全审批。
- 仿真中 MuJoCo 接触/摩擦与实机差异大；建议先在仿真做到 ≥90%，再做吊架/软垫受控测试。
- 不要把“翻正”与“起身”混为一个策略；翻正后交回官方 `LieDown/StandUp` 更容易验收和解释。

---

# 真机接入交接

## 5. 当前实现状态

| 组件 | 位置 | 状态 |
|---|---|---|
| 检测器 / FSM | `src/doggie/core/` | 完成，与平台无关 |
| ROS2 后端 | `src/doggie/io/ros2_backend.py` | 已实现：订阅 `/IMU_DATA`、`/JOINTS_DATA`，发布状态/复位/恢复服务 |
| 消息解析 | `src/doggie/io/ros2_convert.py` | 字段别名兼容；离线单测 |
| 高度估计 | `src/doggie/core/kinematics.py` | 腿部 FK + IMU 重力投影 |
| 安全看门狗 | `src/doggie/core/safety.py` | 心跳超时、阻尼确认 |
| 启动脚本 | `scripts/run_real.py` | 默认只读干跑；`--enable-actuation` 才下发 |
| 回环自检 | `scripts/selftest_ros2.py` | 单进程真实 rclpy/DDS 回环 |
| 人工恢复 | `/doggie/recover` (`std_srvs/srv/Trigger`) | 对当前姿态触发一次 S1→S2→S3；不制造摔倒 |
| 复位服务 | `/doggie/reset` | 清除 S5 锁存 |

## 6. 上机前核对清单

1. **消息类型**
   ```bash
   ros2 topic info /IMU_DATA -v
   ros2 topic info /JOINTS_DATA -v
   ```
   默认 `drdds/msg/ImuData`、`drdds/msg/JointsData`；若是 `sensor_msgs/msg/Imu` / `JointState`，改 `ros2.imu_type` / `joints_type`。
2. **关节顺序**：默认 `FL, FR, HL, HR × (HipX, HipY, Knee)`；用 `ros2 topic echo /JOINTS_DATA --once` 对照。
3. **状态切换话题**：`ros2.state_cmd_topic` 默认为空（只读）。确认官方 `kJointDamping / kLieDown / kStandUp` 的下发方式后填入；若走 SDK 调用，替换 `Ros2Backend._publish_state()` 单点。
4. **状态回读**：强烈建议配 `ros2.state_feedback_topic`，否则 `E_DAMPING_NO_ACK` 看门狗自动关闭。
5. **急停**：配 `ros2.estop_topic`（`std_msgs/Bool`）或确认硬件急停链路。
6. **IMU 零偏**：上电直立静置，`python scripts/run_real.py --duration 30 --calibrate 2`。
7. **自检**：`python scripts/selftest_ros2.py --duration 15` 应为 PASS。

## 7. 四阶段验收

| 阶段 | 命令 | 通过标准 |
|---|---|---|
| 一 话题/姿态/零偏 | `run_real.py --duration 60 --calibrate 2` | 终端 roll/pitch 与手持倾角一致；30° 不触发 |
| 二 只检不立 | 同上，人工缓推 55°/80° | 打印 `FallEvent`；`detect_delay_s` ≤0.35 s；建议干跑 |
| 三 完整恢复 | `run_real.py --duration 300 --enable-actuation` | S1→S2→(S2B)→S3→S4→S0；先安全后起身为真 |
| 四 受阻锁存 | 软垫轻压背部限制抬升 <0.15 m | 2 次尝试后 `S5_SAFE_FAULT` + `E_STANDUP_NOT_UPRIGHT`；`/doggie/reset` 后复位 |

现场必须：≥5 cm 软垫、龙门吊架、三人分工、硬件急停可用；`safety.allow_real_fall_test: false` 保持不变，不自由摔倒。

## 8. 指标口径

- 正式矩阵 `latency_s` 是从运行开始到确认；实机常驻看 `detect_delay_s`（首个判据越限→确认）。
- 主路径 nominal：前翻 ≈0.155 s、侧翻 ≈0.305 s、受阻 ≈0.320 s。
- 可选 full_flip：高速翻转通道 0.05 s；该数据只用于附加实验。
- 采样率若为 100 Hz，用 `--det-hz 100` 同步低通系数，不要手改配置。

## 9. 已知差距（提前说明）

1. 背部静止着地不能自恢复（见第 0 节）；路线 2 未完成前，实机仍按 `E_UNRECOVERABLE_POSE` 安全锁存。
2. 仿真高度是 `qpos[2]` 真值，实机用 FK 估计（偏低约 2–2.5 cm，方向安全）；门限保留裕度。
3. 无足端力话题时，触地数仅用于记录，不参与判定。
4. 未配状态回读时，阻尼确认不可观测，看门狗跳过而不是误报。
5. 通信中断只在 `E_SENSOR_STALE` 且配置了急停链路时触发硬急停。

## 10. 交接产物清单

- 主路径配置：`config/recovery.yaml`（默认 `nominal`；`full_flip` 可选）
- 仿真后端：`src/doggie/io/sim_backend.py`
- ROS2 后端：`src/doggie/io/ros2_backend.py`
- 实机入口：`scripts/run_real.py`、`scripts/selftest_ros2.py`
- 实机文档：`docs/09-real-machine.md`
- 主矩阵：`data/processed/matrix_2026-11-19.csv`、`summary.md`
- 可选全翻数据：`data/processed/stress_2026-11-23.*`、`videos/full_flip/`
- 仿真证据：`docs/report/sim_shots/`、`docs/report/data-pack.md`

> 接手建议：先跑 `python -m pytest -q`、`python scripts/selftest_ros2.py --duration 15`、`python scripts/run_matrix.py --suite all` 三条回归，再开新分支做路线 2；不要在主分支直接改 `RecoveryFSM` 的官方状态表。
