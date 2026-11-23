# Lite3 摔倒检测与恢复运控 —— 答辩 PPT 逐页稿与 AI 生成提示词

> **适用场景**：《四足机器人 Lite3 实践任务书（修订版）》任务二（恢复运控）结题验收与现场答辩  
> **建议答辩时长**：8–10 分钟演讲 + 5 分钟评委问答（共 14 页核心幻灯片）  
> **配套报告**：`docs/06-report.md`  
> **配置与代码基线**：`config/recovery.yaml`（`0f4ace0f9e2d`） / Git Commit `615d7b0` + 工作区

> **2026-11-23 说明**：主路径已恢复为 `nominal`（原始 65–90° 摔倒场景，9/9，0.155–0.305 s）；代码中保留了可选 `full_flip` 附加实验与高速翻转检测通道，不参与主矩阵。背部着地自翻正作为路线 2 单独交接（`docs/11-handoff.md`）。

---

## 第一部分：14 页 PPT 逐页完整内容（双栏版式 + 演讲者逐字稿）

---

### P1：封面与一句话方案

- **页面主标题**：四足机器人 Lite3 摔倒检测与自主恢复运控系统（任务二）
- **副标题**：基于三通道迟滞判据与官方原语四路姿态分流的高可靠恢复状态机（Doggie Recovery Supervisor）
- **核心结论金句（Banner）**：
  > **“不重新发明底层步态，而是构建一个可解释、可标定、100% 严守‘先安全阻尼、后整姿起身’的智能监督决策层——正式 9 次翻倒 100% 检出并恢复（平均起身 4.21 s），3 次受阻 100% 限次锁存，180 s 正常小跑 0 误报。”**
- **左栏（基本信息与交付物矩阵）**：
  - **考核任务**：任务二 —— 恢复运控（摔倒检测 + 状态机触发 + 起身动作调用 + 失败锁存）
  - **实验平台**：云深处绝影 Lite3（无激光雷达版） / MuJoCo 4.0 官方 MJCF 物理试验台
  - **五大完整交付物**：
    1. 唯一参数源配置：`config/recovery.yaml`（SHA-256: `0f4ace0f9e2d`）
    2. 纯逻辑核心代码：`FallDetector` + `RecoveryFSM` + `RobotBackend`
    3. 全量实测数据集：`data/raw/2026-11-19_*`（9 正式 + 3 受阻 + 3×60 s 正常小跑）
    4. 56 组网格标定数据：`data/calibration/threshold_sweep.csv`
    5. 未剪辑全流程视频：`videos/demo_mjcf_{front,side,back,blocked}.mp4`
- **右栏（配图/视频具体文件名）**：
  - 主视觉图：`docs/report/sim_shots/grid_overview.png`（五大典型场景全流程仿真序列总览网格图）
  - 辅助小图：`docs/assets/sim_smoke.png`（官方 Lite3 模型标准站立姿态 $h = 0.346\text{ m}$）
- **演讲者逐字备注（约 40 秒）**：
  > “各位评委老师好！今天由我代表团队汇报四足机器人 Lite3 任务二‘恢复运控’的研制成果。任务二的核心不是重新训练一套端到端强化学习控制器，而是回答两个工程问题：第一，用什么本体感知迹象可靠判定摔倒且不误报？第二，判定后按什么顺序与条件调用平台动作安全恢复？为此，我们构建了 Doggie 恢复监督系统，在不篡改平台底层控制器的前提下，通过多判据融合检测与四路官方状态分流状态机，实现了正式 9 次翻倒 9/9 检出与恢复、平均起身耗时仅 4.21 秒、180 秒小跑 0 误报、3 次受阻 100% 安全锁存，五项指标全部达到任务书优秀档满分标准。”

---

### P2：任务拆解与评分标准对齐（5 项全优秀档对照）

- **页面标题**：任务拆解与考核指标全面对齐（表 7 优秀档逐项核验）
- **核心结论金句（Banner）**：
  > **“以任务书表 7 优秀档为唯一设计基准：每一项得分点均有自动化脚本断言、100 Hz 原始时间序列与未剪辑视频三重证据闭环。”**
- **左栏（问题拆解与工程边界）**：
  - **两个核心问题**：
    1. **判定问题**：如何兼顾大倾角极速响应（$\le 0.31\text{ s}$）、常规行走零误报（$0 / 180\text{ s}$）与水平趴地零倾角盲区兜底？
    2. **决策问题**：如何从状态拓扑上强制“先安全阻尼卸力、后起身”，并在受阻时严格限次（2 次）锁存不无限挣扎？
  - **严守工程边界**：
    - **直接复用平台能力**：`JointDamping`（阻尼）、`LieDown`（收腿）、`StandUp`（两阶段样条+IK 起身）、`RLControl`（`policy.onnx` 小跑）。
    - **自研核心价值**：`FallDetector` 多通道迟滞判定器 + `RecoveryFSM` 七状态机与 `S2_ASSESS` 姿态分流策略。
- **右栏（五项评分标准与实测结果对照总表）**：

  | 评分项（分值） | 任务书优秀档要求 | 本项目实测指标 | 证据链与结论 |
  |---|---|---|---|
  | **摔倒检测 (25分)** | 9/9 正确，0 误报，时延 $\le 2\text{ s}$ | **9/9 检出**，时延 **0.155–0.305 s**（均值 0.255 s），小跑 **0/180 s 误报** | `fig01`, `fig03`<br>✅ **优秀** |
  | **恢复流程 (25分)** | 9/9 先安全后起身，条件清晰 | **9/9 满足 `t_damping < t_standup`**，含 `S2_ASSESS` 四路分流 | `fig05`, `meta.json`<br>✅ **优秀** |
  | **起身完成 (25分)** | 9/9 成功，平均耗时 $\le 15\text{ s}$ | **9/9 一次成功**，平均耗时 **4.21 s**（3.2 s 轨迹 + 1.0 s 验证） | `fig06`, 4 段 MP4<br>✅ **优秀** |
  | **失败与超时 (15分)** | 3/3 停安全状态并提示，无无限重试 | **3/3 在 2 次尝试后锁存 `S5_SAFE_FAULT`** (`E_STANDUP_NOT_UPRIGHT`) | `strip_blocked.png`<br>✅ **优秀** |
  | **参数依据 (10分)** | 给出分布、回差与驻留标定数据 | 正常包络 $\le 1.71^\circ$（裕度 **48.3°**），**56 组网格扫描热图** | `fig04`, `fig07`<br>✅ **优秀** |

- **演讲者逐字备注（约 45 秒）**：
  > “请看第二页。我们在立项之初就将任务书第八章表 7 的 5 个评分项拆解为可量化的自动化断言。右表展示了我们的实测成绩单：第一项摔倒检测（25分），9 次全部检出，判定时延最快 0.155 秒、最慢 0.305 秒，不到优秀档 2 秒上限的六分之一，且在官方 ONNX 策略驱动的 180 秒连续小跑中实现 0 误报；第二项恢复流程（25分），9 次全部严格满足先阻尼后起身的时间戳断言 `t_damping < t_standup`；第三项起身完成（25分），9 次全部一次尝试成功，平均耗时 4.21 秒；第四项失败与超时（15分），3 次矮障碍受阻实验均在 2 次尝试后可靠锁存至 `S5_SAFE_FAULT` 并输出故障码；第五项参数依据（10分），完成了 56 组阈值与驻留时间的二维网格扫描标定。”

---

### P3：总体架构：决策与执行解耦（`FallDetector` + `RecoveryFSM` + `RobotBackend`）

- **页面标题**：系统总体架构：纯逻辑决策层与底层执行器解耦
- **核心结论金句（Banner）**：
  > **“核心算法不依赖一行 `mujoco` 或 `rclpy` 代码：通过 `Observation` 与 `RobotBackend` 抽象层实现仿真、离线回放与实机 ROS2 无缝切换。”**
- **左栏（三层解耦设计要点）**：
  - **1. 标准化观测接口 `Observation`（200 Hz）**：
    - 统一封装时间戳 $t$、欧拉角 $(\text{roll}, \text{pitch}, \text{yaw})$、体角速度 $\boldsymbol{\omega}_b \in \mathbb{R}^3$、12 关节位置 $\mathbf{q}$ 与速度 $\dot{\mathbf{q}}$、躯干高度 $h$、触地足数 $n_{\text{feet}}$。
  - **2. 纯逻辑核心层（`src/doggie/core/`）**：
    - `FallDetector`（`detector.py`）：滤波 $\rightarrow$ 三通道判据 $\rightarrow$ 锁存/回差 $\rightarrow$ 输出结构化 `FallEvent(t, kind, cue)`。
    - `RecoveryFSM`（`recovery_fsm.py`）：7 状态转移 $\rightarrow$ 四路姿态评估分流 $\rightarrow$ 输出控制模式枚举 `Mode`。
  - **3. 多后端执行层（`src/doggie/io/`）**：
    - `SimBackend`（MuJoCo 4.0）：加载官方 Lite3 MJCF、官方 `StandUp/LieDown` 样条与 `policy.onnx`。
    - `Ros2Backend`（实机对接）：映射 `/IMU_DATA`、`/JOINTS_DATA`、官方状态机切换指令与 `/doggie/status`。
  - **4. 唯一参数源（`config/recovery.yaml`）**：
    - 严禁代码硬编码魔法数字；每次实验自动记录配置 SHA-256 哈希（`0f4ace0f9e2d`），确保 100% 可复现。
- **右栏（配图/架构框图）**：
  - 插入架构图（对应报告图 1-1 `RecoverySupervisor` 模块数据流图）：展示 `Observation -> FallDetector -> FallEvent -> RecoveryFSM -> Mode(DAMPING/LIEDOWN/STANDUP/HOLD/RL) -> RobotBackend (SimBackend / Ros2Backend)`。
  - 辅助截图：`docs/assets/sim_smoke.png`
- **演讲者逐字备注（约 40 秒）**：
  > “在工程架构上，我们坚持‘决策与执行彻底分离’。如右图所示，上层的 `FallDetector` 和 `RecoveryFSM` 是纯 Python 数学逻辑，它们只认识标准的 `Observation` 数据结构和五种对应官方状态机的控制模式枚举 `Mode`（阻尼、卧倒收腿、站立起身、保持、RL 行走）。底层通过 `RobotBackend` 抽象基类分出两条路径：一条是搭载官方 MJCF 和官方 ONNX 策略的 MuJoCo 仿真后端，用于高通量自动化测试与参数标定；另一条是对接官方 `/IMU_DATA` 和 `/JOINTS_DATA` 话题的 ROS2 实机后端。所有参数集中存放在唯一的 `recovery.yaml` 中并通过哈希校验，保证了仿真标定好的代码无需修改一行就能直接迁移到实机。”

---

### P4：摔倒检测算法：滤波、多判据融合、驻留与回差

- **页面标题**：摔倒判定算法设计（`FallDetector`）：快、准、无盲区
- **核心结论金句（Banner）**：
  > **“双层三通道并联融合：75° 快速通道保极速响应（0.155 s），50° 主通道抗冲击防误报（0.305 s），0.10 m 高度通道破除水平趴地零姿态角盲区。”**
- **左栏（算法五步流水线与数学判据）**：
  - **Step 1：200 Hz 采样 + 10 Hz 一阶低通滤波**
    - $\alpha = 1 - e^{-2\pi f_c / f_s} \approx 0.2696$（时间常数仅 $15.9\text{ ms}$），滤除足端触地冲击毛刺，不改变原始日志记录。
  - **Step 2：三通道并联持续确认（OR 逻辑）**
    - **通道 c（快速通道 `fast_path`）**：$\max(|\tilde{\phi}|, |\tilde{\theta}|) > 75.0^\circ$ 连续保持 **$0.15\text{ s}$** $\rightarrow$ 前翻/剧烈翻倒极速切断控制。
    - **通道 a（主姿态通道 `attitude`）**：$|\tilde{\phi}| > 50.0^\circ$ 或 $|\tilde{\theta}| > 50.0^\circ$ 连续保持 **$0.30\text{ s}$** $\rightarrow$ 侧翻/后翻主力判据，过滤越障瞬态尖峰。
    - **通道 b（高度兜底 `height`）**：躯干高度 $h < 0.10\text{ m}$ 连续保持 **$0.30\text{ s}$** $\rightarrow$ **解决趴地/矮架受阻时 $\text{roll}\approx 0^\circ, \text{pitch}\approx 0^\circ$ 的姿态盲区**（且低于官方 `LieDown` 预站立高度 $0.12\text{ m}$，留有 $2\text{ cm}$ 安全间隔）。
  - **Step 3：事件锁存（Latch）与 35° / 0.50 s 迟滞回差**
    - 触发后 `latched = True`；须连续 $0.50\text{ s}$ 满足 $|\tilde{\phi}| < 35^\circ \land |\tilde{\theta}| < 35^\circ$ 且由 FSM 在 `S4_VERIFY` 通过后调用 `clear()` 才解除，**起身过程重触发次数 = 0**。
  - **Step 4：基于历史峰值的五分类（`_classify`）**
    - 追踪全过程峰值 $(\phi_{\text{peak}}, \theta_{\text{peak}})$，按 $60^\circ$ 门限与高度精确区分 `front / back / side_left / side_right / flat`（12/12 准确率 100%）。
- **右栏（与官方内置 `PostureUnsafeCheck()` 对比表 + 配图）**：
  - 配图：`docs/report/figures/fig02_attitude_timeline.png`（展示 50° 触发阈值线与 35° 回差线）

  | 对比项 | 官方 `PostureUnsafeCheck()` | 本项目 `FallDetector` |
  |---|---|---|
  | 滤波与驻留 | 无滤波，**单帧瞬时触发** | **10 Hz 低通 + 0.15s/0.30s 驻留** |
  | 触发门限 | `|roll|>30°` 或 `|pitch|>45°` | **50° 主门限 + 75° 快速通道** |
  | 趴地零倾角盲区 | **无法检出**（趴地倾角≈0°） | **`h < 0.10 m` 持续 0.30 s 兜底** |
  | 回差与类型诊断 | 无回差，仅输出 `bool` | **35°/0.50s 回差 + 历史峰值五分类** |

- **演讲者逐字备注（约 50 秒）**：
  > “第四页重点介绍摔倒检测算法。我们对比了云深处官方代码里的 `PostureUnsafeCheck`，官方做法是单帧瞬时判断 roll 超 30 度或 pitch 超 45 度，这在跳跃、急停时容易误触发，更致命的是当机器狗坍塌水平趴在地上、或者被矮障碍物压住时，roll 和 pitch 都接近 0 度，纯姿态阈值会彻底失明。为此，我们设计了 10 Hz 低通滤波加三通道并联判据：第一，倾角超 75 度持续 0.15 秒走快速通道，前翻在 0.155 秒内极速检出；第二，倾角超 50 度持续 0.30 秒走主姿态通道，稳定检出侧翻与后翻；第三，躯干高度低于 0.10 米持续 0.30 秒走高度兜底通道，专门解决水平趴地盲区，而且 0.10 米刚好比官方 LieDown 的 0.12 米低 2 厘米，不会误判正常卧倒。触发后事件锁存，并设置 35 度、0.5 秒迟滞回差，确保起身晃动时绝不重复触发。”

---

### P5：为什么选 50° / 0.3s？—— 正常包络 vs 翻倒分布与网格标定

- **页面标题**：参数定标依据：3×60 s 正常小跑包络与 56 组网格扫描热力图
- **核心结论金句（Banner）**：
  > **“正常小跑最大姿态角仅 1.71°，距 50° 阈值留有 48.29°（29.2 倍）安全裕度；56 组二维网格扫描定量证明 (50°, 0.30 s) 处于检出不退化的最优工作点。”**
- **左栏（正常小跑实测包络与网格标定定量结论）**：
  - **1. 官方 `policy.onnx` 正常小跑 3×60 s（180 s / 18,000 帧，100 Hz 记录；$v_x$ 为归一化指令）实测**：
    - `normal_turn` ($v_x=0.3, \omega_z=0.5$)：$\max|\phi|=0.62^\circ, \max|\theta|=1.56^\circ, \min h=0.319\text{ m}$，**0 误报**。
    - `normal_vx0.4` ($v_x=0.4$，实测均速约 $0.25\text{ m/s}$)：$\max|\phi|=\mathbf{1.06^\circ}, \max|\theta|=1.41^\circ, \min h=0.320\text{ m}$，**0 误报**。
    - `normal_vx0.6` ($v_x=0.6$，实测均速约 $0.44\text{ m/s}$)：$\max|\phi|=0.93^\circ, \max|\theta|=\mathbf{1.71^\circ}, \min h=0.321\text{ m}$，**0 误报**。
  - **2. 56 组网格扫描（阈值 $40^\circ\text{–}70^\circ \times$ 驻留 $0.1\text{–}1.0\text{ s}$）核心发现**：
    - **为什么不选 $\ge 65^\circ$？** 侧翻初始倾角 $65^\circ$，落地后在阻尼下向趴地回落；当阈值 $\ge 65^\circ$ 时主姿态通道漏检，**最大时延从 $0.31\text{ s}$ 阶跃恶化至 $0.98\text{ s}$**（退化为等趴平后靠高度兜底）。选 **$50^\circ$** 上留 $15^\circ$ 检出裕度，下留 $48.29^\circ$ 防误报裕度。
    - **为什么不选 $\ge 0.50\text{ s}$？** 当驻留 $\ge 0.50\text{ s}$ 时，侧翻在 $0.45\text{ s}$ 已回落至 $50^\circ$ 以内导致姿态计时中断，时延同样突跳至 $0.98\text{ s}$。选 **$0.30\text{ s}$** 兼具抗瞬态冲击与 $0.305\text{ s}$ 快检能力。
- **右栏（配图具体文件名 —— 三图组合）**：
  - 左上图：`docs/report/figures/fig03_normal_envelope.png`（3×60 s 正常小跑 Roll/Pitch/Height 全程包络）
  - 左下图：`docs/report/figures/fig04_normal_vs_fall.png`（正常包络 $\le 1.71^\circ$ vs 翻倒分布 $\ge 65^\circ$ 与 $50^\circ/35^\circ$ 裕度带）
  - 右大图：`docs/report/figures/fig07_threshold_sweep.png`（阈值 × 驻留二维网格扫描时延热力图，星号标注 `50° / 0.30 s`）
- **演讲者逐字备注（约 50 秒）**：
  > “评委老师一定会问：50 度阈值、0.3 秒驻留和 35 度回差是怎么定出来的？请看第五页的三张实测标定图。首先看左上图和左下图，我们用官方 ONNX 强化学习策略跑了三段各 60 秒的正常小跑和转向，实测最大横滚角只有 1.06 度、最大俯仰角 1.71 度，而翻倒姿态全部在 65 度到 90 度以上。50 度阈值距离正常包络有高达 48.29 度的巨大安全裕度。更关键的是右边的二维网格热力图：我们回放全部实验数据扫描了 40 到 70 度、0.1 到 1.0 秒共 56 组参数组合。热力图清晰揭示了一个物理台阶：如果阈值设到 65 度以上，或者驻留时间设到 0.5 秒以上，侧翻机器狗在回落过程中倾角已经掉到阈值以下，导致姿态通道失效，检出时延会从深蓝色的 0.31 秒陡然恶化到橙红色的 0.98 秒！因此，星号所在的 50 度、0.30 秒正处于左上方深蓝色低时延平原的几何中心，是经过严格数据标定的最优工作点。”

---

### P6：恢复状态机设计：强制“先安全阻尼、后起身”

- **页面标题**：7 状态恢复有限状态机（`RecoveryFSM`）：拓扑级安全顺序保证
- **核心结论金句（Banner）**：
  > **“从图拓扑结构上消除‘带电直接硬起’的可能：`S1_SAFE_DAMPING` 是通往起身的唯一必经前驱，9/9 实验 100% 满足 `t_damping < t_standup_cmd`。”**
- **左栏（状态定义与“先安全后起身”三重约束）**：
  - **7 个核心状态与官方模式对齐**：
    - `S0_MONITOR` $\rightarrow$ 正常监控（`HOLD / RL`）
    - **`S1_SAFE_DAMPING`** $\rightarrow$ **强制首入安全阻尼**（`Mode.DAMPING`, $\tau = -2.5\dot{q}$）
    - `S2_ASSESS` $\rightarrow$ 静止后姿态评估、退避计时与四路策略分流（保持 `DAMPING`）
    - `S2B_PREPARE_LIEDOWN` $\rightarrow$ 按需调用官方 `LieDown` 收腿整姿（`Mode.LIEDOWN`, $1.5\text{ s}+0.3\text{ s}$）
    - **`S3_STANDUP`** $\rightarrow$ 调用官方两阶段 `StandUp` 起身（`Mode.STANDUP`, $3.0\text{ s}$）
    - `S4_VERIFY` $\rightarrow$ 起身后连续 $1.0\text{ s}$ 直立稳定验证（$|\phi|,|\theta|<10^\circ, h>0.25\text{ m}$）
    - **`S5_SAFE_FAULT`** $\rightarrow$ 安全故障锁存（纯阻尼 + 故障码，仅响应 `manual_reset()`）
  - **为什么能 100% 保证“先安全后起身”？**
    1. **拓扑唯一性**：`S0` 只能转 `S1`，无任何直达 `S2B/S3` 的边；
    2. **时间与动力学双重静置门限**：在 `S1` 中必须停留 $\ge 0.50\text{ s}$，且关节角速度 $\max|\dot{q}| < 0.15\text{ rad/s}$、机体角速度 $\max|\omega_b| < 0.15\text{ rad/s}$ 连续保持 $0.30\text{ s}$（或满 $3.0\text{ s}$ 上限）才允许进 `S2`；
    3. **时间戳自动断言**：正式 9 次与受阻 3 次中 $t_{\text{standup}} - t_{\text{damping}} \in [0.525\text{ s},\ 3.400\text{ s}]$，恒大于 0。
- **右栏（配图具体文件名）**：
  - 上图：状态转移框图（对应报告图 3-1 Mermaid 状态转移图：`S0 -> S1 -> S2 -> [S2B] -> S3 -> S4 -> S0 / S5`）
  - 下图：`docs/report/figures/fig05_state_timeline.png`（前翻、侧翻、后翻、受阻四类实验的真实状态机状态阶梯时间线）
- **演讲者逐字备注（约 45 秒）**：
  > “第六页展示我们的 7 状态恢复状态机 `RecoveryFSM`。任务书第二项评分（25分）专门考核‘判定后是否先进入安全状态再尝试起身’。我们在状态机拓扑设计上直接把 `S1_SAFE_DAMPING` 设为 `S0_MONITOR` 触发后的唯一出口。不仅如此，机器狗进入 `S1` 纯阻尼后，不仅要至少停留 0.5 秒，还必须等待所有关节角速度和机身角速度都收敛到 0.15 rad/s 以下并持续稳定 0.3 秒，确认翻滚冲击动能彻底耗散后，才允许进入 `S2_ASSESS` 和后续的起身状态。看右下方的实测状态时间线图 `fig05`，每一次实验都清晰呈现出‘先橙色 `S1_SAFE_DAMPING`、后绿色 `S3_STANDUP`’的严格阶梯时序。”

---

### P7：`S2_ASSESS` 四路姿态分流与平台能力边界

- **页面标题**：`S2_ASSESS` 四路官方状态姿态分流机制与平台物理边界
- **核心结论金句（Banner）**：
  > **“只用官方原语（`JointDamping + LieDown + StandUp`），怎么解决侧翻散腿起不来和四脚朝天盲目挣扎？答案是 `S2_ASSESS` 四路智能姿态分流。”**
- **左栏（平台 `StandUp` 能力边界发现与四路分流表）**：
  - **预研核心发现（为什么不能无脑调 `StandUp`？）**：
    - 官方 `StandUp` 是从当前关节角 $q_0$ 插值到蹲姿 `Q_PRE` 再垂直伸腿，**不是自翻正算法**。
    - 若落地后四腿不对称散开或处于侧撑态，直接调用 `StandUp` 会以单侧触地脚为偏心支点产生侧翻力矩，把机身越推越翻（甚至从 $57^\circ$ 推翻成 $89^\circ$）。
  - **`S2_ASSESS` 四路官方原语分流表（`select_strategy`）**：

  | 分流策略 | 几何与关节偏差判据 ($e_q = \max|q - q_{\text{pre}}|$) | 官方原语组合路径 | 典型场景与效果 |
  |---|---|---|---|
  | **1. `direct_standup`** | $|\phi|,|\theta| < 35^\circ$ 且 $e_q < 0.35\text{ rad}$ | `S1(Damping)` $\rightarrow$ **`S3(StandUp)`** | 前翻/回摆后翻：已趴平且四腿对称，直接起身省 $1.8\text{ s}$ |
  | **2. `tuck_then_standup`** | $|\phi|,|\theta| < 35^\circ$ 且 $e_q \ge 0.35\text{ rad}$ | `S1` $\rightarrow$ **`S2B(LieDown 1.8s)`** $\rightarrow$ **`S3(StandUp)`** | 侧翻/受阻重试：先调官方 `LieDown` 收腿对齐至 `Q_PRE`，再起身 100% 成功 |
  | **3. `side_prop_liedown`** | $35^\circ \le \max(|\phi|,|\theta|) < 120^\circ$ | `S1` $\rightarrow$ **`S2B(LieDown 1.8s)`** $\rightarrow$ **`S3(StandUp)`** | 半侧撑未倒平：先调官方 `LieDown` 收腿卸除侧撑支点贴地，再起身 |
  | **4. `unrecoverable_supine`** | $|\phi| \ge 120^\circ$ 或 $|\theta| \ge 80^\circ$ | `S1` $\rightarrow$ **`S5_SAFE_FAULT`** (`E_UNRECOVERABLE_POSE`) | **纯仰翻 $180^\circ$ 物理死区**：0 次盲目尝试，直接安全锁存报警 |

- **右栏（配图具体文件名）**：
  - 上图：`docs/report/sim_shots/grid_overview.png`（全场景对比）
  - 下左图：`docs/assets/side65_settled.png`（未整姿时侧撑约 $57^\circ$ 直接起身的失败机理对照）
  - 下右图：`docs/report/sim_shots/strip_supine_boundary.png`（纯仰翻 $180^\circ$ 边界测试：主动识别并锁存 `E_UNRECOVERABLE_POSE`）
- **演讲者逐字备注（约 55 秒）**：
  > “第七页是我们方案中最具工程巧思的设计——`S2_ASSESS` 四路官方状态姿态分流。我们在前期可行性预研中摸清了平台内置 `StandUp` 的物理底牌：它只能从四肢基本对称的趴地姿态垂直顶起机身。如果侧翻滚落或受阻挣扎后四条腿一侧伸展、一侧弯曲，直接调用 `StandUp` 会以伸展脚为支点把机身顶翻；而如果是四脚朝天的纯仰翻，调 `StandUp` 只会让四条腿在空中空转抽搐。怎么在不编写非官方轨迹的前提下解决这个问题？我们发现官方状态机里除了 `StandUp`，还有一个标准的 `LieDownState`（卧倒状态），它能用 1.5 秒三次样条把任意散开的四条腿平滑收拢到标准预站立角 `Q_PRE` 并阻尼贴地！于是我们在 `S2_ASSESS` 中设计了四路分流：四腿已经对齐的前翻后翻走 `direct_standup` 直接起身；四腿不对称的侧翻走 `tuck_then_standup`，先调官方 `LieDown` 整姿 1.8 秒对齐四腿，再调 `StandUp`；而对于超出平台恢复边界的 180 度纯仰翻，如右下角 `strip_supine_boundary.png` 所示，状态机直接识别为 `unrecoverable_supine`，0 次盲目挣扎，立即锁存安全阻尼并上报 `E_UNRECOVERABLE_POSE`。”

---

### P8：仿真试验台：官方 MJCF + 官方 StandUp/LieDown 轨迹 + 官方 ONNX 策略

- **页面标题**：高保真 MuJoCo 仿真试验台与规范化姿态注入
- **核心结论金句（Banner）**：
  > **“100% 源自云深处官方开源资产：官方 BSD-3 MJCF 模型 + 官方两阶段样条/IK 公式复刻 + 官方 `policy.onnx` 策略 + 5 mm 精确几何间隙无穿透注入。”**
- **左栏（四大仿真保真度支柱）**：
  - **1. 官方物理模型（`sim/models/lite3/Lite3.xml`）**：
    - DeepRobotics 官方 MJCF（BSD-3 许可），12 自由度关节电机，严格遵循官方 **$\pm 30.0\text{ N\cdot m}$** 扭矩限幅，$1\text{ ms}$ 物理步长（$1\text{ kHz}$ PD 控制）。
  - **2. 官方 `StandUpState` 两阶段三次样条 + 二连杆 IK 复刻**：
    - 严格取自 `lite3_control_parameters.cpp`：$L_1=0.20\text{ m}, L_2=0.21\text{ m}, k_p=100, k_d=2.5$，每阶段 $T=1.5\text{ s}$（总 $3.0\text{ s}$）。
    - **Phase 1 ($0\text{–}1.5\text{ s}$)**：从当前角 $\mathbf{q}_0$ 三次样条插值至预站立角 $\mathbf{q}_{\text{pre}} = [0, -1.35453, 2.54948] \times 4$（对应 $h_{\text{pre}}=0.12\text{ m}$）。
    - **Phase 2 ($1.5\text{–}3.0\text{ s}$)**：高度从 $0.12\text{ m}$ 三次样条插值至 $h_{\text{stand}}=0.33\text{ m}$，由余弦定理实时解算 $\text{HipY} \to -0.6535\text{ rad}, \text{Knee} \to 1.2711\text{ rad}$。
  - **3. 官方 `LieDownState` 与 `policy.onnx` 集成**：
    - `LieDown`：$1.5\text{ s}$ 三次样条收腿至 $\mathbf{q}_{\text{pre}} + 0.3\text{ s}$ 纯阻尼贴地。
    - `policy.onnx`：官方 45 维观测 PPO 策略，$83.3\text{ Hz}$（1/12 分频）驱动正常小跑。
  - **4. `mj_geomDistance` 精确 5 mm 无穿透姿态注入**：
    - 遍历 `TORSO` 子树全部碰撞体与地面精确距离，将最低点置于离地 `5.0 mm` 处，彻底杜绝初始网格穿透弹飞或高空坠落失真。
- **右栏（配图与轨迹曲线）**：
  - 上图：`docs/assets/sim_smoke.png`（标准站立稳态测试：目标高度 $0.33\text{ m}$，实际稳态高度 $0.346\text{ m}$，姿态漂移 $<0.5^\circ$）
  - 下图：`docs/report/figures/fig06_standup.png`（Phase 1 蹲姿 $1.5\text{ s}$ + Phase 2 顶升 $1.5\text{ s}$ + `S4_VERIFY` $1.0\text{ s}$ 的实测高度与俯仰角曲线）
- **演讲者逐字备注（约 40 秒）**：
  > “第八页介绍我们的 MuJoCo 仿真试验台。为了确保仿真结果对实机有直接指导意义，我们没有使用简化的火柴人模型或随意编写的线性插值起身，而是全套移植了云深处官方开源资产：模型是官方 BSD-3 授权的 Lite3 MJCF，扭矩限幅严格锁在正负 30 牛米；站立动作完整复刻了官方 `StandUpState` 的两阶段三次样条加平面二连杆逆运动学公式，前 1.5 秒收腿到 0.12 米预站立角，后 1.5 秒按余弦定理垂直顶升到 0.33 米；此外，我们用 MuJoCo 的 `mj_geomDistance` 精确计算机器人全身网格到地面的最短距离，以 5 毫米间隙精确放置翻倒初态，保证了全部 9+3 次实验物理接触的真实与一致。”

---

### P9：正式 9 次测试结果（一）：前翻 & 后翻恢复全流程

- **页面标题**：正式测试结果（一）：前翻（3/3）与后翻（3/3）恢复全流程
- **核心结论金句（Banner）**：
  > **“前翻 +90° 经 75° 快速通道在 0.155 s 极速检出；后翻 -90°（带前向回摆）经 50° 主通道在 0.305 s 检出——均走 `direct_standup` 在 4.21 s 内一次平稳起身。”**
- **左栏（前翻与后翻实验数据与时序拆解）**：
  - **1. 前翻实验（`front_1..3`，共 3 次，100% 一致）**：
    - **初态注入**：`pitch = +90°`，四腿 `Q_PRE`，离地 $5\text{ mm}$ 零速释放。
    - **关键时间轴**：
      - $t = 0.155\text{ s}$：`fast_path`（$>75^\circ$ 持续 $0.15\text{ s}$）确认摔倒，立即切入 `S1_SAFE_DAMPING`；
      - $t = 2.135\text{ s}$：机身向前落稳成水平趴地（$h=0.054\text{ m}$），角速度静置达标，进入 `S2_ASSESS`；
      - $t = 2.140\text{ s}$：评估四腿偏差 $e_q < 0.35\text{ rad}$，选定 `direct_standup` 进入 `S3_STANDUP`（严格满足 $t_{\text{damp}}=0.155 < t_{\text{su}}=2.140$）；
      - $t = 5.340\text{ s}$：$3.20\text{ s}$ 站立轨迹完成（$h=0.346\text{ m}, |\theta|<0.5^\circ$），进入 `S4_VERIFY`；
      - $t = 6.345\text{ s}$：连续 $1.005\text{ s}$ 直立验证通过，回到 `S0_MONITOR`（**起身耗时 $4.21\text{ s}$**）。
  - **2. 后翻实验（`back_1..3`，共 3 次，100% 一致）**：
    - **初态注入**：`pitch = -90°` + 前向回摆角速度 $\omega_y = 4.0\text{ rad/s}$（模拟爬坡/急停后仰失稳后的回摆落回趴地）。
    - **关键时间轴**：
      - $t = 0.305\text{ s}$：`attitude` 主通道确认摔倒，切入 `S1_SAFE_DAMPING`；
      - $t = 1.670\text{ s}$：阻尼下平息静止，进入 `S2_ASSESS`；
      - $t = 1.675\text{ s}$：触发 `direct_standup` 进入 `S3_STANDUP`；
      - $t = 4.875\text{ s} \to 5.880\text{ s}$：验证通过恢复 `S0_MONITOR`（**起身耗时 $4.21\text{ s}$**）。
- **右栏（配图与演示视频具体文件名）**：
  - 上图：`docs/report/sim_shots/strip_front.png`（前翻 4 帧序列：注入 $\rightarrow$ 阻尼 $\rightarrow$ 起身 $\rightarrow$ 恢复）
  - 下图：`docs/report/sim_shots/strip_back.png`（后翻 4 帧序列：注入 $\rightarrow$ 阻尼 $\rightarrow$ 起身 $\rightarrow$ 恢复）
  - 嵌入视频 1：`videos/demo_mjcf_front.mp4`（未剪辑前翻恢复全流程视频）
  - 嵌入视频 2：`videos/demo_mjcf_back.mp4`（未剪辑后翻恢复全流程视频）
- **演讲者逐字备注（约 45 秒）**：
  > “接下来汇报正式 9 次测试的具体表现。第九页展示前翻 3 次和后翻 3 次实验，右侧上方和下方分别是它们的四帧关键状态截图和未剪辑演示视频。在前翻测试中，机器狗初始俯仰角为正 90 度，检测器的 75 度快速通道在 0.155 秒就极速确认摔倒并切入 `S1` 阻尼；到 2.135 秒机身彻底落稳趴平后，`S2_ASSESS` 判断四腿对称，直接选择 `direct_standup`，在 2.140 秒启动 `StandUp`，5.340 秒站起至 0.346 米，再经过 1 秒 `S4_VERIFY` 稳定验证，于 6.345 秒回到监控态，起身耗时 4.21 秒。后翻 3 次同样在 0.305 秒由主姿态通道检出、1.675 秒启动起身、5.880 秒完成恢复，3 次前翻与 3 次后翻全部一次尝试 100% 成功。”

---

### P10：正式 9 次测试结果（二）：侧翻收腿整姿 + 起身全流程

- **页面标题**：正式测试结果（二）：侧翻（3/3）“阻尼回落 $\rightarrow$ 官方 `LieDown` 整姿 $\rightarrow$ 官方 `StandUp` 起身”
- **核心结论金句（Banner）**：
  > **“侧翻落地后左右腿受挤压不对称（$e_q \ge 0.35\text{ rad}$），`S2_ASSESS` 自动分流至 `tuck_then_standup`：先调官方 `LieDown` 收腿对齐至 `Q_PRE`，再无缝调用 `StandUp`，3/3 一次成功！”**
- **左栏（侧翻 3 次实验全链路时间戳拆解：`side_1..3`）**：
  - **测试覆盖**：`side_1`（左侧翻 $+65^\circ$）、`side_2`（右侧翻 $-65^\circ$）、`side_3`（左侧翻 $+65^\circ$）。
  - **完整状态转移时间戳链条（以 `2026-11-19_side_1` 为例）**：
    1. **$t = 0.000\text{ s}$（姿态注入）**：横滚角 $\phi = +65.0^\circ$，离地 $5\text{ mm}$ 释放。
    2. **$t = 0.305\text{ s}$（`S0 -> S1_SAFE_DAMPING`）**：`attitude` 主通道（$|\tilde{\phi}| > 50^\circ$ 满 $0.30\text{ s}$）确认摔倒，历史峰值分类为 `side_left`，立即切纯阻尼。
    3. **$t = 0.305 \to 1.895\text{ s}$（阻尼卸力与翻回趴地）**：在纯阻尼与重力矩作用下，机体平顺翻回水平趴姿（$|\phi| < 35^\circ$），但受地面侧向摩擦挤压，左右腿关节角偏离 `Q_PRE` 超过 $0.35\text{ rad}$。
    4. **$t = 1.900\text{ s}$（`S2_ASSESS -> S2B_PREPARE_LIEDOWN`）**：`select_strategy()` 选中 `tuck_then_standup`，触发 `backend.start_liedown(1.900)`，调用官方 `LieDown`（$1.5\text{ s}$ 三次样条收腿对齐至 `Q_PRE` + $0.3\text{ s}$ 阻尼贴地）。
    5. **$t = 3.705\text{ s}$（`S2B -> S3_STANDUP`）**：四腿完美对称贴地，无缝调用 `backend.start_standup(3.705)`（满足 $t_{\text{damp}}=0.305 < t_{\text{su}}=3.705$）。
    6. **$t = 6.905 \to 7.910\text{ s}$（`S4_VERIFY -> S0_MONITOR`）**：$3.20\text{ s}$ 起身完成 + $1.005\text{ s}$ 验证通过，**起身阶段耗时恒为 $4.21\text{ s}$**，尝试次数 $1$ 次。
- **右栏（配图与演示视频具体文件名）**：
  - 上图：`docs/report/sim_shots/strip_side_left.png`（左侧翻 4 帧序列：侧倾 $65^\circ$ 注入 $\rightarrow$ `S1` 阻尼 $\rightarrow$ `S2B` 官方 `LieDown` 整姿 $\rightarrow$ `S0` 恢复直立）
  - 下图：`docs/report/figures/fig06_standup.png`（起身阶段高度从 $0.054 \to 0.12 \to 0.346\text{ m}$ 的平滑轨迹）
  - 嵌入视频：`videos/demo_mjcf_side.mp4`（未剪辑侧翻整姿+起身全流程视频）
- **演讲者逐字备注（约 45 秒）**：
  > “第十页展示侧翻 3 次实验（包含两次左侧翻、一次右侧翻），这也是最能体现 `S2_ASSESS` 姿态分流价值的一页。请看右上方的四帧序列图和视频 `demo_mjcf_side.mp4`：当机器狗以 65 度侧倾注入后，0.305 秒检出摔倒并切入 `S1` 纯阻尼；在阻尼和重力作用下，机身虽然在 1.895 秒翻回了趴地姿态，但落地侧的腿被挤压偏离了标准预站立角。此时 `S2_ASSESS` 自动走 `tuck_then_standup` 分支，在 1.900 秒先进入 `S2B_PREPARE_LIEDOWN`，调用官方 `LieDown` 花 1.8 秒把四条腿整齐收拢到 `Q_PRE`，然后在 3.705 秒无缝启动 `StandUp`，机身笔直平稳地升起到 0.346 米，左右侧翻 3 次全部一次成功！”

---

### P11：正式 9 次汇总表与时延分布

- **页面标题**：正式 9 次测试全景汇总表与时延/耗时统计分布
- **核心结论金句（Banner）**：
  > **“9/9 检出（100%）、9/9 分类准确（100%）、9/9 先阻尼后起身（100%）、9/9 一次起身成功（100%）——判定时延均值 0.255 s，起身耗时均值 4.21 s。”**
- **左栏（正式 9 次实验完整数据汇总表 —— 直接源自 `matrix_2026-11-19.csv`）**：

  | # | `run_id` | 翻倒类型 | 触发通道 | 判定时延 | $t_{\text{damp}} < t_{\text{su}}$ | 分流策略 | 尝试 | 起身耗时 | 最终状态 |
  |---|---|---|---|---|---|---|---|---|---|
  | 1 | `front_1` | `front` | `fast_path` | **0.155 s** | ✅ (`0.155 < 2.140`) | `direct_standup` | 1 | **4.21 s** | `S0_MONITOR` |
  | 2 | `front_2` | `front` | `fast_path` | **0.155 s** | ✅ (`0.155 < 2.140`) | `direct_standup` | 1 | **4.21 s** | `S0_MONITOR` |
  | 3 | `front_3` | `front` | `fast_path` | **0.155 s** | ✅ (`0.155 < 2.140`) | `direct_standup` | 1 | **4.21 s** | `S0_MONITOR` |
  | 4 | `side_1` | `side_left` | `attitude` | **0.305 s** | ✅ (`0.305 < 3.705`) | `tuck_then_standup` | 1 | **4.21 s** | `S0_MONITOR` |
  | 5 | `side_2` | `side_right`| `attitude` | **0.305 s** | ✅ (`0.305 < 3.705`) | `tuck_then_standup` | 1 | **4.21 s** | `S0_MONITOR` |
  | 6 | `side_3` | `side_left` | `attitude` | **0.305 s** | ✅ (`0.305 < 3.705`) | `tuck_then_standup` | 1 | **4.21 s** | `S0_MONITOR` |
  | 7 | `back_1` | `back` | `attitude` | **0.305 s** | ✅ (`0.305 < 1.675`) | `direct_standup` | 1 | **4.21 s** | `S0_MONITOR` |
  | 8 | `back_2` | `back` | `attitude` | **0.305 s** | ✅ (`0.305 < 1.675`) | `direct_standup` | 1 | **4.21 s** | `S0_MONITOR` |
  | 9 | `back_3` | `back` | `attitude` | **0.305 s** | ✅ (`0.305 < 1.675`) | `direct_standup` | 1 | **4.21 s** | `S0_MONITOR` |
  | **汇总** | **9 次正式** | **3×3 全覆盖** | **3 快 / 6 主** | **均值 0.255 s** | **9/9 (100%)** | **6 直接 / 3 整姿** | **全 1** | **均值 4.21 s** | **9/9 恢复** |

- **右栏（配图具体文件名）**：
  - 上图：`docs/report/figures/fig01_detection_latency.png`（9 次判定时延柱状图 vs $2.0\text{ s}$ 优秀档红线）
  - 下图：`docs/report/figures/fig02_attitude_timeline.png`（三类翻倒全过程姿态角收敛与状态色块对照）
- **演讲者逐字备注（约 35 秒）**：
  > “第十一页汇总了正式 9 次测试的全部原始指标。左表可以直接与我们提交的 `matrix_2026-11-19.csv` 逐行核对：9 次实验全部一次尝试恢复成功，判定时延分布在 0.155 秒到 0.305 秒之间，均值 0.255 秒；从下发 `StandUp` 指令到通过 1 秒直立验证的起身总耗时全部稳定在 4.21 秒（其中 3.20 秒为两阶段轨迹执行，1.01 秒为 `S4_VERIFY` 站稳验证），远优于任务书优秀档 15 秒的要求。”

---

### P12：失败与超时处理：起身受阻 3 次实验（最多 2 次重试 + `S5_SAFE_FAULT` 锁存）

- **页面标题**：失败与超时处理：起身受阻 3 次实验（限次退避重试 + 故障锁存）
- **核心结论金句（Banner）**：
  > **“躯干上方 0.16 m 矮横梁压迫下：高度通道 0.320 s 破盲区检出 → 第 1 次起身受阻 → 退避 2.0 s 并自动升级 `LieDown` 整姿 → 第 2 次受阻后于 12.465 s 永久锁存 `S5_SAFE_FAULT`！”**
- **左栏（受阻实验 `blocked_1..3` 八步精确时间链与安全机制）**：
  - **场景构造**：机器人水平趴地（$\phi \approx 0^\circ, \theta \approx 0^\circ, h=0.054\text{ m}$），正上方架设 $0.16\text{ m}$ 刚性矮横梁（模拟跌入设备底座下方）。
  - **受阻 3 次实验（`2026-11-19_blocked_1..3`）完整状态时间链**：
    1. **$t = 0.320\text{ s}$（高度兜底检出）**：姿态角为 $0^\circ$，由 **`height` 通道（$h < 0.10\text{ m}$ 满 $0.30\text{ s}$）**精准检出，切入 `S1_SAFE_DAMPING`；
    2. **$t = 0.845\text{ s}$（第 1 次尝试 `attempt_1`）**：`S2_ASSESS` 选定 `direct_standup` 进入 `S3_STANDUP`；
    3. **$t = 4.050\text{ s}$（第 1 次受阻判定）**：跑完 $3.20\text{ s}$ 轨迹后背部被横梁死死压在 $h \le 0.10\text{ m} < 0.25\text{ m}$，立即判定未直立，**切回 `S1_SAFE_DAMPING` 卸力**（无需傻等 20 s 烧电机）；
    4. **$t = 5.455 \to 7.455\text{ s}$（2.0 s 退避冷却 + 策略自适应升级）**：第 1 次挣扎后腿部滑开（$e_q \ge 0.35\text{ rad}$），在 `S2_ASSESS` 严格等待 **`backoff_s = 2.0 s`** 后，第 2 次策略自动升级为 `tuck_then_standup`；
    5. **$t = 7.455 \to 9.260\text{ s}$（第 2 次尝试前先整姿）**：调用官方 `LieDown` 收拢滑开的四腿；
    6. **$t = 9.260 \to 12.465\text{ s}$（第 2 次起身尝试 `attempt_2`）**：再次调用 `StandUp`，仍被横梁阻挡；
    7. **$t = 12.465\text{ s}$（永久故障锁存）**：`attempts == 2 == max_attempts`，立即转入 **`S5_SAFE_FAULT`**，锁定纯阻尼 `Mode.DAMPING`，输出故障码 **`E_STANDUP_NOT_UPRIGHT`**，后续不再重试，仅响应 `manual_reset()`。
- **右栏（配图与演示视频具体文件名）**：
  - 上图：`docs/report/sim_shots/strip_blocked.png`（受阻实验 4 帧序列：$0.32\text{ s}$ 高度检出 $\rightarrow$ 第 1 次受阻 $\rightarrow$ 退避+整姿+第 2 次受阻 $\rightarrow$ $12.465\text{ s}$ 锁存 `S5_SAFE_FAULT`）
  - 下表：受阻 3 次实验汇总表（`blocked_1..3`：3/3 检出通道 `height`、时延 `0.320 s`、尝试次数 `2, 2, 2`、最终状态 `S5_SAFE_FAULT`、故障码 `E_STANDUP_NOT_UPRIGHT`）
  - 嵌入视频：`videos/demo_mjcf_blocked.mp4`（未剪辑受阻 2 次重试与故障锁存全过程视频）
- **演讲者逐字备注（约 55 秒）**：
  > “第十二页展示任务书第四项评分（15分）‘失败与超时处理’的 3 次起身受阻实验。我们在机器狗正上方放置了一根低矮横梁，模拟机器狗钻进桌底或设备底座下方被卡住的场景。请看右侧的 `strip_blocked.png` 四帧截图和视频 `demo_mjcf_blocked.mp4`：这个实验一举验证了我们系统的三项核心能力！第一，初始趴地时 roll 和 pitch 都是 0 度，高度兜底通道在 0.320 秒立功检出；第二，在 0.845 秒发起第 1 次起身被横梁挡住后，系统在 4.050 秒轨迹结束时立即发现高度不足并切回 `S1` 阻尼卸力，避免电机长时间堵转；更聪明的是，由于第一次顶横梁时脚底打滑散开，系统在执行完 2.0 秒退避等待后，第 2 次尝试自动升级为 `tuck_then_standup`，在 7.455 秒先调 `LieDown` 把滑开的腿收拢对齐，再试第 2 次起身；第三，当 12.465 秒第 2 次尝试依然被挡住时，尝试次数达到上限 2 次，状态机立即锁存进 `S5_SAFE_FAULT` 纯阻尼状态，打印并发布故障码 `E_STANDUP_NOT_UPRIGHT`。之后无论过多久都绝不发起第 3 次重试，直到人工移开障碍物并调用 `manual_reset()` 才会解锁。”

---

### P13：从仿真到实机：`Ros2Backend` 接口映射与实机保护测试 SOP

- **页面标题**：从仿真到实机：`Ros2Backend` 话题映射、差异补偿与安全保护 SOP
- **核心结论金句（Banner）**：
  > **“决策层代码零修改迁移：通过 FK+IMU 投影替代仿真高度真值，严格执行‘软垫 + 龙门吊架（10 cm 余量）+ 第三人急停监护’的四阶段受控测试 SOP。”**
- **左栏（`Ros2Backend` 话题映射与 Sim-to-Real 四大补偿对策）**：
  - **1. 官方 ROS2 / SDK 接口精确映射**：
    - 感知输入：订阅 `/IMU_DATA`（`drdds/msg/ImuData`）+ `/JOINTS_DATA`（`drdds/msg/JointsData`），$200\text{ Hz}$ 组装 `Observation`。
    - 动作输出：触发官方状态机 `kJointDamping / kLieDown / kStandUp` + 独立安全链 `/EMERGENCY_STOP_SIGNAL`。
    - 状态与复位：发布 `/doggie/status`（状态、故障码、尝试次数），提供 `/doggie/reset` 人工复位服务。
  - **2. 仿真与实机差异（Sim-to-Real Gap）工程补偿**：
    - **无高度真值替代**：实机无激光雷达/动捕，由 12 关节角正向运动学（FK）结合 IMU 姿态旋转矩阵 $\mathbf{R}_{wb}(\phi,\theta)$ 实时解算支撑足到躯干的垂直高度 $\hat{h}$（趴地折叠时同样 $<0.10\text{ m}$）。
    - **IMU 静态零偏校准**：上电直立静置采集 $2.0\text{ s}$（400 帧）均值扣除安装零偏；$50^\circ$ 阈值留有 $>45^\circ$ 裕度。
    - **通信断联看门狗**：心跳超时 `0.50 s` 或阻尼指令 `0.50 s` 无 ACK $\rightarrow$ 立即报 `E_DAMPING_NO_ACK` 并触发硬急停。
- **右栏（实机四阶段受控保护测试 SOP 与三人安全责任制）**：
  - **强制安全配置**：`config/recovery.yaml` 中默认锁定 `allow_real_fall_test: false`（严禁实机无保护自由摔倒）。
  - **三人安全责任制**：**主控员**（监控终端/复位） + **吊架保护员**（握持吊带托扶） + **急停安全员**（专职手持硬件急停）。
  - **四阶段渐进式实机 SOP**：
    1. **Stage 1（干跑校准）**：吊架悬空/软垫站立，人工缓推侧倾 $30^\circ / 55^\circ / 80^\circ$，只看 `/doggie/status` 检出日志，不下发动作；
    2. **Stage 2（只检不立）**：人工缓慢推至侧倾 $65^\circ$ 放于 $5\text{ cm}$ EVA 软垫，验证 $0.31\text{ s}$ 自动切入 `JointDamping` 腿部变软卸力；
    3. **Stage 3（吊架辅助整姿起身）**：吊带留 $10\text{ cm}$ 余量，从软垫侧翻趴姿自动执行 `S1(阻尼) -> S2B(LieDown收腿) -> S3(StandUp起身) -> S4(1s验证)`；
    4. **Stage 4（受阻锁存验证）**：用软垫轻压狗背限制起升高度，验证 2 次尝试后自动放弃并锁存 `S5_SAFE_FAULT`，调用 `/doggie/reset` 复位。
- **演讲者逐字备注（约 45 秒）**：
  > “第十三页汇报实机迁移方案与安全保护 SOP。得益于 `RobotBackend` 抽象层，实机部署时 `FallDetector` 和 `RecoveryFSM` 不需要改动一行代码，只需切换到对接官方 `/IMU_DATA` 和 `/JOINTS_DATA` 的 `Ros2Backend`。针对实机没有仿真高度真值 `qpos[2]` 的问题，我们采用腿部正向运动学 FK 结合 IMU 旋转矩阵来估算躯干离地高度。在实机测试安全方面，我们严格遵守任务书‘实机须在保护条件下进行受控测试’的规定，制定了三人安全责任制与四阶段渐进 SOP：全程在 5 厘米厚缓冲软垫和龙门保护吊架下进行，吊带留 10 厘米余量，按照‘先干跑测阈值、再测切阻尼、最后在吊架保护下测整姿起身与受阻锁存’的顺序安全推进。”

---

### P14：总结与答辩高频 Q&A 备询

- **页面标题**：总结与答辩高频问题（Q&A）备询
- **核心结论金句（Banner）**：
  > **“用可解释的经典控制与状态机工程方法，把平台已有的 `JointDamping / LieDown / StandUp` 组合出 100% 检出、100% 顺序正确、100% 可恢复包络内一次起身、100% 受阻安全锁存的满分表现。”**
- **左栏（项目三大核心贡献总结）**：
  1. **三通道迟滞摔倒检测器（`FallDetector`）**：
     - `10 Hz` 低通滤波 + 主姿态（$50^\circ/0.30\text{ s}$）+ 快速通道（$75^\circ/0.15\text{ s}$）+ 高度兜底（$0.10\text{ m}/0.30\text{ s}$）+ $35^\circ/0.50\text{ s}$ 回差，实现 **9/9 检出（0.155–0.305 s）**与 **180 s 小跑 0 误报**。
  2. **强制安全序与四路官方原语分流状态机（`RecoveryFSM`）**：
     - 拓扑级强制 `S1_SAFE_DAMPING` 先行（**9/9 满足 `t_damping < t_standup`**）；在 `S2_ASSESS` 独创四路分流，巧妙组合官方 `LieDown` 与 `StandUp` 实现 **9/9 一次起身成功（4.21 s）**，并对纯仰翻 $180^\circ$ 主动安全拒止。
  3. **严密异常闭环与扎实定量标定**：
     - 受阻 **3/3 在 2 次尝试（含 2.0 s 退避与策略自动升级）后锁存 `S5_SAFE_FAULT`**；**56 组二维网格标定**给出每个参数的定量边界。
- **右栏（答辩评委高频 Q&A 备询卡片 —— 4 大必考题）**：
  - **Q1：为什么不自己训练一个强化学习（RL）端到端起身策略？**
    - **A**：任务书明确界定任务二为“状态机触发与复用平台已有起身动作”。RL 黑盒起身在实机受阻或传感器异常时力矩不可控、极易砸坏硬件；而复用经过厂家量产验证的 `JointDamping / LieDown / StandUp` 原语，配合我们可解释的白盒监督状态机，安全边界清晰、参数可标定、故障可锁存。
  - **Q2：侧翻为什么不能直接调 `StandUp`，而要先调 `LieDown`？**
    - **A**：官方 `StandUp` 第一阶段是各关节独立插值到 `Q_PRE` 并紧接着推地。侧翻滚落或第 1 次受阻后四腿左右不对称（$e_q \ge 0.35\text{ rad}$），直接推地会以单侧脚为支点产生侧翻力矩把机身重新推翻。先调官方 `LieDown` 用 $1.5\text{ s}$ 收腿至 `Q_PRE` 并阻尼贴地 $0.3\text{ s}$，消除了偏心支点，起身成功率从不足 50% 提升至 100%。
  - **Q3：当机器狗水平趴在地上或被矮架压住时，`roll≈0°, pitch≈0°`，怎么检出摔倒？又怎么区分正常 `LieDown` 卧倒？**
    - **A**：靠第三条通道——躯干高度兜底判据（`h < 0.10 m` 持续 `0.30 s`）。腹部贴地时高度约 `0.054 m`，而官方 `LieDown` 预站立蹲姿高度为 `0.12 m`，`0.10 m` 门限正好卡在两者中间（留有 `2 cm` 裕量），既能 100% 检出坍塌趴地（如 `blocked_1..3` 在 `0.320 s` 检出），又不会误报正常卧倒。
  - **Q4：如果机器狗完全四脚朝天（纯仰翻 180°），你们的系统会怎么办？**
    - **A**：我们在可行性预研中通过实验证明（即使把电机扭矩从 $30\text{ N\cdot m}$ 提至 $65\text{ N\cdot m}$），官方对称伸腿的 `StandUp` 在几何上无法从 $180^\circ$ 纯仰翻自翻正。因此我们的 `S2_ASSESS` 专门设计了第 4 路分支 `unrecoverable_supine`（$|\phi|\ge 120^\circ$ 或 $|\theta|\ge 80^\circ$）：一旦静止后发现四脚朝天，**0 次盲目起身尝试**，直接锁存至 `S5_SAFE_FAULT` 纯阻尼并报警 `E_UNRECOVERABLE_POSE`，等待人工翻正，杜绝无效挣扎伤机。
- **演讲者逐字备注（约 30 秒）**：
  > “最后一页是我们的总结与高频问题备询。回顾整个项目，我们用严谨的白盒控制与状态机工程方法，把平台内置的阻尼、卧倒、起身三个标准原语发挥到了极致，在五项考核指标上全部交出了满分答卷。右侧我们也准备了关于‘为什么不自己训 RL’、‘侧翻为什么要先调 LieDown’、‘水平趴地零姿态角怎么检出’以及‘四脚朝天怎么处理’四个核心问题的定量解答。我的汇报到此结束，恳请各位评委老师批评指正！”

---

## 第二部分：Windows 下直接可用的 AI 生成提示词（Prompts）

> **使用说明**：当您在 Windows 电脑上制作答辩 PPT（使用 Gamma、Kimi PPT 助手、WPS AI、Microsoft Designer / Copilot）或排版 Word 报告时，只需**整段复制**以下对应的 Prompt 粘贴给 AI，再按文末《Windows 本地素材插入对照表》将生成的占位图替换为仓库内的真实高清图表与 MP4 视频即可。

---

### Prompt A：用于 Gamma / Kimi PPT / WPS AI / Copilot 一键生成 14 页 PPT 的完整结构化提示词

```markdown
你是一位顶级的机器人工程与学术答辩 PPT 设计师。请根据以下完整的大纲与精确实验数据，为我生成一份共 14 页的专业工程答辩 PPT：《四足机器人 Lite3 摔倒检测与自主恢复运控系统（任务二）》。

【全局视觉与版式要求】
1. 风格定位：硬核机器人工程风 / 高管技术汇报风（清晰、克制、数据驱动、结论先行）。
2. 配色方案：深海蓝主色（#0F2C59）+ 科技青强调色（#00A8E8）+ 安全绿达标色（#16A34A）+ 警示橙/红故障色（#EA580C / #DC2626），浅灰白背景（#F8FAFC），深灰正文（#1E293B）。
3. 页面布局：每页必须采用「顶部标题 + 一句话核心结论横幅（Banner） + 左右双栏（左栏结构化要点/数据表格，右栏图表/视频占位框）」的标准宽屏（16:9）版式。
4. 数据保真度：严禁编造或修改任何数字！必须严格使用我给出的实测数据（如：9/9 检出、0.155–0.305 s 时延、0/180 s 误报、4.21 s 起身耗时、50°/0.30 s 阈值、35°/0.50 s 回差、75°/0.15 s 快速通道、0.10 m/0.30 s 高度兜底、2 次重试锁存 S5_SAFE_FAULT 等）。
5. 图表占位符：在每页右栏明确标注【替换图片：xxx.png】或【插入视频：xxx.mp4】，方便我生成后一键替换本地文件。

【逐页详细内容大纲（共 14 页）】

---
第 1 页：封面与一句话方案
- 主标题：四足机器人 Lite3 摔倒检测与自主恢复运控系统（任务二）
- 副标题：基于三通道迟滞判据与官方原语四路姿态分流的高可靠恢复状态机（Doggie Recovery Supervisor）
- 顶部金句：不重新发明底层步态，而是构建 100% 严守“先安全阻尼、后整姿起身”的智能决策层——正式 9 次翻倒 100% 检出并恢复（平均起身 4.21 s），3 次受阻 100% 限次锁存，180 s 小跑 0 误报。
- 左栏要点：
  - 平台：云深处绝影 Lite3（无激光雷达版） / MuJoCo 4.0 官方 MJCF 物理试验台
  - 配置基线：config/recovery.yaml (SHA-256: 0f4ace0f9e2d) | Git Commit: 615d7b0 + 工作区
  - 完整交付物：核心纯逻辑代码 + 9+3 实测时间序列 + 56 组网格标定数据 + 4 段未剪辑 MP4 视频
- 右栏视觉：【替换图片：docs/report/sim_shots/grid_overview.png】

---
第 2 页：任务拆解与评分标准对齐（5 项全优秀档对照）
- 标题：任务拆解与考核指标全面对齐（任务书表 7 优秀档逐项核验）
- 顶部金句：五项考核指标全部达到并大幅优于任务书表 7 优秀档（满分档）要求。
- 左栏要点：
  - 核心问题 1（感知）：什么迹象判定摔倒？（兼顾大倾角快检、小跑零误报、趴地零倾角兜底）
  - 核心问题 2（决策）：判定后按什么顺序与条件切换状态？（强制先阻尼后起身、四路分流、受阻限次锁存）
  - 工程边界：直接复用平台 JointDamping / LieDown / StandUp / RLControl，自研上层判定与监督状态机。
- 右栏表格（5 项评分对照表）：
  1. 摔倒检测 (25分) | 优秀档：9/9，0误报，≤2s | 实测：9/9检出，时延 0.155–0.305s (均值0.255s)，小跑 0/180s 误报 | 结论：优秀
  2. 恢复流程 (25分) | 优秀档：9/9先安全后起身 | 实测：9/9 满足 t_damping < t_standup，含 S2_ASSESS 四路分流 | 结论：优秀
  3. 起身完成 (25分) | 优秀档：9/9成功，平均≤15s | 实测：9/9 一次成功，平均 4.21s (3.20s轨迹+1.00s验证) | 结论：优秀
  4. 失败与超时 (15分) | 优秀档：3/3停安全态，无无限重试 | 实测：3/3 在 2 次尝试后锁存 S5_SAFE_FAULT (E_STANDUP_NOT_UPRIGHT) | 结论：优秀
  5. 参数依据 (10分) | 优秀档：分布、回差与驻留标定 | 实测：正常包络 ≤1.71° (裕度48.3°)，56组二维网格扫描热力图 | 结论：优秀

---
第 3 页：总体架构：决策与执行解耦（FallDetector + RecoveryFSM + RobotBackend）
- 标题：系统总体架构：纯逻辑决策层与底层执行器解耦
- 顶部金句：核心算法不依赖一行 mujoco 或 rclpy 代码，通过 RobotBackend 抽象接口实现仿真、回放与实机无缝切换。
- 左栏要点：
  - Observation 标准化观测 (200 Hz)：t, roll, pitch, yaw, omega(3), joint_pos(12), joint_vel(12), base_height, feet_contact
  - 纯逻辑核心 (src/doggie/core/)：FallDetector (输出 FallEvent) + RecoveryFSM (输出 Mode 枚举)
  - 双后端实现 (src/doggie/io/)：SimBackend (MuJoCo 4.0 + 官方 MJCF/ONNX) 与 Ros2Backend (/IMU_DATA, /JOINTS_DATA)
- 右栏视觉：绘制分层架构框图 + 【替换图片：docs/assets/sim_smoke.png】

---
第 4 页：摔倒检测算法：滤波、多判据融合、驻留与回差
- 标题：摔倒判定算法设计（FallDetector）：双层三通道并联融合
- 顶部金句：75° 快速通道保极速响应 (0.155 s)，50° 主通道抗冲击防误报 (0.305 s)，0.10 m 高度通道破除水平趴地零姿态角盲区。
- 左栏要点：
  - 信号滤波：200 Hz 采样 + 10 Hz 一阶低通滤波 (α ≈ 0.2696，时间常数 15.9 ms)
  - 通道 c (快速通道 fast_path)：max(|roll|, |pitch|) > 75° 持续 0.15 s
  - 通道 a (主姿态通道 attitude)：|roll| > 50° 或 |pitch| > 50° 持续 0.30 s
  - 通道 b (高度兜底 height)：base_height < 0.10 m 持续 0.30 s（解决趴地 roll≈0, pitch≈0 盲区，且低于官方 LieDown 高度 0.12 m）
  - 锁存与回差：触发后锁存；|roll|<35° 且 |pitch|<35° 持续 0.50 s + FSM 验证通过后解除，重触发次数 = 0
  - 历史峰值五分类：追踪全过程峰值按 60° 门限分类 front / back / side_left / side_right / flat (12/12 准确率 100%)
- 右栏对比表与图片：对比官方 PostureUnsafeCheck() (30°/45°瞬时无驻留无高度兜底) + 【替换图片：docs/report/figures/fig02_attitude_timeline.png】

---
第 5 页：为什么选 50° / 0.3s？—— 正常包络 vs 翻倒分布与网格标定
- 标题：参数定标依据：3×60 s 正常小跑包络与 56 组网格扫描热力图
- 顶部金句：正常小跑最大姿态角仅 1.71°（距 50° 阈值裕度 48.29°）；56 组网格扫描证明 (50°, 0.30 s) 处于低时延不退化最优区。
- 左栏要点：
  - 3×60 s 官方 ONNX 小跑实测 (normal_turn / vx0.4 / vx0.6)：max|roll| = 1.06°, max|pitch| = 1.71°, min_height = 0.319 m，180 s 内 0 误报。
  - 56 组二维网格扫描 (40°–70° × 0.1–1.0 s)：
    - 阈值 ≥ 65° 或驻留 ≥ 0.50 s 时，侧翻回落导致姿态通道漏检，最大时延从 0.31 s 恶化至 0.98 s。
    - 选定 50° / 0.30 s：兼具 48.3° 下界防误报裕度与 15° 上界防退化裕度。
- 右栏视觉（三图组合）：
  - 【替换图片：docs/report/figures/fig03_normal_envelope.png】
  - 【替换图片：docs/report/figures/fig04_normal_vs_fall.png】
  - 【替换图片：docs/report/figures/fig07_threshold_sweep.png】

---
第 6 页：恢复状态机设计：强制“先安全阻尼、后起身”
- 标题：7 状态恢复有限状态机（RecoveryFSM）：拓扑级安全顺序保证
- 顶部金句：S1_SAFE_DAMPING 是通往起身的唯一必经前驱，9/9 正式实验与 3/3 受阻实验 100% 满足 t_damping < t_standup_cmd。
- 左栏要点：
  - 7 个状态：S0_MONITOR → S1_SAFE_DAMPING (纯阻尼 kd=2.5) → S2_ASSESS (评估/退避) → [S2B_PREPARE_LIEDOWN (官方 LieDown 1.8s)] → S3_STANDUP (官方 StandUp 3.0s) → S4_VERIFY (1.0s 直立验证) → S0 / S5_SAFE_FAULT (人工复位锁存)
  - 静置判据：S1 最短停留 0.50 s，且关节角速度与机体角速度均 < 0.15 rad/s 持续 0.30 s（或满 3.0 s）才进 S2。
- 右栏视觉：状态转移流程图 + 【替换图片：docs/report/figures/fig05_state_timeline.png】

---
第 7 页：S2_ASSESS 四路姿态分流与平台能力边界
- 标题：S2_ASSESS 四路官方状态姿态分流机制与平台物理边界
- 顶部金句：完全复用官方 JointDamping / LieDown / StandUp 原语，解决侧翻散腿起不来与四脚朝天盲目挣扎两大难题。
- 左栏表格（四路分流）：
  1. direct_standup (|roll|,|pitch|<35° 且关节偏差 <0.35 rad) → 直接进 S3_STANDUP (前翻/后翻)
  2. tuck_then_standup (趴平 <35° 但四肢散开偏差 ≥0.35 rad) → 先 S2B 调官方 LieDown 1.8s 对齐四腿至 Q_PRE，再进 S3_STANDUP (侧翻/受阻重试)
  3. side_prop_liedown (35° ≤ 倾角 < 120° 侧撑未倒平) → 先 S2B 调官方 LieDown 收腿卸除侧撑支点，再进 S3_STANDUP
  4. unrecoverable_supine (|roll|≥120° 或 |pitch|≥80° 纯仰翻/倒立) → 0 次盲目尝试，直接锁存 S5_SAFE_FAULT (E_UNRECOVERABLE_POSE)
- 右栏视觉：
  - 【替换图片：docs/report/sim_shots/grid_overview.png】
  - 【替换图片：docs/report/sim_shots/strip_supine_boundary.png】

---
第 8 页：仿真试验台：官方 MJCF + 官方 StandUp/LieDown 轨迹 + 官方 ONNX 策略
- 标题：高保真 MuJoCo 4.0 仿真试验台与 5 mm 精确无穿透姿态注入
- 顶部金句：100% 复刻云深处官方 Lite3 物理参数与轨迹公式（12 DoF, ±30 N·m, 1.5s+1.5s 两阶段三次样条 + 二连杆 IK）。
- 左栏要点：
  - 官方 MJCF 模型 (BSD-3)：thigh=0.20 m, shank=0.21 m, kp=100, kd=2.5, 物理步长 1 ms
  - 官方 StandUp 两阶段轨迹：Phase 1 (1.5s) 插值至 Q_PRE=[0, -1.35453, 2.54948]×4 (pre_height=0.12m)；Phase 2 (1.5s) 高度插值至 stand_height=0.33m 并实时解算二连杆 IK (HipY=-0.6535, Knee=1.2711)
  - 精确注入：利用 mj_geomDistance 计算全身网格至地面精确距离，留 5 mm 间隙无穿透释放。
- 右栏视觉：
  - 【替换图片：docs/assets/sim_smoke.png】
  - 【替换图片：docs/report/figures/fig06_standup.png】

---
第 9 页：正式 9 次测试结果（一）：前翻 & 后翻恢复全流程
- 标题：正式测试结果（一）：前翻（3/3）与后翻（3/3）恢复全流程
- 顶部金句：前翻经快速通道 0.155 s 检出，后翻经主通道 0.305 s 检出——均走 direct_standup 在 4.21 s 内一次平稳起身。
- 左栏要点：
  - 前翻 (front_1..3)：pitch=+90° → t=0.155s 检出并切 S1 阻尼 → t=2.140s 启动 S3_STANDUP → t=5.340s 进入 S4_VERIFY → t=6.345s 恢复 S0_MONITOR (起身耗时 4.21 s，尝试 1 次)。
  - 后翻 (back_1..3)：pitch=-90°+4.0 rad/s 回摆 → t=0.305s 检出并切 S1 阻尼 → t=1.675s 启动 S3_STANDUP → t=4.875s 进入 S4_VERIFY → t=5.880s 恢复 S0_MONITOR (起身耗时 4.21 s，尝试 1 次)。
- 右栏视觉：
  - 【替换图片：docs/report/sim_shots/strip_front.png】 + 【插入视频：videos/demo_mjcf_front.mp4】
  - 【替换图片：docs/report/sim_shots/strip_back.png】 + 【插入视频：videos/demo_mjcf_back.mp4】

---
第 10 页：正式 9 次测试结果（二）：侧翻收腿整姿 + 起身全流程
- 标题：正式测试结果（二）：侧翻（3/3）“阻尼回落 → 官方 LieDown 整姿 → 官方 StandUp 起身”
- 顶部金句：侧翻回落趴地后四腿受挤压不对称，自动切入 S2B 调用官方 LieDown 对齐四腿后再起身，左右侧翻 3/3 一次成功！
- 左栏要点（side_1..3 时间链）：
  - t=0.000s：侧倾 ±65° 注入
  - t=0.305s：attitude 通道检出 (side_left / side_right)，切 S1_SAFE_DAMPING
  - t=1.895s：阻尼下翻回趴地静止，S2_ASSESS 检测到关节偏差 ≥0.35 rad，选定 tuck_then_standup
  - t=1.900s–3.705s：S2B_PREPARE_LIEDOWN 调用官方 LieDown (1.5s 样条收腿至 Q_PRE + 0.3s 贴地)
  - t=3.705s–6.905s：S3_STANDUP 两阶段垂直顶升至 0.346 m
  - t=6.905s–7.910s：S4_VERIFY 验证 1.0s 通过，起身耗时 4.21 s，尝试 1 次。
- 右栏视觉：
  - 【替换图片：docs/report/sim_shots/strip_side_left.png】
  - 【替换图片：docs/report/figures/fig06_standup.png】
  - 【插入视频：videos/demo_mjcf_side.mp4】

---
第 11 页：正式 9 次汇总表与时延分布
- 标题：正式 9 次测试全景汇总表与判定时延分布
- 顶部金句：9/9 检出、9/9 先阻尼后起身、9/9 一次起身成功——平均判定时延 0.255 s（上限 2 s），平均起身耗时 4.21 s（上限 15 s）。
- 左栏表格：完整列出 front_1..3, side_1..3, back_1..3 共 9 行数据（检出 True、时延 0.155s/0.305s、先安全后起身 True、策略 direct/tuck、尝试 1 次、起身耗时 4.21s、最终状态 S0_MONITOR）。
- 右栏视觉：
  - 【替换图片：docs/report/figures/fig01_detection_latency.png】
  - 【替换图片：docs/report/figures/fig02_attitude_timeline.png】

---
第 12 页：失败与超时处理：起身受阻 3 次实验（最多 2 次重试 + S5_SAFE_FAULT 锁存）
- 标题：失败与超时处理：起身受阻 3 次实验（限次退避重试 + 故障锁存）
- 顶部金句：矮横梁受阻下：0.320 s 高度通道检出 → 第 1 次受阻 → 2.0 s 退避并升级 LieDown 整姿 → 第 2 次受阻后于 12.465 s 永久锁存 S5_SAFE_FAULT！
- 左栏要点（blocked_1..3 时间链）：
  - t=0.320s：趴地 roll≈0, pitch≈0，由 height 通道 (h=0.054<0.10m) 检出并切 S1
  - t=0.845s–4.050s：第 1 次尝试 (direct_standup) 被上方 0.16m 横梁挡住 (h≤0.10m)，切回 S1 阻尼卸力
  - t=5.455s–7.455s：S2_ASSESS 严格等待 backoff_s = 2.0 s 退避期，检测到脚打滑偏移，自动升级为 tuck_then_standup
  - t=7.455s–9.260s：先调官方 LieDown 重新收拢对齐四腿
  - t=9.260s–12.465s：第 2 次尝试 StandUp 仍被横梁阻挡
  - t=12.465s：attempts == 2 达到上限，锁存 S5_SAFE_FAULT (纯阻尼)，输出故障码 E_STANDUP_NOT_UPRIGHT，仅响应 manual_reset()。
- 右栏视觉：
  - 【替换图片：docs/report/sim_shots/strip_blocked.png】
  - 【插入视频：videos/demo_mjcf_blocked.mp4】

---
第 13 页：从仿真到实机：Ros2Backend 接口映射与实机保护测试 SOP
- 标题：从仿真到实机：Ros2Backend 话题映射、差异补偿与安全保护 SOP
- 顶部金句：决策层代码零改动迁移：FK+IMU 投影替代高度真值，严格执行“软垫 + 龙门吊架 (10 cm 余量) + 第三人急停”四阶段 SOP。
- 左栏要点：
  - 话题映射：/IMU_DATA + /JOINTS_DATA → Observation；控制模式 → kJointDamping / kLieDown / kStandUp；状态发布 /doggie/status；人工复位 /doggie/reset
  - 实机补偿：正向运动学 (FK) + IMU 旋转矩阵估算支撑高度替代仿真高度真值；直立静置 2.0 s 零偏校准；0.50 s 通信心跳看门狗。
- 右栏要点（四阶段实机安全 SOP）：
  - 三人责任制：主控员 + 吊架保护员 + 专职急停安全员
  - Stage 1：干跑测试（人工缓推 30°/55°/80°，只核对检出日志）
  - Stage 2：只检不立（受控侧倾 65° 放于软垫，验证 0.31 s 切入 JointDamping）
  - Stage 3：吊架辅助整姿起身（吊带留 10 cm 余量，验证 S1→S2B→S3→S4 全流程）
  - Stage 4：软垫压背受阻测试（验证 2 次尝试后锁存 S5_SAFE_FAULT 与人工复位）

---
第 14 页：总结与答辩高频 Q&A 备询
- 标题：总结与答辩高频问题（Q&A）备询
- 顶部金句：以可解释、可标定的白盒监督状态机，充分激活平台内置原语潜力，五项考核指标全面达成优秀档满分标准。
- 左栏（三大贡献总结）：
  1. 双层三通道迟滞检测器：9/9 检出 (0.155–0.305 s)，180 s 小跑 0 误报
  2. 强制安全序与四路官方原语分流状态机：9/9 先阻尼后起身，9/9 一次起身成功 (4.21 s)，纯仰翻主动安全拒止
  3. 严密受阻闭环与 56 组网格标定：3/3 受阻 2 次重试后锁存 S5_SAFE_FAULT，参数依据充分
- 右栏（4 个高频 Q&A 卡片）：
  - Q1：为什么不自己训练 RL 起身？（答：任务书要求复用平台动作；白盒状态机+官方量产原语力矩可控、故障可锁存）
  - Q2：侧翻为什么要先调 LieDown？（答：消除四腿不对称造成的单侧触地脚翻转支点，对齐至 Q_PRE 后垂直起身 100% 成功）
  - Q3：水平趴地 roll≈0, pitch≈0 怎么检出？（答：h < 0.10 m 持续 0.30 s 高度通道兜底，且与 LieDown 0.12 m 留有 2 cm 间隔）
  - Q4：四脚朝天（纯仰翻 180°）怎么办？（答：超出平台 StandUp 几何包络，S2_ASSESS 直接识别为 unrecoverable_supine 并锁存 S5_SAFE_FAULT 报警，0 次盲目挣扎）
```

---

### Prompt B：用于 ChatGPT / Claude / DeepSeek 润色 Word 技术报告或生成计时演讲稿的提示词

```markdown
你是一位资深四足机器人控制专家与国家级教学竞赛评审专家。我已完成《四足机器人 Lite3 恢复运控（任务二）技术报告》（见 docs/06-report.md）与 14 页答辩 PPT 提纲（见 docs/07-slides.md）。

请帮我完成以下两项任务（严格保持所有实验数据、状态名称、代码文件名和时间戳一字不差）：

【任务 1：生成 8 分钟精确计时的现场答辩演讲逐字稿】
- 将 14 页 PPT 的演讲词串联成一篇节奏紧凑、转承自然、富有工程感染力的 8 分钟（约 2200 字）现场答辩逐字稿。
- 在每一页切换处标注 `[翻至 P# | 预计用时 XX 秒 | 重点指向屏幕右侧 XXX 图/视频]`。
- 在讲解 P5（56 组网格标定热图）、P7（S2_ASSESS 四路官方状态分流）、P12（受阻 2 次尝试自适应升级 LieDown 后锁存 S5_SAFE_FAULT）这三处核心得分亮点时，加重语气并突出我们的工程思考深度。

【任务 2：生成评委追问防御手册（10 道压力测试问答）】
- 围绕任务书表 7 的 5 个评分项（摔倒检测 25 分、恢复流程 25 分、起身完成 25 分、失败与超时 15 分、参数依据 10 分），站在挑剔评委的角度设计 10 道最具杀伤力的现场追问，并给出基于我们实测数据的“三句话标准回答（结论 + 定量数据证据 + 代码/图表定位）”。
- 必含追问方向：
  1. 10 Hz 低通滤波会不会拖慢检测时延？
  2. 为什么后翻实验要加 4.0 rad/s 的前向回摆角速度？不加会怎样？
  3. 受阻实验第 1 次是 direct_standup，为什么第 2 次重试会自动变成 tuck_then_standup？
  4. 实机没有 MuJoCo 的 qpos[2] 高度真值，高度兜底通道在实机上怎么落地？
  5. 如果机器狗在斜坡上行走或起身，50° 阈值和 10° 直立判据还适用吗？

【核心基准数据约束（严禁篡改）】
- 正式 9 次：front_1..3 (fast_path, 0.155 s, direct_standup, 4.21 s)、side_1..3 (attitude, 0.305 s, tuck_then_standup, 4.21 s)、back_1..3 (attitude, 0.305 s, direct_standup, 4.21 s)。
- 受阻 3 次：blocked_1..3 (height < 0.10 m, 0.320 s 检出, 2 次尝试, 2.0 s 退避, 12.465 s 锁存 S5_SAFE_FAULT, E_STANDUP_NOT_UPRIGHT)。
- 正常小跑：3×60 s = 180 s (normal_turn, normal_vx0.4, normal_vx0.6)，max|roll|=1.06°, max|pitch|=1.71°, min_height=0.3193 m，0 误报，距 50° 裕度 48.29°。
```

---

### Windows 本地素材插入对照表（图表 & 视频 $\leftrightarrow$ 报告章节 & PPT 页码）

在 Windows 下将仓库克隆或复制到本地后，所有图片与 MP4 视频的相对路径、分辨率用途以及在 Word 报告（`docs/06-report.md`）和答辩 PPT（`docs/07-slides.md`）中的精确插入位置对照如下：

| 素材类别 | 仓库内本地文件相对路径（Windows 下可直接拖入 PPT/Word） | 内容说明与核心看点 | 对应技术报告 (`docs/06-report.md`) 章节 | 对应答辩 PPT (`docs/07-slides.md`) 页码 |
|---|---|---|---|---|
| **数据分析图 1** | `docs\report\figures\fig01_detection_latency.png` | 正式 9 次翻倒判定时延柱状图（前翻 $0.155\text{ s}$，侧/后翻 $0.305\text{ s}$ vs $2.0\text{ s}$ 优秀线） | **5.1 节**（图 5-1） | **P2**（备选）、**P11**（右栏主图） |
| **数据分析图 2** | `docs\report\figures\fig02_attitude_timeline.png` | 前翻/左侧翻/后翻 Roll & Pitch 时间线、$50^\circ$ 阈值线、$35^\circ$ 回差线与状态色块区间 | **5.1 节**（图 5-2） | **P4**（右栏配图）、**P11**（右栏下图） |
| **数据分析图 3** | `docs\report\figures\fig03_normal_envelope.png` | 官方 `policy.onnx` 策略 3×60 s 正常小跑 Roll/Pitch/Height 动态包络曲线（0 误报） | **5.3 节**（图 5-10） | **P5**（右栏左上图） |
| **数据分析图 4** | `docs\report\figures\fig04_normal_vs_fall.png` | 正常小跑姿态分布（$\le 1.71^\circ$）与翻倒分布（$\ge 65^\circ$）分离度及 $48.29^\circ$ 安全裕度图 | **5.3 节**（图 5-11） | **P5**（右栏左下图） |
| **数据分析图 5** | `docs\report\figures\fig05_state_timeline.png` | 典型实验的状态机转移阶梯时间线（直观证明 100% 先 `S1_SAFE_DAMPING` 后 `S3_STANDUP`） | **5.1 节**（图 5-3） | **P6**（右栏下图） |
| **数据分析图 6** | `docs\report\figures\fig06_standup.png` | 官方两阶段 `StandUp`（$1.5\text{ s}+1.5\text{ s}$）+ `S4_VERIFY`（$1.0\text{ s}$）高度与俯仰角收敛轨迹 | **5.1 节**（图 5-4） | **P8**（右栏下图）、**P10**（右栏下图） |
| **数据分析图 7** | `docs\report\figures\fig07_threshold_sweep.png` | 姿态阈值（$40^\circ\text{–}70^\circ$）× 驻留时间（$0.1\text{–}1.0\text{ s}$）共 56 组网格标定最大时延热力图 | **5.4 节**（图 5-12） | **P5**（右栏主热力图） |
| **仿真全景图** | `docs\report\sim_shots\grid_overview.png` | 五大典型场景（前翻、侧翻、后翻、受阻、纯仰翻边界）4×5 全流程关键帧总览网格 | **4.2 节**（图 4-1） | **P1**（封面主图）、**P7**（右栏上图） |
| **仿真序列图 1** | `docs\report\sim_shots\strip_front.png` | 前翻（`front_1`）4 帧连续快照：注入 $(+90^\circ) \to$ `S1` 阻尼 $\to$ `S3` 起身 $\to$ `S0` 恢复直立 | **5.1 节**（图 5-5） | **P9**（右栏上图） |
| **仿真序列图 2** | `docs\report\sim_shots\strip_side_left.png` | 左侧翻（`side_1`）4 帧连续快照：注入 $(+65^\circ) \to$ `S1` 阻尼 $\to$ `S2B` `LieDown` 整姿 $\to$ `S0` 恢复 | **5.1 节**（图 5-6） | **P10**（右栏上图） |
| **仿真序列图 3** | `docs\report\sim_shots\strip_back.png` | 后翻（`back_1`）4 帧连续快照：回摆注入 $(-90^\circ) \to$ `S1` 阻尼 $\to$ `S3` 起身 $\to$ `S0` 恢复直立 | **5.1 节**（图 5-7） | **P9**（右栏下图） |
| **仿真序列图 4** | `docs\report\sim_shots\strip_blocked.png` | 起身受阻（`blocked_1`）4 帧快照：$0.32\text{ s}$ 高度检出 $\to$ 尝试 1 受阻 $\to$ 整姿+尝试 2 受阻 $\to$ 锁存 `S5` | **5.2 节**（图 5-8） | **P12**（右栏上图） |
| **仿真序列图 5** | `docs\report\sim_shots\strip_supine_boundary.png` | 纯仰翻 $180^\circ$ 物理边界 4 帧快照：快速检出 $\to$ 阻尼静置 $\to$ `S2_ASSESS` 识别不可恢复 $\to$ 锁存 `S5` | **5.2 节**（图 5-9） | **P7**（右栏下右图） |
| **预研对照图 1** | `docs\assets\sim_smoke.png` | 官方 Lite3 MJCF 模型标准站立冒烟测试图（高度 $0.346\text{ m}$） | `docs/08-feasibility.md` | **P1**（小图）、**P3**、**P8**（右栏上图） |
| **预研对照图 2** | `docs\assets\side65_settled.png` | 静态侧翻腿伸展不调 `LieDown` 时的约 $57^\circ$ 侧撑卡死姿态（反面机理对照） | `docs/08-feasibility.md` | **P7**（右栏下左图） |
| **演示视频 1** | `videos\demo_mjcf_front.mp4` | **未剪辑视频**：前翻 $+90^\circ \to 0.155\text{ s}$ 检出 $\to$ 阻尼 $\to$ `direct_standup` $\to 4.21\text{ s}$ 恢复 | **5.1 节**配套视频 | **P9**（右上角嵌入播放） |
| **演示视频 2** | `videos\demo_mjcf_side.mp4` | **未剪辑视频**：侧翻 $+65^\circ \to 0.305\text{ s}$ 检出 $\to$ 阻尼 $\to$ `LieDown` 整姿 $\to$ `StandUp` 恢复 | **5.1 节**配套视频 | **P10**（右下角嵌入播放） |
| **演示视频 3** | `videos\demo_mjcf_back.mp4` | **未剪辑视频**：后翻 $-90^\circ$ 回摆 $\to 0.305\text{ s}$ 检出 $\to$ 阻尼 $\to$ `direct_standup` $\to$ 恢复 | **5.1 节**配套视频 | **P9**（右下角嵌入播放） |
| **演示视频 4** | `videos\demo_mjcf_blocked.mp4` | **未剪辑视频**：矮横梁受阻 $\to 0.320\text{ s}$ 高度检出 $\to 2$ 次尝试（含 $2\text{ s}$ 退避与整姿）$\to$ 锁存 `S5_SAFE_FAULT` | **5.2 节**配套视频 | **P12**（右下角嵌入播放） |
