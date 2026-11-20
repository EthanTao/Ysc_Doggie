# Doggie — Lite3 摔倒检测与恢复运控

> 课程实践任务二 · 2026 秋
> 核心链路：**判定摔倒 → 转入安全状态 → 尝试起身 → 失败则停在安全状态并提示**

## 一句话方案

以 IMU 姿态（roll / pitch / 投影重力）为主判据，关节姿态与角速度为辅助确认；用「阈值 + 驻留时间 + 回差」抑制行进中的误报。确认摔倒后锁存事件，先切 `JointDamping` 安全状态，待机身静止再调用平台 `StandUp`；单次起身 20 s 超时、最多 2 次尝试，仍失败则进入 `SAFE_FAULT` 并报警，等待人工复位。

## 当前状态

- [x] 任务书解读与评分映射
- [x] 总体方案与四周计划
- [x] 仿真可行性预研（MuJoCo 加载官方 MJCF、站立、离屏渲染、翻倒姿态筛选）
- [x] 仿真工程化（场景、姿态注入、官方站立轨迹）
- [x] 摔倒检测器 v0（含单元测试）
- [x] 恢复状态机 v0（含超时/重试/锁存）
- [x] 正式 9 次矩阵 + 受阻 3 次（检出 9/9、顺序 9/9、起身 9/9、受阻 3/3）
- [x] 官方 ONNX 策略小跑误报测试（3×60 s，0 误报）
- [x] 阈值 / 回差 / 驻留时间标定
- [x] ROS2 实机后端与安全看门狗（`Ros2Backend`、`SafetyMonitor`、`run_real.py`；默认只读干跑）
- [ ] 实机联调与受控倾斜验收（按 `docs/09-real-machine.md` 四阶段 SOP）
- [x] 技术报告与答辩材料（`docs/06-report.md`、`docs/07-slides.md`）

当前 MuJoCo 演示按官方 `StandUpState` 默认时长设置每阶段 1.5 s。仿真实现复刻轨迹结构，不是直接调用官方控制器。

## 当前完整矩阵（2026-11-19，官方 MJCF 展示场景）

| 指标 | 结果 |
|---|---|
| 摔倒判定 | 9/9，误报 0 |
| 判定时延 | 0.155–0.305 s（优秀档 ≤2 s） |
| 先安全后起身 | 9/9（每次 `t_damping < t_standup`） |
| 起身成功 | **9/9**：前翻 3/3、侧翻 3/3、后翻 3/3（后翻采用前向回摆构造落到趴地） |
| 起身耗时 | 4.21 s（优秀档 ≤15 s） |
| 受阻处理 | 3/3 停 `S5_SAFE_FAULT` + 故障码，尝试 2 次后锁存 |
| 正常小跑误报 | 3×60 s，**0 次**；包络 |roll| ≤1.06°、|pitch| ≤1.71° |

复现：`python scripts/build_scene.py`（首次或改场景后）→ `python scripts/run_matrix.py --suite all`；明细见 `data/processed/summary.md`，视频在 `videos/`。

展示视频（官方 MJCF 场景，1280×720）：

- `videos/demo_mjcf_front.mp4`（前翻 → 起身）
- `videos/demo_mjcf_side.mp4`（侧翻 → 起身）
- `videos/demo_mjcf_back.mp4`（后翻 → 起身）
- `videos/demo_mjcf_blocked.mp4`（趴地受阻 → 安全故障）

汇报数据：`python scripts/calibrate_thresholds.py` → `python scripts/make_report_pack.py`，输出 `docs/report/data-pack.md` 与 7 张图。

## 目标档位

按任务书表 7，目标为**优秀档**（各项条件同时满足）：

| 考核项 | 优秀档要求 | 本方案对应手段 |
|---|---|---|
| 摔倒检测（25） | 9/9 正确，误报 0，时延 ≤2 s | 多判据 + 0.3 s 驻留 + 回差；9 种姿态全覆盖 |
| 恢复流程（25） | 9/9 先安全后起身，能说明切换条件 | 显式 FSM，转移条件与时间戳全部落盘 |
| 起身完成（25） | 9/9 成功，平均 ≤15 s | 复用平台站立轨迹；后翻采用「后仰 + 前向回摆」构造落到趴地 |
| 失败与超时（15） | 3/3 停安全并提示，无无限重试 | 20 s 超时、最多 2 次、退避 2 s、故障锁存 |
| 参数依据（10） | 回差与持续时间标定数据 | 网格扫描 + 实测分布 + 迟滞实验 |

> 说明：平台 `StandUp` 只能从趴地起身。正式 9 次的翻倒姿态按此构造：前翻落到趴地；侧翻带前向动量翻正到趴地；后翻带前向回摆越过仰面落到趴地。**真正的四脚朝天（无回摆）仍然起不来**，作为边界分析与失败案例单独展示，不混入正式 9 次。

## 文档索引

- [任务理解与评分映射](docs/00-requirements-traceability.md)
- [总体计划（技术路线、状态机、测试、进度）](docs/01-plan.md)
- [技术报告（完整正文、公式、图表与标定）](docs/06-report.md)
- [答辩 PPT 逐页稿与 AI 生成提示词](docs/07-slides.md)
- [仿真可行性预研（含起身边界实测）](docs/08-feasibility.md)
- [实机验收指南（ROS2 话题、四阶段 SOP、指标口径与风险）](docs/09-real-machine.md)
- [汇报数据包（指标表 + 7 张图）](docs/report/data-pack.md)

## 目录结构

```
Doggie/
├── docs/                  # 方案、设计、测试与报告
├── config/                # 检测阈值与恢复策略参数
├── src/doggie/            # 决策层（检测器 + 状态机 + 后端抽象）
│   ├── core/              # 与平台无关的纯逻辑
│   ├── io/                # SimBackend / Ros2Backend
│   └── utils/             # 滤波、日志、指标
├── sim/                   # MuJoCo 场景、官方模型、摔倒注入
├── scripts/               # 单次试验、批量矩阵、标定、绘图、录像
├── tests/                 # 单元测试与回归测试
├── data/                  # raw / processed / calibration
└── videos/                # 未剪辑演示视频
```

## 快速开始

```bash
# 1. 环境
python3 -m venv .venv && source .venv/bin/activate
pip install -e ".[dev]"

# 2. 由官方 MJCF 生成展示场景（仓库已带生成结果；改场景后重跑）
python scripts/build_scene.py

# 3. 冒烟测试：加载官方 MJCF 生成的场景，站立 5 s（可选出图）
MUJOCO_GL=egl python scripts/sim_smoke_test.py --render smoke.png

# 4. 实时观看仿真（MuJoCo 窗口，0.5 倍速）
python scripts/watch_sim.py --playlist all --speed 0.5 --loop      # 5 类场景连播，自动切 flat/blocked
python scripts/watch_sim.py --fall side_left --speed 0.5           # 单场景
python scripts/watch_sim.py --playlist side_left@flat,prone@blocked --speed 0.5
# 单项可写 fall@scene；省略 @scene 时用 --scene（默认 flat）
python scripts/watch_sim.py --fall prone --scene blocked --speed 0.5

# 5. 实机（默认只读干跑，必须显式 --enable-actuation 才下发运动指令）
source /opt/ros/jazzy/setup.bash                    # 按实际发行版
python scripts/selftest_ros2.py --duration 15       # ROS2 回环自检（不需要机器人）
python scripts/run_real.py --duration 60                  # 只读验证话题/姿态
python scripts/run_real.py --duration 300 --enable-actuation
```

## 安全声明

- **仿真优先**。所有摔倒动作先在 MuJoCo 中验证，再考虑实机。
- 实机必须在软垫、保护架/吊架、急停手柄可用的情况下进行，且只做**受控倾斜**，不自由摔倒。
- 改变机器人姿态前先切 `JointDamping`，不得带电硬掰关节。
- 演示前冻结版本并打 tag，避免现场临时改动。
