# 可中断 SDK 全姿态自翻正与实机恢复（`experiments/sdk_self_right`）

本目录是按 [docs/11-handoff.md](../../docs/11-handoff.md) 的独立实验约束实现的 **Lite3 可中断 SDK 全姿态（仰倒 / 左右侧倒 / 前摔 / 后摔）自翻正、单侧障碍自动换侧、二次失败永久锁停与零落差原厂起立方案**，**不修改主路径** `src/doggie/`、`config/recovery.yaml` 或正式 9/9 回归测试。

- 详细使用、实机操作与排障手册：[docs/12-interruptible-sdk-recovery.md](../../docs/12-interruptible-sdk-recovery.md)
- 任务二（恢复运控）实机验收数据包、参数标定表与 4 张实机图表：[report/task2_real_data_report.md](report/task2_real_data_report.md)

## 核心能力与解决的问题

1. **全姿态自动分流恢复（仰倒 / 左右侧倒 / 前摔 / 后摔）**：
   - **侧倒/仰倒（`tilt ≥ 35°` 无条件检出）**：稳定静置 `2.0 s` 后，按侧倾方向自动选择**左翻**（`left_roll`）或**镜像右翻**（`right_roll`），执行 6 段五次多项式平滑翻身（`lite3_left_roll_v3`）。
   - **前摔机首着地 / 后摔坐倒（`18° ≤ tilt < 35°` 且 `est_height_m < 0.19 m`）**：高度门限 `0.19 m` 精确拦截 20°~25° 斜坡站立（实机 681 帧斜坡 0 误报），并配合 `post_fall_latched` 事件锁存解决前/后摔滑平到肚皮贴地（`1.4°~11.3°`）后的漏恢复问题；分别加载 `front_restore_profile.json` 与 `back_restore_profile.json` 柔和折叠理顺四腿后零落差起立。
2. **全程可中断、0~1 ms 纯阻尼急停 + 单侧受阻自动换侧 + 二次失败永久锁停**：
   - 同一互斥锁内串行化 STOP 锁存与 UDP 发送边界，任意时刻收到物理手柄 STOP（`deeprcs` Soft Emergency）、终端取消、受阻停滞、力矩护栏越界、反馈断流（`>200 ms`）或指令过期（`>20 ms`），实机实测 **`0~1 ms`** 切纯阻尼（`kp = 0, kd = 2.5`）。
   - **单侧障碍自动换侧**：翻身第 1 次因障碍受阻停机后，纯阻尼静置 `2.0 s`（`--retry-wait-ms 2000`）自动从**另一侧**镜像翻身重试（实机 **5/5** 成功）。
   - **二次失败永久安全锁停**：若第 2 次仍受阻（双侧/顶障，实机 **3/3**），或收到人工 STOP，立即永久锁定并输出中文 `[ALERT / SAFE_FAULT]` / `[ALERT / STOP]` 告警，必须人工扶正站稳 `3.0 s` 才复位，**杜绝无限重试**。
3. **零落差无缝移交原厂起立**：翻正对齐 `PRE_POSE` 并趴稳 `0.5 s` 后，在 SDK 仍托住机身时预发原厂站立命令 `0x21010202`，监听到 `jy_exe` 进入 `Standing up, please wait...`（~5 ms）后瞬间发 `0x113` 交回原厂完成力控站立，起立后物理手柄立即可操控行走。
4. **开机默认关闭**：不安装任何 systemd 自启服务，仅在显式执行 `scripts/run_on_robot.sh start` 后启用，随时 `stop` / `cancel` 关闭。

## 目录结构

```text
experiments/sdk_self_right/
├── README.md                                  # 本说明
├── recovery_sdk/                              # 独立 C++17 门控库 + 单测 + 轨迹配置 + MuJoCo 仿真
│   ├── CMakeLists.txt
│   ├── config/
│   │   ├── left_roll_profile.json             # v3 6 段左翻配置（镜像右翻由程序实时生成）
│   │   ├── front_restore_profile.json         # 前摔机首着地 2 段整腿配置
│   │   └── back_restore_profile.json          # 后摔坐倒 2 段降前躯整腿配置
│   ├── include/lite3_recovery/                # guard / sequence(含post_fall_latched) / controller / standup / kinematics / profile_loader / reservation
│   ├── sim/
│   │   ├── mujoco_roll_eval.py                # 34 项 MuJoCo 物理回归与 3D 交互演示（复用仓库 Lite3.xml）
│   │   └── generate_real_report.py            # 真机 JSONL 审计日志自动统计、标定与 4 图生成脚本
│   └── tests/                                 # C++ 门控、3 套配置校验、多线程异步 STOP 延迟测试
├── robot_node/
│   ├── interruptible_recovery_node.cpp        # 机器狗板载 ROS 2 Foxy + MotionSDK 单进程节点
│   └── sdk_deploy_integration.patch           # 接入官方 sdk_deploy (lite3_transfer) 的最小补丁
├── scripts/
│   └── run_on_robot.sh                        # 机器狗本机 start/stop/status/dry-run/stop-test/recover 脚本
└── report/                                    # 任务二实机验收数据包与 4 张高清实机图表
    ├── task2_real_data_report.md              # 对照表 7 达标汇总、逐次表与参数标定报告
    ├── real_trials_table.csv                  # 实机 38 周期明细表
    ├── normal_vs_fall_calibration.csv         # 105,913 帧非跌倒 vs 37 次真跌倒阈值网格扫描
    └── figures/                               # 4 张实机标定/耗时/时序/护栏对比 PNG 图表
```

## 快速验证（离线 / 仿真，无需连接实机）

```bash
# 1. C++17 recovery_sdk 门控与多线程异步 STOP 单测（2/2 PASS）
cmake -S experiments/sdk_self_right/recovery_sdk -B /tmp/lite3-recovery-sdk-build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/lite3-recovery-sdk-build -j
ctest --test-dir /tmp/lite3-recovery-sdk-build --output-on-failure

# 2. MuJoCo 34 项全套无头回归（8 仰倒 + 8 侧倒 + 8 前/后摔 + 5 STOP + 2 单侧换侧 + 3 双侧/顶障锁停）
python3 experiments/sdk_self_right/recovery_sdk/sim/mujoco_roll_eval.py --suite all

# 3. 3D 窗口交互演示
python3 experiments/sdk_self_right/recovery_sdk/sim/mujoco_roll_eval.py --view --scenario demo_all
python3 experiments/sdk_self_right/recovery_sdk/sim/mujoco_roll_eval.py --view --scenario demo_pitch
python3 experiments/sdk_self_right/recovery_sdk/sim/mujoco_roll_eval.py --view --scenario demo_obstacle
```
