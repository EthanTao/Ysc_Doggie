# 可中断 SDK 侧倒/仰倒自翻正与实机恢复（`experiments/sdk_self_right`）

本目录是按 [docs/11-handoff.md](../../docs/11-handoff.md) 的独立实验约束实现的 **Lite3 可中断 SDK 自翻正与零落差原厂起立方案**，**不修改主路径** `src/doggie/`、`config/recovery.yaml` 或正式 9/9 回归测试。

详细使用、实机操作与排障手册见：[docs/12-interruptible-sdk-recovery.md](../../docs/12-interruptible-sdk-recovery.md)。

## 解决的问题

1. **补齐侧倒与纯仰面（背部着地）自翻正**：倾角 `≥ 45°` 检出跌倒，稳定默认 `2.0 s`（`--stable-wait-ms 2000`）后，根据侧倾方向自动选择**左翻**或**镜像右翻**（`-165° < roll < 0°`）执行 6 段五次多项式平滑翻身（`lite3_left_roll_v3`）。
2. **全程可中断、≤20 ms 纯阻尼安全护栏（实机测得 0 ms）**：原厂高层 `AbnormalPostureRecovery()` 在翻身中段不响应 STOP；本方案使用独立 C++17 `Guard` 在同一互斥锁内串行化 STOP 锁存与发送边界，任意时刻收到物理手柄急停（`deeprcs` Soft Emergency）、取消信号、受阻停滞、力矩护栏越界、反馈断流（>200 ms）或指令过期（>20 ms），立即切纯阻尼（`kp = 0, kd = 2.5`），`0.25 s` 后发 `0x113` 归还手柄控制权并锁存等待人工扶正 3 s，杜绝无限重试。
3. **零落差无缝移交原厂起立**：翻正对齐 `PRE_POSE` 并趴稳 `0.5 s` 后，在 SDK 仍托住机身时预发原厂站立命令 `0x21010202`，监听到 `jy_exe` 进入 `Standing up, please wait...`（~5 ms）后瞬间发 `0x113` 交回原厂完成起立与力控站立，避免“突然卸力趴地再起”，且起立完成后物理手柄立即可操控行走。
4. **开机默认关闭**：不安装任何 systemd 自启服务，仅在显式执行 `scripts/run_on_robot.sh start` 后启用，随时 `stop` / `cancel` 关闭。

## 目录结构

```text
experiments/sdk_self_right/
├── README.md                                  # 本说明
├── recovery_sdk/                              # 独立 C++17 门控库 + 单测 + 轨迹配置 + MuJoCo 仿真
│   ├── CMakeLists.txt
│   ├── config/left_roll_profile.json          # v3 6 段左翻配置（镜像右翻由程序实时生成）
│   ├── include/lite3_recovery/                # guard / sequence / controller / standup / kinematics / profile_loader / reservation
│   ├── sim/mujoco_roll_eval.py                # 22 项 MuJoCo 回归与 3D 交互演示（复用仓库 sim/models/lite3/mjcf/Lite3.xml）
│   └── tests/                                 # C++ 门控、轨迹、多线程异步 STOP 延迟测试
├── robot_node/
│   ├── interruptible_recovery_node.cpp        # 机器狗板载 ROS 2 Foxy + MotionSDK 单进程节点
│   └── sdk_deploy_integration.patch           # 接入官方 sdk_deploy (lite3_transfer) 的最小补丁
└── scripts/
    └── run_on_robot.sh                        # 机器狗本机 start/stop/status/dry-run/stop-test/recover 脚本
```

## 快速验证（离线 / 仿真，无需连接实机）

```bash
# 1. C++17 recovery_sdk 门控与多线程异步 STOP 单测（2/2 PASS）
cmake -S experiments/sdk_self_right/recovery_sdk -B /tmp/lite3-recovery-sdk-build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/lite3-recovery-sdk-build -j
ctest --test-dir /tmp/lite3-recovery-sdk-build --output-on-failure

# 2. MuJoCo 22 项全套无头回归（8 仰倒/扰动 + 8 左右多角度侧倒 + 3 中途 STOP + 3 受阻）
python3 experiments/sdk_self_right/recovery_sdk/sim/mujoco_roll_eval.py --suite all

# 3. 3D 窗口交互演示（6 个代表性场景）
python3 experiments/sdk_self_right/recovery_sdk/sim/mujoco_roll_eval.py --view --scenario demo_all
```
