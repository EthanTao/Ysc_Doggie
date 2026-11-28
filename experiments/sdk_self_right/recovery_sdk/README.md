# `recovery_sdk` — Lite3 可中断 SDK 恢复核心库（C++17 + MuJoCo 验证）

> 上层完整介绍见 [../README.md](../README.md)，实机操作与全量命令见 [../../../docs/12-interruptible-sdk-recovery.md](../../../docs/12-interruptible-sdk-recovery.md)。

## 模块组成

- **`include/lite3_recovery/guard.hpp`**：发包边界护栏（同一互斥锁串行化 STOP 锁存与发送；检查单调时钟、20 ms 指令过期、200 ms 反馈看门狗、关节位置/速度/加速度/增益变化率与虚拟 PD 力矩护栏 `effort_safe()`；任意 STOP/故障立即只输出纯阻尼 `kp = 0, kd = 2.5`）。
- **`include/lite3_recovery/sequence.hpp`**：跌倒检出（倾角与驻留可配）、稳定等待、分段五次多项式插值（`valid_segment` / `valid_trajectory` / `interpolate`）、受阻停滞窗口（`StallWindow`）与 10 s 尝试超时状态机。
- **`include/lite3_recovery/controller.hpp`**：整合 `Sequence` 与 `Guard`，支持从实测初始关节姿态平滑进入首段轨迹（`smooth_entry_from_actual`），并在 `prone_check → standing` 保持 C0 连续性。
- **`include/lite3_recovery/standup.hpp`**：复刻官方 `StandUpState` 三次样条站立轨迹（SDK 兜底模式），实机默认使用 `native-handoff` 零落差移交原厂起立。
- **`include/lite3_recovery/kinematics.hpp`**：RPY → 四元数转换、Lite3 腿部正运动学高度估计与趴稳支撑确认（`assess_prone_support`）。
- **`include/lite3_recovery/profile_loader.hpp`**：加载 JSON 轨迹配置并校验限幅。
- **`include/lite3_recovery/reservation.hpp`**：单轮实验排他持久化防重入锁。
- **`config/left_roll_profile.json`**：经过 MuJoCo 22 项回归与重载/高阻尼压力测试的 `lite3_left_roll_v3` 6 段左翻轨迹配置（右侧倒镜像轨迹由运行期实时生成）。
- **`sim/mujoco_roll_eval.py`**：MuJoCo 物理验证器，内建与 C++ 一致的五次多项式插值、`Guard::effort_safe()`、`StallWindow`、自动左/右选向与 3D 交互窗口（`--view`）。
- **`tests/`**：`recovery_tests.cpp`、`transport_mock_tests.cpp`（含多线程异步 STOP 延迟测试）、`replay.cpp`。

## 编译与测试

```bash
# C++17 单元测试（-Wall -Wextra -Wpedantic -Werror）
cmake -S experiments/sdk_self_right/recovery_sdk -B /tmp/lite3-recovery-sdk-build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/lite3-recovery-sdk-build -j
ctest --test-dir /tmp/lite3-recovery-sdk-build --output-on-failure

# MuJoCo 22 项全套无头回归
python3 experiments/sdk_self_right/recovery_sdk/sim/mujoco_roll_eval.py --suite all
```
