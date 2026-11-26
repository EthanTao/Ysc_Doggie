# Lite3 可中断 SDK 侧倒/仰倒恢复使用与命令手册

> 对应独立扩展目录：[experiments/sdk_self_right/](../experiments/sdk_self_right/)  
> 约束说明：与主路径 `src/doggie/` 完全隔离，不改动 `src/doggie/` 正式 9/9 矩阵结论，也不修改机器狗出厂 `~/jy_exe` 与 `~/sdk_service`；实机端独立部署在 `~/lite3_interruptible_ws`，开机默认关闭。

---

## 一、工作原理与安全特性

- **默认开机关闭**：不安装任何开机自启服务，开机时完全不接管端口或控制权；仅在显式执行 `run_on_robot.sh start` 后保持运行，直到机器狗关机或执行 `stop`/`cancel`。
- **待命阶段完全只读**：处于 `observing` 阶段时不发送任何运动 UDP 指令，物理遥控手柄可正常控制站立、行走、趴下。
- **全角度侧倒/仰倒自动选向 + 全程可中断（实机验证 0 ms 切纯阻尼）**：
  - 机身倾角 `≥ 45°` 检出跌倒，稳定 **2.0 秒**（默认 `--stable-wait-ms 2000`，支持 `300..30000 ms` 可调）后，根据侧倾符号自动选择**左翻**（左侧着地或仰倒）或**镜像右翻**（右侧着地 `-165° < roll < 0°`），接管 SDK 执行 **6 段五次多项式翻身**（`lite3_left_roll_v3`：`supine_tuck` → `left_swing_prep` → `left_knee_push` → `roll_and_catch` → `catch_settle` → `prone_align`，总时长 `4400 ms`）。
  - 任意时刻按下物理手柄 STOP、终端取消、发生障碍受阻停滞、虚拟力矩护栏越界、硬件反馈断流（>200 ms）或指令过期（>20 ms），同一互斥锁内 **0–2 ms** 切入纯阻尼（`kp = 0, kd = 2.5`），`0.25 s` 后发送 `0x113` 归还手柄控制权，并进入暂停保护（**需人工扶正并直立站稳 3.0 秒**后才重新待命，绝不无限重试）。
- **零落差直接起立并恢复手柄控制**：翻正对齐 `PRE_POSE` 并确认趴稳 `0.5 s` 后，在 SDK 仍撑住机身时预发原厂站立命令 `0x21010202`，监听到 `jy_exe` 输出 `Standing up, please wait...`（~5 ms）后立即发 `0x113` 无缝切回原厂完成站立并进入力控模式，**不会突然卸力趴地**，起立完成后手柄立即可直接操控行走，站稳 `0.5 s` 后自动重新进入监测待命。

---

## 二、离线单元测试与本地 MuJoCo 物理仿真

仿真直接复用本仓库 [sim/models/lite3/mjcf/Lite3.xml](../sim/models/lite3/mjcf/Lite3.xml)（脚本内自动按 Lite3 实机硬件关节限幅 `HipY ∈ [-3.50, 0.320]` 运行），并与 C++17 `recovery_sdk` 保持一致的五次多项式插值、自动左/右镜像翻身选向、`Guard::effort_safe()` 虚拟力矩护栏、纯阻尼急停与停滞检测。

在仓库根目录执行（如使用独立虚拟环境或 conda，将 `python3` 替换为对应解释器即可；去掉 `--view` 即为无窗口快速跑出 JSON 结果，追加 `--speed 0.5` 可半速慢放）：

```bash
SIM=experiments/sdk_self_right/recovery_sdk/sim/mujoco_roll_eval.py
```

### 1. 组合 3D 窗口演示

```bash
# 依次演示 6 个代表性场景（180°仰倒左翻 → 左侧59° → 右侧90°镜像右翻 → 2s中途STOP → 侧墙受阻 → 顶障受阻）
python3 "$SIM" --view --scenario demo_all

# 依次演示 6 个多角度左右侧倒场景（±59° / ±90° / ±110° 自动选向）
python3 "$SIM" --view --scenario demo_sides
```

### 2. 单项仿真命令 — 仰倒、扰动、摩擦与软增益（8 项）

```bash
python3 "$SIM" --view --scenario nominal_180      # 标准仰倒 180° 左翻 + 站立
python3 "$SIM" --view --scenario roll_165         # 偏倾仰倒 165° 左翻 + 站立
python3 "$SIM" --view --scenario roll_195         # 偏倾仰倒 195° 左翻 + 站立
python3 "$SIM" --view --scenario pitch_p10        # 仰倒 +10° 俯仰扰动
python3 "$SIM" --view --scenario pitch_m10        # 仰倒 -10° 俯仰扰动
python3 "$SIM" --view --scenario low_fric_0.6     # 低摩擦地面 (μ=0.6) 仰倒左翻
python3 "$SIM" --view --scenario high_fric_1.4    # 高摩擦地面 (μ=1.4) 仰倒左翻
python3 "$SIM" --view --scenario soft_kp_0.7      # 70% 软增益仰倒左翻
```

### 3. 单项仿真命令 — 多角度左右侧倒自动左/右翻（8 项）

```bash
python3 "$SIM" --view --scenario side_left_59     # 左侧倒 +59° 自动左翻 + 站立
python3 "$SIM" --view --scenario side_right_59    # 右侧倒 -59° 自动镜像右翻 + 站立
python3 "$SIM" --view --scenario side_left_75     # 左侧倒 +75° 自动左翻 + 站立
python3 "$SIM" --view --scenario side_right_75    # 右侧倒 -75° 自动镜像右翻 + 站立
python3 "$SIM" --view --scenario side_left_90     # 左侧倒 +90° 自动左翻 + 站立
python3 "$SIM" --view --scenario side_right_90    # 右侧倒 -90° 自动镜像右翻 + 站立
python3 "$SIM" --view --scenario side_left_110    # 左侧倒 +110° 自动左翻 + 站立
python3 "$SIM" --view --scenario side_right_110   # 右侧倒 -110° 自动镜像右翻 + 站立
```

### 4. 单项仿真命令 — 早/中/起立段中途 STOP 急停（3 项）

```bash
python3 "$SIM" --view --scenario stop_early       # 翻身早段 (0.8s) STOP 立即切纯阻尼
python3 "$SIM" --view --scenario stop_mid         # 翻身中段 (2.0s) STOP 立即切纯阻尼
python3 "$SIM" --view --scenario stop_stand       # 起立段 (5.5s) STOP 立即切纯阻尼
```

### 5. 单项仿真命令 — 障碍物受阻保护（3 项）

```bash
python3 "$SIM" --view --scenario blocked_side_wall   # 左侧刚体墙受阻 → 安全锁存纯阻尼
python3 "$SIM" --view --scenario blocked_overhead    # 上方低矮顶障受阻 → 安全锁存纯阻尼
python3 "$SIM" --view --scenario blocked_stand_ceil  # 起立顶部遮挡受阻 → 安全锁存纯阻尼
```

### 6. 自定义任意角度 / 摩擦 / 刚度 / STOP 时刻

```bash
# 例：右侧倒 -82°、俯仰 +5°、半速慢放观看自动镜像右翻
python3 "$SIM" --view --scenario custom --roll -82 --pitch 5 --speed 0.5

# 例：左侧倒 +100°、1.5 秒注入 STOP 测试中途阻尼
python3 "$SIM" --view --scenario custom --roll 100 --stop-ms 1500
```

### 7. 无头批量回归测试（22 项 MuJoCo + C++17 门控单测）

```bash
# 一次性无头运行全部 22 项 MuJoCo 仿真（预期 22/22 PASS）
python3 "$SIM" --suite all

# 编译并运行全部 C++17 recovery_sdk 离线单测与多线程异步 STOP 延迟测试（预期 2/2 PASS）
cmake -S experiments/sdk_self_right/recovery_sdk -B /tmp/lite3-recovery-sdk-build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/lite3-recovery-sdk-build -j
ctest --test-dir /tmp/lite3-recovery-sdk-build --output-on-failure -V
```

---

## 三、机器狗端首次部署与编译（ROS 2 Foxy ARM64）

本节点基于官方 `sdk_deploy` 的 `lite3_transfer` 与 `Lite3_MotionSDK` 构建，**不修改出厂 `~/jy_exe` 与 `~/sdk_service`**。

1. 在机器狗上准备独立的 `sdk_deploy` 工作区（例如 `~/lite3_interruptible_ws`），将本目录文件拷入并应用最小集成补丁：
   ```bash
   # 在机器狗 ~/lite3_interruptible_ws 根目录执行（假定已克隆官方 sdk_deploy）
   # 1) 复制 recovery_sdk、脚本与节点源码
   cp -r <repo>/experiments/sdk_self_right/recovery_sdk ~/lite3_interruptible_ws/
   cp -r <repo>/experiments/sdk_self_right/scripts ~/lite3_interruptible_ws/
   mkdir -p ~/lite3_interruptible_ws/src/lite3_transfer/src
   cp <repo>/experiments/sdk_self_right/robot_node/interruptible_recovery_node.cpp \
      ~/lite3_interruptible_ws/src/lite3_transfer/src/

   # 2) 应用 lite3_transfer/CMakeLists.txt 与 state_machine_base.h 补丁
   cd ~/lite3_interruptible_ws
   git apply <repo>/experiments/sdk_self_right/robot_node/sdk_deploy_integration.patch
   ```
2. 编译 `lite3_transfer`（板载 ARM64，ROS 2 Foxy）：
   ```bash
   source /opt/ros/foxy/setup.bash
   cd ~/lite3_interruptible_ws
   colcon build --packages-select lite3_transfer --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
3. 后续如仅修改轨迹参数 `left_roll_profile.json`，直接同步到 `~/lite3_interruptible_ws/recovery_sdk/config/` 即可，**无需重新编译**；修改 `interruptible_recovery_node.cpp` 后需先 `run_on_robot.sh stop` 再重新 `colcon build`。

---

## 四、机器狗端常用运行命令

> 现场安全前提：使用软垫、保护吊架，确认物理急停手柄可用；确保 UDP `43897` 未被其他接收进程占用（`ss -ulnp | grep 43897`）。

### 1. 常驻自动翻身开关（推荐演示与连续测试）

```bash
# 开启常驻自动翻身（默认倒地稳定 2 秒后翻身起立，持续运行至 stop/cancel 或关机）
bash ~/lite3_interruptible_ws/scripts/run_on_robot.sh start

# 可选：自定义倒地稳定等待时间（例如 1.5 秒）或同时启用小倾角趴姿直立分流
bash ~/lite3_interruptible_ws/scripts/run_on_robot.sh start --stable-wait-ms 1500
bash ~/lite3_interruptible_ws/scripts/run_on_robot.sh start --allow-prone-shortcut

# 查看开关状态（ON/OFF）及最近关键事件
bash ~/lite3_interruptible_ws/scripts/run_on_robot.sh status

# 随时关闭 / 取消（若正在翻身中会先 0 ms 切纯阻尼再释放 SDK 控制权）
bash ~/lite3_interruptible_ws/scripts/run_on_robot.sh stop
# 等价别名：
bash ~/lite3_interruptible_ws/scripts/run_on_robot.sh cancel
```

### 2. 单轮验收与吊架排查命令

单轮模式每次会在 `~/.lite3-recovery-attempts/unfinished-sdk-recovery` 写入防重入锁，确认安全后手动归档锁文件再进行下一轮：

```bash
# 归档上一轮防重入锁
bash ~/lite3_interruptible_ws/scripts/run_on_robot.sh archive-lock

# 只读观察与正常动作误报对照（不下发任何控制指令，可同步验证手柄 STOP 链路）
bash ~/lite3_interruptible_ws/scripts/run_on_robot.sh dry-run --duration 60

# 吊架微幅急停专测（仅前左小腿 FL_Knee 微幅摆动，专测 SDK 控制权与 <=20 ms 纯阻尼延迟）
bash ~/lite3_interruptible_ws/scripts/run_on_robot.sh stop-test --duration 30

# 单轮完整可中断翻身 + 零落差原厂站立（可用 --kp-scale 0.7~1.0 调整刚度）
bash ~/lite3_interruptible_ws/scripts/run_on_robot.sh recover --kp-scale 1.0 --duration 60
```

### 3. 审计日志与排障

```bash
# 查看最近生成的 JSONL 审计日志
ls -lt ~/.lite3-sdk-audits/ | head -n 10

# 查看最新常驻服务的关键事件（过滤高频 sample 行）
grep -v '"event":"sample"' /tmp/lite3-auto-recovery.latest.jsonl | tail -n 30

# 排查 43897 端口冲突
ss -ulnp | grep 43897
```

### 4. 电脑端 SSH 远程控制示例

将 `<ROBOT_HOST>`（如 `ysc@192.168.2.1`）与 `<SSH_KEY>` 替换为本机配置：

```bash
ssh -i <SSH_KEY> <ROBOT_HOST> 'bash ~/lite3_interruptible_ws/scripts/run_on_robot.sh start'
ssh -i <SSH_KEY> <ROBOT_HOST> 'bash ~/lite3_interruptible_ws/scripts/run_on_robot.sh status'
ssh -i <SSH_KEY> <ROBOT_HOST> 'bash ~/lite3_interruptible_ws/scripts/run_on_robot.sh stop'
```

---

## 五、实机与仿真验证摘要

- **C++17 `recovery_sdk` 单测**：`2/2 PASS`（`-Wall -Wextra -Wpedantic -Werror`，多线程异步 STOP → 纯阻尼延迟 `0 ms`）。
- **MuJoCo 22 项回归（`v3` 6 段轨迹 + 与 C++ 一致的 `Guard::effort_safe()` 实时校验）**：`22/22 PASS`（8 项仰倒/扰动/摩擦/软增益 + 8 项 `±59°/±75°/±90°/±110°` 左右自动选向侧倒 + 3 项早/中/起立段 `0 ms` STOP + 3 项侧墙/顶障受阻保护），标准工况护栏力矩峰值占比 `34%~40%`；重载/高阻尼/低峰值力矩 6 项极限压力对比测试较旧版 `v2` 从 `0/6` 提升至 `6/6 PASS`。
- **真机 Lite3 验证**（`lite3_interruptible_recovery`，1000 Hz `0x0906` 回调）：
  - 物理手柄 STOP 触发 `deeprcs_soft_emergency`：`damping_delay_ms = 0 ms`，`0.25 s` 内发送 `0x113` 归还手柄。
  - 左侧倒、右侧倒（`≥ 45°`）与纯仰面（`~180°`）均完成自动选向翻身与零落差原厂接管起立（`standing_pre_ack: true`，站立倾角 `< 1°`，估计高度 `~0.325 m`，起立后手柄立即可控）。
  - 针对实机连续多轮日志中暴露的两次偶发问题（`left_swing_prep` 蹭地滞后触发 40 N·m 护栏误判 `fault`、`roll_and_catch` 后过早收腿将 `67.5°` 机身反拽回仰躺），`v3` 配置已通过慢起摆（`950 ms, kp=80`）、`55/75 N·m` 虚拟护栏与新增 `350 ms` `catch_settle` 落平保持段在 MuJoCo 与压力测试中解决。
