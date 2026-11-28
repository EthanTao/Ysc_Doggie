# Lite3 可中断 SDK 全姿态自翻正恢复使用与命令手册

> 对应独立扩展目录：[experiments/sdk_self_right/](../experiments/sdk_self_right/)  
> 任务二实机验收数据与标定图表：[experiments/sdk_self_right/report/task2_real_data_report.md](../experiments/sdk_self_right/report/task2_real_data_report.md)  
> 约束说明：与主路径 `src/doggie/` 完全隔离，不改动 `src/doggie/` 正式 9/9 矩阵结论，也不修改机器狗出厂 `~/jy_exe` 与 `~/sdk_service`；实机端独立部署在 `~/lite3_interruptible_ws`，开机默认关闭。

---

## 一、工作原理与安全特性

- **默认开机关闭**：不安装任何开机自启服务，开机时完全不接管端口或控制权；仅在显式执行 `run_on_robot.sh start` 后保持运行，直到机器狗关机或执行 `stop`/`cancel`。
- **待命阶段完全只读**：处于 `observing` 阶段时不发送任何运动 UDP 指令，物理遥控手柄可正常控制站立、行走、趴下与爬坡。
- **全姿态跌倒检出与自动分流（仰倒 / 左右侧倒 / 前摔 / 后摔）**：
  - **侧倒/仰倒（`tilt ≥ 35°` 无条件触发）**：驻留 `300 ms` 确认后先纯阻尼静置 **2.0 秒**（默认 `--stable-wait-ms 2000`，支持 `300..30000 ms`），按侧倾方向自动选择**左翻**（`left_roll`）或**镜像右翻**（`right_roll`），执行 **6 段五次多项式翻身**（`lite3_left_roll_v3`：`supine_tuck` → `left_swing_prep` → `left_knee_push` → `roll_and_catch` → `catch_settle` → `prone_align`，`4400 ms`）。
  - **前摔机首着地 / 后摔坐倒（`18° ≤ tilt < 35°` 且 `est_height_m < 0.19 m`）**：高度门限 `< 0.19 m` 彻底避免 20°~25° 斜坡误触发（实机 `681` 帧斜坡样本 **0 误报**）；引入 `post_fall_latched` 回差锁存解决前/后摔峰值达 `36°~66°` 后在阻尼下滑平至 `1.4°~11.3°`（肚皮贴地且腿曲折/反撇）丢事件的问题（必须直立 `tilt ≤ 15° && h ≥ 0.25 m` 持续 `500 ms` 或收到 STOP 才解锁）。分别按俯仰符号执行 `front_restore_v1`（`1450 ms`）或 `back_restore_v1`（`1500 ms`）柔和整腿复位。
- **0~1 ms 纯阻尼急停 + 单侧障碍自动换侧 + 二次失败永久锁停**：
  - 同一互斥锁内串行化 STOP 锁存与 UDP 发送边界，任意时刻收到物理手柄 STOP（`deeprcs` Soft Emergency）、终端取消、受阻停滞、力矩护栏越界、反馈断流（`>200 ms`）或指令过期（`>20 ms`），实机实测 **`0~1 ms`** 切纯阻尼（`kp = 0, kd = 2.5`）。
  - **单侧受阻自动换侧**：翻身第 1 次因障碍受阻停机后，输出 `[RETRY]` 中文提示并在纯阻尼下等待 `2.0 s`（`--retry-wait-ms 2000`），随后自动从**另一侧**镜像翻身恢复（实机 **5/5** 成功）。
  - **二次失败永久安全锁停**：若第 2 次换侧仍受阻（双侧/顶障，实机 **3/3**）或收到人工 STOP，立即输出 `[ALERT / SAFE_FAULT]` / `[ALERT / STOP]` 中文醒目告警、`0.25 s` 内发送 `0x113` 归还手柄控制权并进入永久安全锁定（**必须人工扶正并直立站稳 3.0 秒**后才重新待命，绝不无限重试）。
- **零落差直接起立并恢复手柄控制**：翻正/整腿对齐 `PRE_POSE` 并确认趴稳 `0.5 s` 后，在 SDK 仍托住机身时预发原厂站立命令 `0x21010202`，监听到 `jy_exe` 输出 `Standing up, please wait...`（~5 ms）后瞬间发 `0x113` 无缝交回原厂力控站立，起立完成后手柄立即可操控行走，站稳 `0.5 s` 后自动重新待命。

---

## 二、离线单元测试与本地 MuJoCo 物理仿真（34 项全量回归）

仿真直接复用本仓库 [sim/models/lite3/mjcf/Lite3.xml](../sim/models/lite3/mjcf/Lite3.xml)（脚本内自动按 Lite3 实机硬件关节限幅 `HipY ∈ [-3.50, 0.320]` 运行），并与 C++17 `recovery_sdk` 保持完全一致的五次多项式插值、前/后/左/右/仰倒统一选择器、`Guard::effort_safe()` 虚拟力矩护栏、纯阻尼急停、停滞检测与单侧受阻等 2 s 换侧状态机。

在仓库根目录执行（如使用独立虚拟环境或 conda，将 `python3` 替换为对应解释器即可；去掉 `--view` 即为无窗口快速跑出 JSON 结果，追加 `--speed 0.5` 可半速慢放）：

```bash
SIM=experiments/sdk_self_right/recovery_sdk/sim/mujoco_roll_eval.py
```

### 1. 组合 3D 窗口演示

```bash
# 综合 8 场景（仰倒/左右侧倒/前摔/后摔/中途STOP/单侧墙换侧/双墙锁停）
python3 "$SIM" --view --scenario demo_all

# 多角度左右侧倒（±59° / ±75° / ±90° / ±110° 自动选向）
python3 "$SIM" --view --scenario demo_sides

# 前/后摔倒专项演示（前趴折腿 / 前摔大俯仰 / 后坐倒 / 后摔大仰角）
python3 "$SIM" --view --scenario demo_pitch

# 障碍物保护专项演示（单侧墙2s换侧成功 / 右墙换左侧 / 双墙锁停 / 顶障锁停）
python3 "$SIM" --view --scenario demo_obstacle
```

### 2. 单项仿真场景清单（共 34 项）

- **仰倒、扰动、摩擦与软增益（8 项）**：`nominal_180`、`roll_165`、`roll_195`、`pitch_p10`、`pitch_m10`、`low_fric_0.6`、`high_fric_1.4`、`soft_kp_0.7`
- **多角度左右侧倒自动左/右翻（8 项）**：`side_left_59`、`side_right_59`、`side_left_75`、`side_right_75`、`side_left_90`、`side_right_90`、`side_left_110`、`side_right_110`
- **前摔（机首着地折腿）与后摔（坐倒）恢复（8 项）**：`front_bent_prone`、`front_nose_steep`、`front_asym_tangled`、`front_soft_kp`、`back_sit`、`back_sit_steep`、`back_sit_asym`、`back_soft_kp`
- **早/中/前摔/后摔/起立段中途 STOP 0 ms 急停（5 项）**：`stop_early`、`stop_mid`、`stop_front_mid`、`stop_back_mid`、`stop_stand`
- **单侧墙受阻 → 等 2 s 换另一侧成功（2 项）**：`blocked_side_wall`、`blocked_right_wall`
- **双侧墙 / 低矮顶障 / 全宽顶障 → 永久安全锁停（3 项）**：`blocked_both_walls`、`blocked_overhead`、`blocked_stand_ceil`

### 3. 自定义任意角度 / 摩擦 / 刚度 / STOP 时刻

```bash
# 例：右侧倒 -82°、俯仰 +5°、半速慢放观看自动镜像右翻
python3 "$SIM" --view --scenario custom --roll -82 --pitch 5 --speed 0.5

# 例：左侧倒 +100°、1.5 秒注入 STOP 测试中途阻尼
python3 "$SIM" --view --scenario custom --roll 100 --stop-ms 1500
```

### 4. 无头批量回归测试（34 项 MuJoCo + C++17 门控单测）

```bash
# 一次性无头运行全部 34 项 MuJoCo 仿真（预期 34/34 PASS）
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
   cp -r <repo>/experiments/sdk_self_right/recovery_sdk ~/lite3_interruptible_ws/
   cp -r <repo>/experiments/sdk_self_right/scripts ~/lite3_interruptible_ws/
   mkdir -p ~/lite3_interruptible_ws/src/lite3_transfer/src
   cp <repo>/experiments/sdk_self_right/robot_node/interruptible_recovery_node.cpp \
      ~/lite3_interruptible_ws/src/lite3_transfer/src/
   cd ~/lite3_interruptible_ws
   git apply <repo>/experiments/sdk_self_right/robot_node/sdk_deploy_integration.patch
   ```
2. 编译 `lite3_transfer`（板载 ARM64，ROS 2 Foxy）：
   ```bash
   source /opt/ros/foxy/setup.bash
   cd ~/lite3_interruptible_ws
   colcon build --packages-select lite3_transfer --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
3. 后续如仅修改轨迹 JSON（`left_roll_profile.json`、`front_restore_profile.json`、`back_restore_profile.json`），直接同步到 `~/lite3_interruptible_ws/recovery_sdk/config/` 即可，**无需重新编译**；修改 `interruptible_recovery_node.cpp` 后需先 `run_on_robot.sh stop` 再重新 `colcon build`。

---

## 四、机器狗端常用运行命令

> 现场安全前提：使用软垫、保护吊架，确认物理急停手柄可用；确保 UDP `43897` 未被出厂 `lite3_sdk_service` 占用（若被占用先执行 `sudo systemctl stop lite3_sdk_service`）。

### 1. 常驻自动恢复开关（推荐演示与连续测试）

```bash
# 开启常驻全姿态自动恢复（默认倒地稳定 2 秒，单侧受阻等 2 秒换侧）
bash ~/lite3_interruptible_ws/scripts/run_on_robot.sh start

# 可选：自定义稳定等待或换侧等待时间
bash ~/lite3_interruptible_ws/scripts/run_on_robot.sh start --stable-wait-ms 2000 --retry-wait-ms 2000

# 查看开关状态（ON/OFF）、中文告警与最近关键事件
bash ~/lite3_interruptible_ws/scripts/run_on_robot.sh status

# 随时关闭 / 取消（若正在动作中会先 0 ms 切纯阻尼再释放 SDK 控制权）
bash ~/lite3_interruptible_ws/scripts/run_on_robot.sh stop
# 等价别名：
bash ~/lite3_interruptible_ws/scripts/run_on_robot.sh cancel
```

### 2. 单轮验收与吊架排查命令

```bash
bash ~/lite3_interruptible_ws/scripts/run_on_robot.sh archive-lock
bash ~/lite3_interruptible_ws/scripts/run_on_robot.sh dry-run --duration 60
bash ~/lite3_interruptible_ws/scripts/run_on_robot.sh stop-test --duration 30
bash ~/lite3_interruptible_ws/scripts/run_on_robot.sh recover --kp-scale 1.0 --duration 60
```

### 3. 审计日志与实机报告复现

```bash
# 查看机器狗实时中文告警输出
tail -f /tmp/lite3-auto-recovery.out

# 查看最新常驻服务的结构化 JSONL 关键事件
grep -v '"event":"sample"' /tmp/lite3-auto-recovery.latest.jsonl | tail -n 30

# 将真机 JSONL 日志自动生成任务二表 7 报告与 4 张高清图表
python3 experiments/sdk_self_right/recovery_sdk/sim/generate_real_report.py \
  --audit-dir <downloaded_jsonl_dir> \
  --out-dir experiments/sdk_self_right/report
```

---

## 五、实机与仿真验收摘要（详见实机数据包）

完整逐次数据表、105,913 帧非跌倒标定矩阵与 4 张高清实机图见：[task2_real_data_report.md](../experiments/sdk_self_right/report/task2_real_data_report.md)。

- **C++17 `recovery_sdk` 单测**：x86_64 与板载 ARM64 均为 **`2/2 PASS`**（`-Wall -Wextra -Wpedantic -Werror`，多线程异步 STOP 延迟 `0 ms`）。
- **MuJoCo 34 项全量物理回归**：**`34/34 PASS`**（8 仰倒扰动 + 8 左右侧倒 + 8 前/后摔倒 + 5 中途 STOP + 2 单侧墙换侧 + 3 双侧/顶障永久锁停）；6 项极端重载压力对比测试较旧版 `v2` 从 `0/6` 提升至 **`6/6 PASS`**。
- **真机 Lite3 实测验证**（`lite3_interruptible_recovery`，1000 Hz `0x0906` 回调）：
  - **26 次实机连续成功起身**（前摔 5 + 后摔 2 + 左右侧倒 8 + 仰倒 6 + 单侧受阻换侧 5），动作平均耗时 **`6.44 s`**，含 2 s 安全静置总耗时平均 **`10.23 s`**（最大 `13.62 s` ≪ 优秀档 `15 s`）。
  - **单侧障碍自动换侧 `5/5` 成功**（受阻 `0~1 ms` 切纯阻尼 → 等 2 s → 换侧起身）。
  - **双侧/全向受阻永久锁停 `3/3` 成功**（阻尼延迟 `0 ms`，输出 `[ALERT / SAFE_FAULT]`，绝不无限重试）。
  - **物理手柄 STOP 急停 `0 ms` 切纯阻尼**，`0.25 s` 内发送 `0x113` 归还控制权。
  - **非跌倒零误报标定**：覆盖 `105,913` 帧实机非跌倒样本（站立/行走 `32,926` 帧、遥控趴下 `72,306` 帧、20°~25° 斜坡 `681` 帧），在 `18° + h < 0.19 m` / `35°` 判据下 **总误报率 `0.00%`、斜坡误报率 `0.0%`、真跌倒检出率 `100.0% (37/37)`**。
