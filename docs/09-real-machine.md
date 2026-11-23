# 实机验收指南（Lite3 / ROS2）

> 对应报告 6.1–6.3。主路径仿真结论（常规前/后/侧翻 9/9 检出、9/9 先安全后起身、
> 9/9 起身、3/3 受阻锁存）已在 MuJoCo 中复现；本文只讲**搬到真机上还差什么、怎么验、
> 哪些指标会变**。完整 360° `full_flip` 是可选附加实验，不是实机默认路径。

## 1. 当前实现状态

| 组件 | 位置 | 状态 |
|---|---|---|
| 摔倒检测 / 恢复状态机 | `src/doggie/core/` | 纯逻辑，与平台无关，实机零改动复用 |
| 仿真后端 | `src/doggie/io/sim_backend.py` | 完成，固定步长调度 |
| **实机后端** | `src/doggie/io/ros2_backend.py` | **已实现**（话题收发、时间戳、高度估计、状态广播、复位服务） |
| 消息解析层 | `src/doggie/io/ros2_convert.py` | 字段别名兼容，可离线单测 |
| 腿部正运动学 | `src/doggie/core/kinematics.py` | 杆长引用 `core.params`，与官方 MJCF 对拍误差 < 1 mm |
| 安全看门狗 | `src/doggie/core/safety.py` | 心跳超时 + 阻尼确认，接入 `Supervisor` |
| 实机启动脚本 | `scripts/run_real.py` | 默认**只读干跑**，`--enable-actuation` 才下发指令 |
| 恢复 profile | `run_real.py --profile` | `nominal`（默认，静置窗口 3 s）/ `full_flip`（可选附加实验，5 s）；只影响摔倒后参数 |
| 人工恢复接口 | `/doggie/recover` (`std_srvs/srv/Trigger`) | 对**当前已发生姿态**触发一次官方恢复，不主动制造摔倒 |
| ROS2 链路自检 | `scripts/selftest_ros2.py` | 单进程真实 rclpy/DDS 回环，上机前 1 分钟跑通 |

`Supervisor` 已不再引用 `MjModel`：仿真按步计数调度，实机按时间戳调度，
两条路径共用同一套决策逻辑（`tests/test_safety_and_supervisor.py` 用无 MuJoCo 的
假后端验证了完整「判定 → 阻尼 → 收腿 → 起身 → 验证」闭环）。

## 2. 上机前必须现场核对的 3 件事

这三项因驱动版本而异，代码无法替你猜：

1. **消息类型**（`config/recovery.yaml` → `ros2.imu_type` / `joints_type`）
   默认按报告表 6-1 的 `drdds/msg/ImuData`、`drdds/msg/JointsData`。
   ```bash
   ros2 topic info /IMU_DATA -v
   ros2 topic info /JOINTS_DATA -v
   ```
   如果实际是 `sensor_msgs/msg/Imu`，改 `imu_type` 即可（解析层两种都支持）。

2. **关节顺序**（`ros2.joint_order`）
   默认按仿真顺序 `FL, FR, HL, HR × (HipX, HipY, Knee)`。用
   ```bash
   ros2 topic echo /JOINTS_DATA --once
   ```
   对照实际字段确认；顺序错会导致高度估计和 `S2_ASSESS` 分流全部错误。

3. **状态切换话题**（`ros2.state_cmd_topic`）
   **默认为空 = 只读干跑**，不会动任何关节。确认官方状态机（`kJointDamping` /
   `kLieDown` / `kStandUp`）的切换方式后填入话题名；若驱动要求的是 SDK 调用而非
   话题，把 `Ros2Backend._publish_state()` 换成对应调用即可（单点改动）。

可选但强烈建议：配 `state_feedback_topic`（状态回读）与 `state_feedback_type`，
否则 `damping_ack_timeout_s` 看门狗无法生效（代码会显式跳过，不会误报）。回读消息
默认按 `std_msgs/msg/String` 的 `data` 字段与 `state_names` 比较；若官方用自定义
State 消息，请同时确认字段解析逻辑。

人工恢复服务（`ros2.recover_service`，默认 `/doggie/recover`）用于机器人在外部作用下
已经摔倒、但检测器未触发或操作员希望立即恢复的场景。调用后状态机仍**先进入 S1
纯阻尼**、等静止后再 S2 评估，不会跳过安全流程；服务本身不会让机器人翻转。

## 3. 高度估计：实机没有高度真值时怎么办

实机只有 IMU + 关节角，`kinematics.base_height_from_q()` 用官方 MJCF 的腿部几何
正解四个足端位置，再把重力方向投影到机身系取最低足端距离：

$$
\hat h = \max\left(0,\ \min_i\left(-\mathbf{p}^{b}_{\text{foot},i}\cdot \mathbf{n}_b\right)\right),
\qquad \mathbf{n}_b = \mathbf{R}_{wb}^{\top}\mathbf{e}_z
$$

与仿真真值对拍（静止 2.5 s 后）：

| 姿态 | 仿真 `qpos[2]` | FK 估计 | 偏差 |
|---|---|---|---|
| 站立（q_stand） | 0.346 | 0.324 | −0.022 |
| 趴地（Q_PRE 塌陷后） | 0.054 | 0.029 | −0.025 |
| 侧翻 90°（落定） | 0.108 | 0.084 | −0.024 |

偏差主要来自足端球半径与接触穿透，方向是**低估**（偏安全）。关键结论：
趴地估计 0.029 m 远低于 `height_min_m = 0.10`，站立 0.324 m 远高于
`upright_height_m = 0.25`，两个门限都有 ≥ 7 cm 裕度。

上电后先做 2 s 静态零偏标定（`--calibrate 2`），扣除 IMU 安装偏置。

## 4. 四阶段验收 SOP（与报告 6.3 一致）

| 阶段 | 目的 | 命令 | 通过标准 |
|---|---|---|---|
| 一 | 话题/姿态/零偏 | `python scripts/run_real.py --duration 60 --calibrate 2` | 终端 roll/pitch 与手持倾角一致；30° 不触发 |
| 二 | 只检不立：验证阻尼切入 | 同上，人工缓推至 55° 保持 >0.3 s、80° 观察快速通道 | 打印 `FallEvent`，`detect_delay_s` ≤ 0.35 s（80° 时 ≈0.15 s）；**若已配 `state_cmd_topic`，此阶段仍建议先用干跑** |
| 三 | 完整恢复（吊架 + 软垫） | `python scripts/run_real.py --duration 300 --enable-actuation` | `S1` 阻尼 → `S2` 分流 → (`S2B`) → `S3` 起身 → `S4` 1 s → `S0`；打印「先安全后起身」为真 |
| 四 | 受阻锁存 | 同阶段三，软垫轻压背部限制抬升 < 0.15 m | 2 次尝试后 `S5_SAFE_FAULT` + `E_STANDUP_NOT_UPRIGHT`；`ros2 service call /doggie/reset std_srvs/srv/Trigger` 后回到 `S0` |

现场必须：≥5 cm 软垫、龙门吊架（站立松弛 / 趴地留 10 cm 余量）、三人分工、
硬件急停可用、`config/recovery.yaml` 里 `allow_real_fall_test: false` 保持不变。

### 4.1 可选 360° 全翻 profile（不主动制造摔倒）

默认实机路径是 `nominal`（常规 65–90° 摔倒/受控倾斜）。`full_flip` 只作为附加实验：
仿真中由初始角速度/离地高度构造，真机不能由代码复制这一点，也不应主动把机器人抛翻。
实机的用法是：**外部原因已经造成完整翻转后**，用同一套检测与恢复逻辑处理：

```bash
# 干跑观察：只读，不下发动作；适合先确认姿态/高度估计
python scripts/run_real.py --duration 120 --profile full_flip

# 受控恢复：必须已配 state_cmd_topic；会二次确认
python scripts/run_real.py --duration 300 --profile full_flip --enable-actuation

# 若检测器没触发（例如机器人被人工搬到趴地），操作员手动触发恢复
ros2 service call /doggie/recover std_srvs/srv/Trigger
```

- `full_flip` 把 `fsm.settle_max_s` 从 3 s 放宽到 5 s，给落地弹跳/滚动更多时间；
- 检测器有高速翻转通道（姿态越限 + |ω| ≥ 5 rad/s 持续 0.05 s），仿真中全翻在
  0.05 s 内确认；实机需用 `/IMU_DATA` 实测角速度复核该阈值；
- 目标仍是**翻转后落回趴地**。若最终静止在仰面，`S2_ASSESS` 会锁存
  `E_UNRECOVERABLE_POSE`，不要用“仰面静置后要求自恢复”作为验收条件；
- 操作员必须使用软垫/吊架并保持急停可用；`/doggie/recover` 不会替代这些保护。

## 5. 指标口径：验收时看 `detect_delay_s`

- `latency_s`（仿真矩阵用的）是「从本次运行开始到确认」的时间，跑得越久越大，
  实机常驻运行时没有意义。
- `detect_delay_s`（本次新增）是「首个判据越限 → 确认」的耗时，才是与
  「时延 ≤ 2 s」对应的量。主路径 nominal 仿真中侧翻/前翻/受阻约为
  **0.305 / 0.155 / 0.320 s**；可选 full_flip 由高速翻转通道在 0.05 s 确认。

采样率变化会同时影响低通滤波与驻留判据的采样点密度：若 `/IMU_DATA` 是 100 Hz，
用 `run_real.py --det-hz 100` 覆盖即可；该参数会同步 `detector.rate_hz`（低通系数），
不需要再手改配置。默认使用 `config/recovery.yaml` 的 `detector.rate_hz`。

## 6. 已知差距与验收风险（务必提前和老师说明）

1. **纯仰面（背部静止着地）仍然起不来**。主路径 nominal 与可选 `full_flip` 都要求
   最终落回趴地；真机若最终静止在仰面，`StandUp`/`LieDown+StandUp` 都无法自翻正。
   当前策略会识别 `|roll| ≥ 120°` 判 `E_UNRECOVERABLE_POSE` 并安全锁存，**这是设计上的
   主动拒止，不是回归**。验收时不要用「静置仰面后要求自恢复」作为通过条件；该目标的
   研究交接见 `docs/11-handoff.md`。
2. **摔倒过程与注入不同**。仿真 nominal 是静态 65–90° 注入，full_flip 是固定初始角速度/
   离地高度构造；真机摔倒是带滑移、任意轴、能量耗散的过程。判定阈值裕度大
   （正常包络 1.7° vs 50° 门限），但高速翻转通道的 `|ω| ≥ 5 rad/s` 必须用现场
   `/IMU_DATA` 复核；检测器已锁存 + 回差 + 事件起点保持，预期不会因翻转中反复穿越
   阈值而重复触发，但仍需实测。
3. **足端触地**：无足端力话题时用几何估计（`feet_contact_from_q`），仅用于记录；
   当前判据不依赖 `feet_contact`，不影响判定结果。
4. **`E_DAMPING_NO_ACK` 依赖状态回读**。没配 `state_feedback_topic` 时该看门狗
   自动关闭，此时「阻尼是否真的生效」只能靠现场观察腿部是否变软。
5. **通信中断兜底**：`estop_on_comm_loss` 只在 `E_SENSOR_STALE` 时触发，且需要有
   `estop_topic` 或官方急停链路；没有的话只会打印告警并保持阻尼指令重发。

## 7. 故障码

| 码 | 触发条件 | 恢复方式 |
|---|---|---|
| `E_STANDUP_NOT_UPRIGHT` | 起身轨迹跑完仍不直立 | 2 次后锁存，人工复位 |
| `E_STANDUP_TIMEOUT` | 起身超 20 s | 同上 |
| `E_VERIFY_TIMEOUT` | 直立后 5 s 内没稳住 1 s | 同上 |
| `E_UNRECOVERABLE_POSE` | 仰翻/倒立，超出 StandUp 包络 | 人工翻正后 `/doggie/reset` |
| `E_RETRY_EXHAUSTED` / `E_TOO_MANY_RECOVERIES` | 重试或频次超限 | 人工复位 |
| `E_SENSOR_STALE` | 观测超过 `backend_heartbeat_timeout_s` 未更新 | 检查通信后复位 |
| `E_DAMPING_NO_ACK` | 切阻尼后超过 `damping_ack_timeout_s` 未确认 | 检查状态回读/底层后复位 |

## 8. 常用命令

```bash
# 0) 上机前先跑 ROS2 回环自检（不需要机器人，1~15 s）
source /opt/ros/jazzy/setup.bash
python scripts/selftest_ros2.py --duration 15

# 只读干跑（默认），验证话题与姿态
python scripts/run_real.py --duration 60

# 可选全翻 profile（仍只读）
python scripts/run_real.py --duration 120 --profile full_flip

# 静态零偏
python scripts/run_real.py --duration 30 --calibrate 2

# 使能运动指令（会二次确认；默认 nominal）
python scripts/run_real.py --duration 300 --enable-actuation

# 可选：360° 全翻附加实验
python scripts/run_real.py --duration 300 --enable-actuation --profile full_flip

# 外部摔倒后人工触发一次恢复（不制造摔倒）
ros2 service call /doggie/recover std_srvs/srv/Trigger

# 人工复位
ros2 service call /doggie/reset std_srvs/srv/Trigger

# 查看状态广播
ros2 topic echo /doggie/status

# 离线回归（不需要 ROS2 / 不开窗口）
python -m pytest -q
```
