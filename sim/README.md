# 仿真资产说明

## 模型来源

- **机器人本体**：DeepRoboticsLab 官方 Lite3 MJCF，BSD-3-Clause。
  - 官方仓库：<https://github.com/DeepRoboticsLab/deep_robotics_model>
  - commit：`52352f5bbf2a30b5d02ece91900415188e7abc74`（2026-10-27）
  - 官方路径：`Lite3/mjcf/`
  - 本项目原样保存：`sim/models/lite3/mjcf/Lite3.xml` + `meshes/`
  - 详细来源、校验和与生成方式见 `models/lite3/SOURCE.md`
- **行走策略**：`policy/ppo/policy.onnx`（官方 PPO 策略，45 维观测 / 12 维动作）。
- **站立参数**：`state_machine/parameters/lite3_control_parameters.cpp` 与 `control_parameters.h`
  - `pre_height = 0.12 m`，`stand_height = 0.33 m`
  - `stand_duration = 1.5 s`（两阶段共 3 s）
  - `kp = 100`，`kd = 2.5`
  - 预站立姿态 `[0, -1.35453, 2.54948] × 4`

## 场景生成

`sim/models/lite3/Lite3.xml` 与 `Lite3_blocked.xml` 由官方 MJCF 生成，不要手改：

```bash
python scripts/build_scene.py
```

生成脚本使用 MuJoCo 官方 `MjSpec` API，在官方 MJCF 之上只叠加展示层（材质配色、
地面网格、圆形展示台、三点布光、渐变天空盒与雾），并把碰撞体与传感器 site 放入
`group=3`（物理生效、渲染隐藏），未改动任何机器人物理参数。

## 场景

| 模型文件 | 用途 | 状态 |
|---|---|---|
| `models/lite3/Lite3.xml` | 平地，确定性基准测试 | ✅ |
| `models/lite3/Lite3_blocked.xml` | 可切换的低矮设备架与 11 cm 绊脚路沿；支持受阻和动态摔倒场景 | ✅ |
| 斜坡 5° | 阈值复核 | ⬜ 待建 |

视觉设置：深色网格地面 + 圆形展示台 + 三点布光、渐变天空盒与雾；视频默认
1280×720、45° 前侧 3/4 视角。`videos/` 中的演示按此视角录制。

姿态注入与站立轨迹的实现见 `src/doggie/io/sim_backend.py`：

- `place_fall(kind)`：按 `config/recovery.yaml` 的 profile 注入，`mj_geomDistance` 精确留出设定间隙，支持前翻/后翻/左右侧翻/趴地；可选 `jitter` + `seed` 做随机化压力测试；
  - `nominal`（默认）：前翻 +90°、后翻 -90° + 4 rad/s 回摆、侧翻 ±65° 蜷腿；
  - `full_flip`（可选附加实验）：带初始角速度与离地高度，累计翻转 >400°（实际经过仰面）后落回趴地；
  - `prone` 在两种 profile 中都是 5 mm 间隙的静态趴地（受阻场景用）；
- `place_trip()`：从站姿启动官方小跑策略，沿途真实碰撞 11 cm 路沿；不直接注入翻倒姿态，摔倒后可进入正常恢复流程；
- RL 安全限幅：只有机器人与路沿实际发生接触时才启用，对策略目标做一阶低通、限幅策略观测角速度并临时提高关节阻尼；正常平地步态与官方权重不变；
- `StandUp`：按官方两阶段三次样条 + IK 结构复刻（`pre_height_m`→`stand_height_m`，每阶段 `duration_per_phase_s`，kp=100/kd=2.5）；不是直接运行官方控制器。

## 注意

- 姿态注入前先检查关节限位与初始穿透，否则 MuJoCo 会直接弹飞。
- `full_flip` 是可选附加实验，目标是“完整翻过 360° 后落回趴地”。若参数/扰动导致最终静止在仰面，官方 `LieDown + StandUp` 仍无法自翻正，FSM 会按边界策略锁存 `E_UNRECOVERABLE_POSE`；背部静止着地后的自翻正研究见 `docs/11-handoff.md`。
- 所有试验必须固定种子、初始状态与场景几何，保证 trial 可复现；动态路沿场景复用固定速度指令，起身后保持站立以免再次撞击。
- 仿真结果不能直接等同于实机；实机阈值需重新标定并在报告中说明差异。
- 若在 MuJoCo 官方查看器里看到半透明碰撞体，请关闭 geom group 3；本项目脚本默认已关闭。
