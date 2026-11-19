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
| `models/lite3/Lite3.xml` | 平地，正式 9 次测试 | ✅ |
| `models/lite3/Lite3_blocked.xml` | 趴地 + 低矮设备架（下方净空 0.15 m），起身受阻 3 次 | ✅ |
| 斜坡 5° | 阈值复核 | ⬜ 待建 |

视觉设置：深色网格地面 + 圆形展示台 + 三点布光、渐变天空盒与雾；视频默认
1280×720、45° 前侧 3/4 视角。`videos/` 中的演示按此视角录制。

姿态注入与站立轨迹的实现见 `src/doggie/io/sim_backend.py`：

- `place_fall(kind)`：`mj_geomDistance` 精确留 5 mm 间隙放置，支持前翻（pitch +90°）/后翻（pitch -90° + 回摆 4.0 rad/s）/侧翻（±60° + 前向动量 3.0 rad/s）/趴地；
- `StandUp`：按官方两阶段三次样条 + IK 结构复刻（0.12→0.33 m，1.5 s + 1.5 s，kp=100/kd=2.5）；不是直接运行官方控制器。

## 注意

- 姿态注入前先检查关节限位与初始穿透，否则 MuJoCo 会直接弹飞。
- 所有场景必须固定种子、固定初始状态，保证 trial 可复现。
- 仿真结果不能直接等同于实机；实机阈值需重新标定并在报告中说明差异。
- 若在 MuJoCo 官方查看器里看到半透明碰撞体，请关闭 geom group 3；本项目脚本默认已关闭。
