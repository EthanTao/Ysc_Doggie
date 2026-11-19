# 仿真资产说明

## 模型来源

- **MJCF / meshes**：DeepRobotics 官方 Lite3 模型，BSD-3-Clause。
  - 官方仓库：<https://github.com/DeepRoboticsLab/Lite3_rl_deploy>
  - 模型路径：`Lite3_description/lite3_mjcf/`
  - 本项目使用精简版 `Lite3.xml`（4 个网格：torso / hip / thigh / shank），保留官方关节名、限位、质量与惯量。
- **行走策略**：`policy/ppo/policy.onnx`（官方 PPO 策略，45 维观测 / 12 维动作）。
- **站立参数**：`state_machine/parameters/lite3_control_parameters.cpp`
  - `pre_height = 0.12 m`，`stand_height = 0.33 m`
  - `stand_duration = 2 s`（两阶段共 4 s）
  - `kp = 100`，`kd = 2.5`
  - 预站立姿态 `[0, -1.35453, 2.54948] × 4`

复制模型时**必须同时保留** `LICENSE.txt` 与来源说明。若后续替换为完整官方 meshes，在 `models/lite3/SOURCE.md` 记录 commit hash。

## 场景

| 模型文件 | 用途 | 状态 |
|---|---|---|
| `models/lite3/Lite3.xml` | 平地，正式 9 次测试 | ✅ |
| `models/lite3/Lite3_blocked.xml` | 趴地 + 上方矮障碍（模拟跌入设备底座下方），起身受阻 3 次 | ✅ |
| 斜坡 5° | 阈值复核 | ⬜ 待建 |

姿态注入与站立轨迹的实现见 `src/doggie/io/sim_backend.py`：

- `place_fall(kind)`：`mj_geomDistance` 精确留 5 mm 间隙放置，支持前翻/后翻/侧翻（±65°+前向动量 1.5 rad/s）/趴地；
- `StandUp`：官方两阶段三次样条 + IK（0.12→0.33 m，2 s + 2 s，kp=100/kd=2.5）。

## 注意

- 姿态注入前先检查关节限位与初始穿透，否则 MuJoCo 会直接弹飞。
- 所有场景必须固定种子、固定初始状态，保证 trial 可复现。
- 仿真结果不能直接等同于实机；实机阈值需重新标定并在报告中说明差异。
