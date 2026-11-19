# Lite3 MJCF 资产来源

- **原始版权**：DeepRoboticsLab，BSD-3-Clause（见 `LICENSE.txt`）。
- **官方仓库**：<https://github.com/DeepRoboticsLab/Lite3_rl_deploy>
  - commit: `1c4a8f14c9c531903fa5a26783e3f9b99d1039d9`
  - 模型目录：`Lite3_description/lite3_mjcf/`
- **精简版来源**：<https://github.com/Yuncheng-255/lite3-mujoco-sim>
  - commit: `03d39693a2d8fcb8a02ba06de2302258f27ff7ce`
  - 路径：`models/lite3/`
  - 说明：把官方网格合并为 torso / hip / thigh / shank 四个 STL，保留关节名、限位、质量、惯量与执行器范围，便于快速加载与批量仿真。
- **本项目改动**：仅将模型放入 `sim/models/lite3/`，未修改 XML 的物理参数与关节定义。

使用与再分发时请保留 `LICENSE.txt` 与本说明。
