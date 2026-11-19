# Lite3 模型来源与生成方式

## 官方 MJCF（机器人本体唯一来源）

- **版权**：DeepRoboticsLab，BSD-3-Clause（见 `LICENSE.txt`）。
- **官方仓库**：<https://github.com/DeepRoboticsLab/deep_robotics_model>
- **commit**：`52352f5bbf2a30b5d02ece91900415188e7abc74`
  （2026-10-27，Correct Lite3 sensor mount and optical frame orientations）
- **官方路径**：`Lite3/mjcf/Lite3.xml` 与 `Lite3/mjcf/meshes/`
- **本项目保存位置**：
  - `sim/models/lite3/mjcf/Lite3.xml`（原样保存，未改写；SHA-256
    `45ddd5323aacf125635c0018fe77cc7b68b060721735896734a3601a6a2b33fa`）
  - `sim/models/lite3/meshes/`（官方 `Lite3/mjcf/meshes/`，四个 STL）

## 从官方 MJCF 生成展示场景

`scripts/build_scene.py` 使用 MuJoCo 官方 `MjSpec` API 读取原样保存的
`sim/models/lite3/mjcf/Lite3.xml`，在不改动机器人物理参数的前提下叠加展示层：

| 叠加项 | 说明 |
|---|---|
| 外观材质 | 白色机身 + 深灰关节 + 银灰腿杆 + 黑色脚垫（贴合真机配色） |
| 地面 / 舞台 / 三点布光 / 天空盒 / 雾 | 展示画面与阴影；保留 `floor` 平面摩擦设置 |
| 碰撞体与传感器 site 放入 `group=3` | 物理与传感器正常生效；渲染时隐藏，避免与 STL 外观重叠闪烁 |
| `blocked` 场景低矮设备架 | 下方净空 0.15 m 的盒形碰撞体 + 可见的设备架外观 |

**没有做的改动**：没有修改官方 MJCF 的 link/joint 名称、轴、限位、质量、惯量、
STL 碰撞几何、执行器范围或 contact exclude。

### 生成命令

```bash
python scripts/build_scene.py
```

输出：

- `sim/models/lite3/Lite3.xml`（平地场景）
- `sim/models/lite3/Lite3_blocked.xml`（起身受阻场景）

这两个 XML 是生成物，不要手改；改场景请改 `scripts/build_scene.py` 后重新生成。

使用与再分发时请保留 `LICENSE.txt` 与本说明。
