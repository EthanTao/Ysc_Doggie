#!/usr/bin/env python3
"""由官方 Lite3 MJCF 生成展示版 MuJoCo 场景。

机器人本体使用 DeepRoboticsLab/deep_robotics_model 的官方 MJCF（原样保存在
`sim/models/lite3/mjcf/Lite3.xml`，不改写几何/质量/惯量/关节/执行器/传感器定义）。
本脚本仅在官方 MJCF 之上叠加展示用材质、三点布光、舞台地面、天空盒、
受阻场景架子与绊脚路沿，
并将碰撞体移到 group=3（只参与物理，渲染时隐藏，避免与 STL 外观重叠闪烁）。

生成结果：
    sim/models/lite3/Lite3.xml
    sim/models/lite3/Lite3_blocked.xml

用法：
    python scripts/build_scene.py
"""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "sim" / "models" / "lite3"
OFFICIAL_MJCF = MODEL_DIR / "mjcf" / "Lite3.xml"
TEXTURE_DIR = MODEL_DIR / "textures"


# --------------------------------------------------------------------------- assets
def write_floor_texture(path: Path, size: int = 512) -> None:
    """生成可平铺的深色网格地面纹理（无外部版权素材）。"""
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        if path.is_file():
            return  # 仓库已带生成好的纹理时，不强制安装 Pillow
        raise SystemExit("Pillow is required to generate the floor texture: pip install Pillow")

    rng = np.random.default_rng(7)
    base = np.array([27, 32, 40], dtype=np.float32)
    img = np.broadcast_to(base, (size, size, 3)).copy()
    img += rng.normal(0.0, 1.8, (size, size, 1)).astype(np.float32)

    im = Image.fromarray(np.clip(img, 0, 255).astype(np.uint8))
    draw = ImageDraw.Draw(im)
    for x in range(0, size, 32):
        draw.line([(x, 0), (x, size)], fill=(46, 53, 66), width=1)
    for y in range(0, size, 32):
        draw.line([(0, y), (size, y)], fill=(46, 53, 66), width=1)
    for x in range(0, size, 128):
        draw.line([(x, 0), (x, size)], fill=(70, 80, 96), width=2)
    for y in range(0, size, 128):
        draw.line([(0, y), (size, y)], fill=(70, 80, 96), width=2)
    path.parent.mkdir(parents=True, exist_ok=True)
    im.save(path)


def add_material(spec: mujoco.MjSpec, name: str, rgba, metallic=0.0, roughness=0.5,
                 specular=0.5, shininess=0.5, texture: str | None = None,
                 texrepeat=(1, 1)) -> None:
    m = spec.add_material()
    m.name = name
    m.rgba = rgba
    m.metallic = metallic
    m.roughness = roughness
    m.specular = specular
    m.shininess = shininess
    if texture is not None:
        m.textures = [texture] + [""] * 9
        m.texrepeat = list(texrepeat)
        m.texuniform = True


# --------------------------------------------------------------------------- robot
def load_official_robot(mjcf_path: Path) -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(mjcf_path))
    spec.modelname = "Lite3"

    # 外观材质：官方 Lite3 白色机身 + 深色关节 + 银灰腿杆 + 黑色脚垫。
    add_material(spec, "shell", [0.86, 0.87, 0.86, 1], 0.05, 0.42, 0.50, 0.55)
    add_material(spec, "joint_dark", [0.07, 0.08, 0.09, 1], 0.25, 0.45, 0.50, 0.50)
    add_material(spec, "metal", [0.46, 0.49, 0.53, 1], 0.0, 0.82, 0.15, 0.20)
    add_material(spec, "rubber", [0.025, 0.025, 0.028, 1], 0.0, 0.90, 0.20, 0.20)
    rgba = {
        "shell": [0.86, 0.87, 0.86, 1.0],
        "joint_dark": [0.07, 0.08, 0.09, 1.0],
        "metal": [0.46, 0.49, 0.53, 1.0],
        "rubber": [0.025, 0.025, 0.028, 1.0],
    }

    for body in spec.bodies:
        if body.name == "TORSO":
            vmat = "shell"
        elif body.name.endswith("_HIP"):
            vmat = "joint_dark"
        elif body.name.endswith(("_THIGH", "_SHANK")):
            vmat = "metal"
        elif body.name.endswith("_FOOT"):
            vmat = "rubber"
        else:
            vmat = None

        for site in body.sites:
            # 传感器参考点放到 group 3 隐藏显示，不影响传感器采样。
            site.group = 3
            site.rgba = [0.0, 0.0, 0.0, 0.0]

        for geom in body.geoms:
            if geom.name.endswith("_collision"):
                # 碰撞体放入 group 3：只参与物理，渲染时关闭该组，避免与外观重叠闪烁。
                geom.group = 3
                geom.castshadow = False
            elif geom.name.endswith("_visual") and vmat is not None:
                geom.material = vmat
                geom.rgba = rgba[vmat]

    return spec


# --------------------------------------------------------------------------- scene
def decorate_scene(spec: mujoco.MjSpec, scene: str) -> None:
    text_name = "floor_grid"
    tex = spec.add_texture()
    tex.name = text_name
    tex.type = mujoco.mjtTexture.mjTEXTURE_2D
    tex.file = "floor_grid.png"
    tex.width = 512
    tex.height = 512

    sky = spec.add_texture()
    sky.name = "sky"
    sky.type = mujoco.mjtTexture.mjTEXTURE_SKYBOX
    sky.builtin = mujoco.mjtBuiltin.mjBUILTIN_GRADIENT
    sky.rgb1 = [0.15, 0.19, 0.26]
    sky.rgb2 = [0.012, 0.016, 0.024]
    sky.width = 512
    sky.height = 512

    sky_mat = spec.add_material()
    sky_mat.name = "skybox"
    sky_mat.textures = ["sky"] + [""] * 9
    sky_mat.specular = 0.0
    sky_mat.shininess = 0.0

    add_material(spec, "floor", [1, 1, 1, 1], 0.0, 0.55, 0.35, 0.40,
                 texture=text_name, texrepeat=(12, 12))
    add_material(spec, "stage", [0.15, 0.17, 0.20, 1], 0.10, 0.48, 0.45, 0.50)
    add_material(spec, "stage_edge", [0.22, 0.26, 0.32, 1], 0.25, 0.40, 0.50, 0.50)
    add_material(spec, "shelf", [0.18, 0.19, 0.21, 0.45], 0.25, 0.45, 0.45, 0.50)
    add_material(spec, "hazard", [0.95, 0.42, 0.06, 1], 0.05, 0.45, 0.40, 0.50)

    # 替换官方 MJCF 里的默认地面材质与单灯，保留 floor geom 的名字与碰撞设置。
    for geom in spec.worldbody.geoms:
        if geom.name == "floor":
            geom.size = [12.0, 12.0, 0.1]
            geom.material = "floor"
            geom.friction = [1.0, 0.01, 0.01]

    for old_light in list(spec.worldbody.lights):
        spec.delete(old_light)

    edge = spec.worldbody.add_geom()
    edge.name = "stage_edge"
    edge.type = mujoco.mjtGeom.mjGEOM_CYLINDER
    edge.size = [1.10, 0.0003, 0.0]
    edge.pos = [0.0, 0.0, 0.0003]
    edge.material = "stage_edge"
    edge.group = 2
    edge.contype = 0
    edge.conaffinity = 0

    stage = spec.worldbody.add_geom()
    stage.name = "stage"
    stage.type = mujoco.mjtGeom.mjGEOM_CYLINDER
    stage.size = [1.04, 0.0005, 0.0]
    stage.pos = [0.0, 0.0, 0.0009]
    stage.material = "stage"
    stage.group = 2
    stage.contype = 0
    stage.conaffinity = 0

    # 三点布光 + 前侧补光：主光投影 + 冷色补光 + 背部轮廓光。
    def light(name: str, pos, direction, diffuse, specular, shadow: bool, intensity: float):
        item = spec.worldbody.add_light()
        item.name = name
        item.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
        item.pos = pos
        item.dir = direction
        item.diffuse = diffuse
        item.specular = specular
        item.castshadow = shadow
        item.intensity = intensity

    light("key", [-2.5, -2.5, 4.0], [0.45, 0.45, -1.0],
          [0.95, 0.93, 0.88], [0.5, 0.5, 0.5], True, 1.15)
    light("fill", [2.5, -1.5, 2.5], [-0.60, 0.40, -0.80],
          [0.35, 0.42, 0.55], [0.2, 0.2, 0.2], False, 0.9)
    light("rim", [0.5, 3.0, 3.0], [-0.20, -1.0, -0.70],
          [0.55, 0.62, 0.75], [0.4, 0.4, 0.4], False, 1.0)
    light("front_fill", [3.0, -1.0, 1.2], [-1.0, 0.30, -0.20],
          [0.55, 0.52, 0.48], [0.2, 0.2, 0.2], False, 0.55)

    if scene in ("blocked", "trip"):
        # 矮障碍：碰撞体仍是 0.32 x 0.32 x 0.24 的盒体，下方净空 0.15 m；
        # 外观做成可见的低矮设备架，方便展示机器人被卡住。
        shelf_body = spec.worldbody.add_body()
        shelf_body.name = "shelf"
        shelf = shelf_body.add_geom()
        shelf.name = "shelf"
        shelf.type = mujoco.mjtGeom.mjGEOM_BOX
        shelf.size = [0.16, 0.16, 0.12]
        shelf.pos = [0.0, 0.0, 0.27]
        shelf.material = "shelf"
        shelf.friction = [1.0, 0.01, 0.01]
        shelf.group = 3  # 只参与物理，外观由下面的框架表示
        shelf.castshadow = False

        def rack_box(name, size, pos, material, group=2):
            g = shelf_body.add_geom()
            g.name = name
            g.type = mujoco.mjtGeom.mjGEOM_BOX
            g.size = size
            g.pos = pos
            g.material = material
            g.group = group
            g.contype = 0
            g.conaffinity = 0
            return g

        rack_box("shelf_plate", [0.16, 0.16, 0.005], [0.0, 0.0, 0.143], "shelf")
        rack_box("shelf_top", [0.17, 0.17, 0.008], [0.0, 0.0, 0.39], "shelf")
        for i, (x, y) in enumerate(((0.17, 0.17), (0.17, -0.17), (-0.17, 0.17), (-0.17, -0.17))):
            rack_box(f"shelf_post{i}", [0.008, 0.008, 0.12], [x, y, 0.27], "shelf")
        rack_box("shelf_stripe", [0.16, 0.004, 0.006], [0.0, -0.161, 0.145], "hazard")

    if scene in ("blocked", "trip"):
        # 场景触发型摔倒：小跑中的机器狗迎面撞上 11 cm 高的路沿/门槛，
        # 由真实几何接触扰乱步态，而非直接把模型摆成翻倒姿态。
        hurdle_body = spec.worldbody.add_body()
        hurdle_body.name = "trip_hurdle"
        hurdle_body.pos = [0.75, 0.0, 0.055]

        collision = hurdle_body.add_geom()
        collision.name = "trip_hurdle_collision"
        collision.type = mujoco.mjtGeom.mjGEOM_BOX
        collision.size = [0.035, 0.60, 0.055]
        collision.friction = [1.0, 0.01, 0.01]
        collision.group = 3
        collision.castshadow = False

        visual = hurdle_body.add_geom()
        visual.name = "trip_hurdle_visual"
        visual.type = mujoco.mjtGeom.mjGEOM_BOX
        visual.size = [0.035, 0.60, 0.055]
        visual.material = "hazard"
        visual.group = 2
        visual.contype = 0
        visual.conaffinity = 0

        stripe = hurdle_body.add_geom()
        stripe.name = "trip_hurdle_stripe"
        stripe.type = mujoco.mjtGeom.mjGEOM_BOX
        stripe.size = [0.002, 0.60, 0.012]
        stripe.pos = [-0.037, 0.0, 0.025]
        stripe.material = "stage"
        stripe.group = 2
        stripe.contype = 0
        stripe.conaffinity = 0

    # 画面设置：柔和雾气、远景、阴影分辨率。
    spec.visual.global_.offwidth = 1920
    spec.visual.global_.offheight = 1080
    spec.visual.quality.shadowsize = 4096
    spec.visual.map.znear = 0.01
    spec.visual.map.zfar = 50.0
    spec.visual.map.fogstart = 4.5
    spec.visual.map.fogend = 17.0
    spec.visual.rgba.fog = [0.015, 0.020, 0.028, 1.0]
    spec.visual.rgba.rangefinder = [0.0, 0.0, 0.0, 0.0]
    spec.visual.headlight.active = 0
    spec.stat.center = [0.0, 0.0, 0.30]
    spec.stat.extent = 1.35


# --------------------------------------------------------------------------- save
def save(spec: mujoco.MjSpec, output: Path) -> None:
    for mesh in spec.meshes:
        mesh.file = Path(mesh.file).name
    spec.compiler.meshdir = str(MODEL_DIR / "meshes")
    spec.compiler.texturedir = str(TEXTURE_DIR)

    model = spec.compile()
    print(f"{output.name}: nq={model.nq} nu={model.nu} ngeom={model.ngeom} "
          f"nbody={model.nbody}")

    # 生成文件里写相对路径，仓库可整体移动。
    spec.compiler.meshdir = "meshes"
    spec.compiler.texturedir = "textures"
    spec.to_file(str(output))

    # MuJoCo 4.0.1 的 MjSpec.to_file 没有写出 material 的 texture 引用，
    # 这里补上（标准 MJCF 属性，不涉及机器人物理）。
    text = output.read_text()
    text = text.replace('<material name="floor"', '<material name="floor" texture="floor_grid"', 1)
    text = text.replace('<material name="skybox"', '<material name="skybox" texture="sky"', 1)
    if 'name="floor" texture="floor_grid"' not in text or 'name="skybox" texture="sky"' not in text:
        raise RuntimeError("failed to attach floor/skybox texture references")
    output.write_text(text)

    # 从磁盘重新加载一次，确认相对路径与生成的 XML 都有效。
    mujoco.MjModel.from_xml_path(str(output))


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mjcf", type=Path, default=OFFICIAL_MJCF)
    ap.add_argument("--out-dir", type=Path, default=MODEL_DIR)
    args = ap.parse_args()

    if not args.mjcf.is_file():
        raise SystemExit(f"official MJCF not found: {args.mjcf}")
    write_floor_texture(TEXTURE_DIR / "floor_grid.png")

    for scene in ("flat", "blocked"):
        spec = load_official_robot(args.mjcf)
        decorate_scene(spec, scene)
        name = "Lite3.xml" if scene == "flat" else f"Lite3_{scene}.xml"
        save(spec, args.out_dir / name)


if __name__ == "__main__":
    main()
