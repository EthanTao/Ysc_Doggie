#!/usr/bin/env python3
"""生成技术报告与答辩 PPT 所需的高清 MuJoCo 仿真关键帧与序列长图。

输出目录：docs/report/sim_shots/
  - 各场景单帧高清图（1280x720，带状态/姿态角/时间 HUD 标注）
  - strip_front.png            前翻恢复 4 阶段序列长图
  - strip_side_left.png        侧翻（含 S2B 收腿整姿）恢复 4 阶段序列长图
  - strip_back.png             后翻（回摆落趴地）恢复 4 阶段序列长图
  - strip_blocked.png          矮架受阻（2 次重试 -> S5_SAFE_FAULT）4 阶段序列长图
  - strip_supine_boundary.png  四脚朝天（仰翻 180° -> S2 直接拒止 E_UNRECOVERABLE_POSE）边界序列图
  - grid_overview.png          全场景对比总览海报（用于 PPT 与报告首页）
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from doggie.core.detector import FallDetector  # noqa: E402
from doggie.core.recovery_fsm import RecoveryFSM  # noqa: E402
from doggie.core.types import FsmState as S  # noqa: E402
from doggie.io.sim_backend import Q_PRE, SimBackend  # noqa: E402
from doggie.utils.logger import load_config  # noqa: E402

OUT_DIR = ROOT / "docs" / "report" / "sim_shots"

FONT_CANDIDATES = [
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
]

STATE_BADGE_COLOR = {
    "S0_MONITOR": (70, 130, 180),
    "S1_SAFE_DAMPING": (230, 145, 0),
    "S2_ASSESS": (148, 103, 189),
    "S2B_PREPARE_LIEDOWN": (40, 150, 215),
    "S3_STANDUP": (0, 114, 178),
    "S4_VERIFY": (0, 158, 115),
    "S5_SAFE_FAULT": (195, 30, 45),
}


def load_font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    for p in FONT_CANDIDATES:
        if Path(p).is_file():
            try:
                return ImageFont.truetype(p, size=size)
            except Exception:
                continue
    return ImageFont.load_default()


def render_frame(
    renderer: mujoco.Renderer,
    backend: SimBackend,
    azimuth: float = 45.0,
    elevation: float = -18.0,
    distance: float = 1.48,
    lookat: tuple[float, float, float] = (0.03, 0.0, 0.22),
) -> Image.Image:
    option = mujoco.MjvOption()
    option.geomgroup[3] = 0  # 隐藏碰撞体，保留官方 STL 外观与舞台
    cam = mujoco.MjvCamera()
    cam.distance = distance
    cam.azimuth = azimuth
    cam.elevation = elevation
    cam.lookat[:] = list(lookat)
    renderer.update_scene(backend.data, cam, scene_option=option)
    arr = renderer.render()
    return Image.fromarray(arr)


def annotate_frame(
    img: Image.Image,
    stage_title: str,
    state: str,
    mode: str,
    t: float,
    roll_deg: float,
    pitch_deg: float,
    height_m: float,
    extra: str = "",
) -> Image.Image:
    out = img.copy().convert("RGBA")
    overlay = Image.new("RGBA", out.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)

    w, h = out.size
    font_title = load_font(26)
    font_badge = load_font(22)
    font_meta = load_font(20)

    # Top bar background
    draw.rectangle([(0, 0), (w, 56)], fill=(12, 16, 24, 195))
    draw.text((18, 12), stage_title, fill=(245, 247, 250, 255), font=font_title)

    # State badge on top-right
    b_color = STATE_BADGE_COLOR.get(state, (90, 90, 90))
    badge_text = f"{state} ({mode})"
    bbox = draw.textbbox((0, 0), badge_text, font=font_badge)
    bw = bbox[2] - bbox[0] + 28
    draw.rounded_rectangle([(w - bw - 16, 10), (w - 16, 46)], radius=8, fill=(*b_color, 235))
    draw.text((w - bw - 2, 14), badge_text, fill=(255, 255, 255, 255), font=font_badge)

    # Bottom telemetry bar
    draw.rectangle([(0, h - 48), (w, h)], fill=(12, 16, 24, 195))
    telem = (
        f"t = {t:5.2f} s   |   roll = {roll_deg:+6.1f}°   "
        f"pitch = {pitch_deg:+6.1f}°   |   h = {height_m:.3f} m"
    )
    if extra:
        telem += f"   |   {extra}"
    draw.text((18, h - 38), telem, fill=(225, 232, 240, 255), font=font_meta)

    return Image.alpha_composite(out, overlay).convert("RGB")


def make_strip(images: list[Image.Image], header: str) -> Image.Image:
    """将 4 张 1280x720 关键帧横向拼接为高分辨率序列图（供报告和 PPT 直接插入）。"""
    thumb_w, thumb_h = 640, 360
    pad = 14
    top_h = 54
    n = len(images)
    total_w = n * thumb_w + (n + 1) * pad
    total_h = top_h + thumb_h + pad

    canvas = Image.new("RGB", (total_w, total_h), (18, 22, 30))
    draw = ImageDraw.Draw(canvas)
    font_h = load_font(26)
    draw.text((pad + 4, 12), header, fill=(240, 244, 250), font=font_h)

    for idx, im in enumerate(images):
        resized = im.resize((thumb_w, thumb_h), Image.Resampling.LANCZOS)
        x = pad + idx * (thumb_w + pad)
        canvas.paste(resized, (x, top_h))
        draw.rectangle([(x, top_h), (x + thumb_w - 1, top_h + thumb_h - 1)], outline=(70, 82, 102), width=2)

    return canvas


def capture_scenario(
    cfg: dict,
    key: str,
    title_prefix: str,
    fall: str,
    scene: str,
    targets: list[tuple[str, str, callable]],
    max_time: float = 15.0,
    custom_place: callable | None = None,
) -> list[Image.Image]:
    backend = SimBackend(cfg["sim"], scene=scene)
    detector = FallDetector(cfg["detector"])
    fsm = RecoveryFSM(cfg["fsm"], detector)

    if custom_place is not None:
        custom_place(backend)
    else:
        backend.place_fall(fall)

    renderer = mujoco.Renderer(backend.model, 720, 1280)
    dt = backend.model.opt.timestep
    det_every = max(1, int(round(1.0 / (200.0 * dt))))

    captured: dict[int, Image.Image] = {}
    obs = backend.read()

    # Check t=0 trigger before stepping
    for idx, (slug, label, pred) in enumerate(targets):
        if idx not in captured and pred(0.0, obs, detector, fsm):
            raw = render_frame(renderer, backend)
            extra = fsm.fault_code or (fsm.strategy if fsm.strategy else "")
            ann = annotate_frame(
                raw,
                f"{title_prefix} — {label}",
                fsm.state.value,
                fsm.mode.value,
                obs.t,
                obs.roll_deg,
                obs.pitch_deg,
                obs.base_height,
                extra=extra,
            )
            ann.save(OUT_DIR / f"{key}_{idx+1}_{slug}.png")
            captured[idx] = ann

    step = 0
    while backend.data.time < max_time and len(captured) < len(targets):
        backend.step()
        step += 1
        if step % det_every == 0:
            obs = backend.read()
            detector.update(obs)
            fsm.step(obs, backend)
            backend.set_mode(fsm.mode)

            for idx, (slug, label, pred) in enumerate(targets):
                if idx not in captured and pred(obs.t, obs, detector, fsm):
                    raw = render_frame(renderer, backend)
                    extra = fsm.fault_code or (fsm.strategy if fsm.strategy else "")
                    ann = annotate_frame(
                        raw,
                        f"{title_prefix} — {label}",
                        fsm.state.value,
                        fsm.mode.value,
                        obs.t,
                        obs.roll_deg,
                        obs.pitch_deg,
                        obs.base_height,
                        extra=extra,
                    )
                    ann.save(OUT_DIR / f"{key}_{idx+1}_{slug}.png")
                    captured[idx] = ann

    renderer.close()
    return [captured[i] for i in range(len(targets)) if i in captured]


def place_supine(backend: SimBackend) -> None:
    """构造纯四脚朝天（roll=180° 仰翻，无回摆动量）用于展示能力边界与 S2 直接安全拒止。"""
    backend.reset()
    a = math.radians(178.0) / 2.0
    axis = np.array([1.0, 0.0, 0.0])
    backend.data.qpos[2] = 0.5
    backend.data.qpos[3:7] = np.concatenate([[math.cos(a)], math.sin(a) * axis])
    backend.data.qpos[7:] = Q_PRE
    mujoco.mj_forward(backend.model, backend.data)
    min_dist = min(
        mujoco.mj_geomDistance(backend.model, backend.data, g, backend._floor, 10.0, None)
        for g in backend._robot_geoms()
    )
    backend.data.qpos[2] += 0.005 - min_dist
    mujoco.mj_forward(backend.model, backend.data)
    backend.set_mode(fsm_damping_mode())


def fsm_damping_mode():
    from doggie.core.types import Mode

    return Mode.DAMPING


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    cfg = load_config(ROOT / "config" / "recovery.yaml")

    # 1. 前翻
    front_imgs = capture_scenario(
        cfg,
        key="front",
        title_prefix="前翻恢复 (pitch=+90°)",
        fall="front",
        scene="flat",
        targets=[
            ("inject", "① 初始翻倒注入 (+90°)", lambda t, o, d, f: t == 0.0),
            ("damping", "② 快速通道检出 → S1 安全阻尼", lambda t, o, d, f: f.state == S.S1_SAFE_DAMPING and t >= 0.20),
            ("standup", "③ 阻尼落稳趴地 → S3 起身中段", lambda t, o, d, f: f.state == S.S3_STANDUP and t - f._su_t0 >= 1.6),
            ("recovered", "④ S4 站稳验证完成 → S0 恢复直立", lambda t, o, d, f: "t_recovered" in f.m),
        ],
    )
    strip_front = make_strip(
        front_imgs,
        "场景 1：前翻恢复全流程（0.155 s 快速通道检出 → S1 阻尼落稳 → S3 两阶段起身 → 4.21 s 恢复直立）",
    )
    strip_front.save(OUT_DIR / "strip_front.png")
    print("saved strip_front.png")

    # 2. 侧翻（含 S2B LieDown 收腿整姿）
    side_imgs = capture_scenario(
        cfg,
        key="side_left",
        title_prefix="左侧翻恢复 (roll=+65°)",
        fall="side_left",
        scene="flat",
        targets=[
            ("inject", "① 初始侧翻注入 (+65°)", lambda t, o, d, f: t == 0.0),
            ("damping", "② 姿态判据检出 (0.305s) → S1 阻尼", lambda t, o, d, f: f.state == S.S1_SAFE_DAMPING and t >= 0.35),
            ("liedown", "③ S2B 调用官方 LieDown 收腿整姿", lambda t, o, d, f: f.state == S.S2B_PREPARE_LIEDOWN and t - f._ld_t0 >= 0.9),
            ("recovered", "④ S3 起身 + S4 验证 → S0 恢复直立", lambda t, o, d, f: "t_recovered" in f.m),
        ],
    )
    strip_side = make_strip(
        side_imgs,
        "场景 2：侧翻恢复全流程（0.305 s 检出 → S1 阻尼翻回趴地 → S2B 官方 LieDown 对齐四腿 → S3 起身恢复）",
    )
    strip_side.save(OUT_DIR / "strip_side_left.png")
    print("saved strip_side_left.png")

    # 3. 后翻（带前向回摆）
    back_imgs = capture_scenario(
        cfg,
        key="back",
        title_prefix="后翻恢复 (pitch=-90°+回摆)",
        fall="back",
        scene="flat",
        targets=[
            ("inject", "① 后仰注入 (-90° + 回摆)", lambda t, o, d, f: t == 0.0),
            ("damping", "② 0.305s 检出 → S1 阻尼回摆落趴地", lambda t, o, d, f: f.state == S.S1_SAFE_DAMPING and t >= 0.55),
            ("standup", "③ S2 评估 direct_standup → S3 起身", lambda t, o, d, f: f.state == S.S3_STANDUP and t - f._su_t0 >= 1.6),
            ("recovered", "④ S4 验证通过 → S0 恢复直立", lambda t, o, d, f: "t_recovered" in f.m),
        ],
    )
    strip_back = make_strip(
        back_imgs,
        "场景 3：后翻恢复全流程（后仰 -90° 伴随前向回摆 → 0.305 s 检出 → S1 阻尼落稳趴地 → S3 起身成功）",
    )
    strip_back.save(OUT_DIR / "strip_back.png")
    print("saved strip_back.png")

    # 4. 起身受阻（矮架阻挡 -> 2 次尝试 -> S5_SAFE_FAULT）
    blocked_imgs = capture_scenario(
        cfg,
        key="blocked",
        title_prefix="矮架受阻 (blocked)",
        fall="prone",
        scene="blocked",
        targets=[
            ("detect", "① 跌入矮架下 → 高度判据检出 (0.32s)", lambda t, o, d, f: f.state == S.S1_SAFE_DAMPING and f.attempts == 0),
            ("attempt1", "② 第 1 次起身被横梁压住 (h≈0.10m)", lambda t, o, d, f: f.state == S.S3_STANDUP and f.attempts == 1 and t - f._su_t0 >= 2.2),
            ("attempt2", "③ 退避 2s + LieDown 整姿 → 第 2 次尝试", lambda t, o, d, f: f.state == S.S3_STANDUP and f.attempts == 2 and t - f._su_t0 >= 2.2),
            ("fault", "④ 达上限 2 次 → S5_SAFE_FAULT 安全锁存", lambda t, o, d, f: f.state == S.S5_SAFE_FAULT),
        ],
        max_time=16.0,
    )
    strip_blocked = make_strip(
        blocked_imgs,
        "场景 4：起身受阻异常处理（高度判据 0.32 s 检出 → 第 1 次受阻 → 退避 2 s 第 2 次重试 → S5_SAFE_FAULT 锁存报警）",
    )
    strip_blocked.save(OUT_DIR / "strip_blocked.png")
    print("saved strip_blocked.png")

    # 5. 四脚朝天能力边界（S2_ASSESS 直接识别 unrecoverable_supine -> S5_SAFE_FAULT）
    supine_imgs = capture_scenario(
        cfg,
        key="supine",
        title_prefix="能力边界：纯仰翻 (180°)",
        fall="front",
        scene="flat",
        custom_place=place_supine,
        targets=[
            ("inject", "① 纯仰翻四脚朝天 (roll≈178°)", lambda t, o, d, f: t == 0.0),
            ("detect", "② 快速通道 0.155s 检出 → S1 阻尼", lambda t, o, d, f: f.state == S.S1_SAFE_DAMPING and t >= 0.25),
            ("settle", "③ 阻尼等待静止 (仰面朝天 h≈0.073m)", lambda t, o, d, f: f.state == S.S1_SAFE_DAMPING and t >= 0.70),
            ("fault", "④ S2 识别不可恢复 → 直接 S5 拒止锁存", lambda t, o, d, f: f.state == S.S5_SAFE_FAULT),
        ],
        max_time=6.0,
    )
    strip_supine = make_strip(
        supine_imgs,
        "能力边界展示：纯四脚朝天（无回摆）超出平台 StandUp 包络，S2_ASSESS 直接拒止并锁存 E_UNRECOVERABLE_POSE（0 次盲目起身）",
    )
    strip_supine.save(OUT_DIR / "strip_supine_boundary.png")
    print("saved strip_supine_boundary.png")

    # 6. 总览海报（把 4 个主场景 strip 纵向拼成一张大图，方便 PPT 一页总览）
    strips = [strip_front, strip_side, strip_back, strip_blocked]
    sw, sh = strips[0].size
    grid = Image.new("RGB", (sw, sh * len(strips)), (18, 22, 30))
    for i, st in enumerate(strips):
        grid.paste(st, (0, i * sh))
    grid.save(OUT_DIR / "grid_overview.png")
    print("saved grid_overview.png")


if __name__ == "__main__":
    main()
