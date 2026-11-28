#!/usr/bin/env python3
"""MuJoCo physics verification for the interruptible Lite3 left-roll recovery.

Mirrors the exact C++ recovery_sdk quintic segment interpolation, bounds/rate/
effort checks, pure-damping STOP latch, StallWindow detector, and official
StandUpState cubic spline.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


RECOVERY_SDK_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[4]
_MJCF_CANDIDATES = [
    REPO_ROOT / "sim/models/lite3/mjcf/Lite3.xml",
    REPO_ROOT / "sim/models/lite3/Lite3.xml",
    RECOVERY_SDK_DIR.parent / "src/Lite3_sdk_deploy/Lite3_description/lite3_mjcf/mjcf/Lite3.xml",
]
DEFAULT_MJCF = next((p for p in _MJCF_CANDIDATES if p.exists()), _MJCF_CANDIDATES[0])
DEFAULT_PROFILE_OUT = RECOVERY_SDK_DIR / "config/left_roll_profile.json"

# Official Lite3 parameters from lite3_control_parameters.cpp
THIGH_LEN = 0.20
SHANK_LEN = 0.21
PRE_HEIGHT = 0.12
STAND_HEIGHT = 0.30
PRE_STAND_DURATION_S = 0.4  # Short settle at PRE_POSE before rising
STAND_DURATION_S = 1.5  # Official stand_duration_ for height rise (0.12m -> 0.30m)
STAND_KP = 100.0
STAND_KD = 2.5
DAMPING_KD = 2.5

# Official hardware joint limits from lite3_control_parameters.cpp
HW_MIN_POS = [-0.530, -3.50, 0.349] * 4
HW_MAX_POS = [0.530, 0.320, 2.80] * 4
HW_MAX_SPEED = [30.0, 30.0, 20.0] * 4
HW_MAX_TORQUE = [40.0, 40.0, 65.0] * 4


def hipy_by_height(h: float) -> float:
    l1, l2 = THIGH_LEN, SHANK_LEN
    theta = -math.acos(max(-1.0, min(1.0, (l1 * l1 + h * h - l2 * l2) / (2.0 * h * l1))))
    return max(-3.50, min(0.320, theta))


def knee_by_height(h: float) -> float:
    l1, l2 = THIGH_LEN, SHANK_LEN
    theta = math.pi - math.acos(max(-1.0, min(1.0, (l1 * l1 + l2 * l2 - h * h) / (2.0 * l1 * l2))))
    return max(0.349, min(2.80, theta))


PRE_POSE_LEG = [0.0, hipy_by_height(PRE_HEIGHT), knee_by_height(PRE_HEIGHT)]
PRE_POSE = PRE_POSE_LEG * 4


def cubic_pos(x0: float, v0: float, xf: float, vf: float, t: float, T: float) -> float:
    if t >= T:
        return xf
    d = x0
    c = v0
    a = (vf * T - 2.0 * xf + v0 * T + 2.0 * x0) / (T**3)
    b = (3.0 * xf - vf * T - 2.0 * v0 * T - 3.0 * x0) / (T**2)
    return a * (t**3) + b * (t**2) + c * t + d


def cubic_vel(x0: float, v0: float, xf: float, vf: float, t: float, T: float) -> float:
    if t >= T:
        return 0.0
    c = v0
    a = (vf * T - 2.0 * xf + v0 * T + 2.0 * x0) / (T**3)
    b = (3.0 * xf - vf * T - 2.0 * v0 * T - 3.0 * x0) / (T**2)
    return 3.0 * a * (t**2) + 2.0 * b * t + c


def quat_wxyz_to_xyzw(q: np.ndarray) -> np.ndarray:
    return np.array([q[1], q[2], q[3], q[0]], dtype=float)


def tilt_deg_from_xyzw(q: np.ndarray) -> float:
    x, y, z, w = q
    c = max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))
    return math.degrees(math.acos(c))


def roll_deg_from_xyzw(q: np.ndarray) -> float:
    x, y, z, w = q
    return math.degrees(math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))


def pitch_deg_from_xyzw(q: np.ndarray) -> float:
    x, y, z, w = q
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return math.degrees(math.asin(sinp))


def attitude_distance_deg(a: np.ndarray, b: np.ndarray) -> float:
    dot = abs(float(np.dot(a, b)))
    na = float(np.linalg.norm(a))
    nb = float(np.linalg.norm(b))
    return 2.0 * math.degrees(math.acos(max(0.0, min(1.0, dot / (na * nb)))))


def quintic_interpolate(start_q, start_kp, start_kd, target_q, target_kp, target_kd, elapsed_ms, duration_ms):
    u = min(1.0, max(0.0, elapsed_ms / duration_ms))
    w = u * u * u * (10.0 + u * (-15.0 + 6.0 * u))
    slope = 30.0 * u * u * (1.0 - u) * (1.0 - u) * 1000.0 / duration_ms
    q = start_q + (target_q - start_q) * w
    v = (target_q - start_q) * slope
    kp = start_kp + (target_kp - start_kp) * w
    kd = start_kd + (target_kd - start_kd) * w
    return q, v, kp, kd


def default_left_roll_profile() -> dict:
    """Segmented left-roll profile derived from the 4 real Lite3 factory left-roll
    logs (normal-01.jsonl) and official control limits, designed for smooth quintic
    interpolation and immediate pure-damping interruption.
    """
    kp_soft = [65.0] * 12
    kp_prep = [80.0] * 12
    kp_roll = [100.0] * 12
    kd_nom = [2.5] * 12

    # Start pose: typical supine resting pose (legs tucked symmetrically)
    start_q = [
        -0.05, -1.36, 2.55,
         0.05, -1.36, 2.55,
        -0.05, -1.36, 2.55,
         0.05, -1.36, 2.55,
    ]

    # Segment 1: Tuck & settle symmetrically while supine (also brings arbitrary
    # supine legs into the canonical start posture if starting slightly off)
    seg1_tuck = [
        -0.05, -1.36, 2.55,
         0.05, -1.36, 2.55,
        -0.05, -1.36, 2.55,
         0.05, -1.36, 2.55,
    ]
    # Segment 2: Swing left thighs back over the torso (-3.48 rad) smoothly
    # (950 ms, kp=80) to keep floor-drag PD effort well inside Guard limits
    seg2_prep = [
        -0.44, -3.48, 2.55,
         0.45, -1.36, 2.55,
        -0.44, -3.48, 2.55,
         0.45, -1.36, 2.55,
    ]
    # Segment 3: Extend left knees (2.55 -> 0.50) firmly to push torso past 90°
    # left roll; swing right thighs back to shift center of mass
    seg3_push = [
        -0.46, -3.48, 0.50,
         0.45, -1.95, 2.56,
        -0.46, -3.48, 0.50,
         0.46, -2.92, 2.56,
    ]
    # Segment 4: Briskly complete roll to prone — retract left legs while
    # sweeping right HipX inward (-0.46) to carry angular momentum past 90°
    seg4_catch = [
        -0.25, -1.40, 2.41,
        -0.46, -2.62, 2.70,
        -0.25, -1.40, 2.41,
        -0.46, -2.62, 2.70,
    ]
    # Segment 6: Symmetrical prone crouched pose ready for official StandUpState
    seg5_prone = list(PRE_POSE)

    return {
        "name": "lite3_left_roll_v3",
        "bounds": {
            "min_position": HW_MIN_POS,
            "max_position": HW_MAX_POS,
            "max_speed": [12.0, 14.0, 12.0] * 4,
            "max_kp": [110.0] * 12,
            "max_kd": [5.0] * 12,
            "max_effort": [55.0, 55.0, 75.0] * 4,
            "max_acceleration": [80.0, 100.0, 80.0] * 4,
            "max_gain_rate": [500.0] * 12,
        },
        "damping": [DAMPING_KD] * 12,
        "initial_position_tolerance": 3.5,
        "stall": {
            "hold_ms": 500,
            "motion_rad": 0.04,
            "speed_rad_s": 0.15,
            "attitude_deg": 3.0,
        },
        "start": {"position": start_q, "kp": kp_soft, "kd": kd_nom},
        "segments": [
            {
                "label": "supine_tuck",
                "duration_ms": 1000,
                "position": seg1_tuck,
                "kp": kp_soft,
                "kd": kd_nom,
                "expects_progress": False,
            },
            {
                "label": "left_swing_prep",
                "duration_ms": 950,
                "position": seg2_prep,
                "kp": kp_prep,
                "kd": kd_nom,
                "expects_progress": True,
            },
            {
                "label": "left_knee_push",
                "duration_ms": 800,
                "position": seg3_push,
                "kp": kp_roll,
                "kd": kd_nom,
                "expects_progress": True,
            },
            {
                "label": "roll_and_catch",
                "duration_ms": 600,
                "position": seg4_catch,
                "kp": kp_roll,
                "kd": kd_nom,
                "expects_progress": True,
            },
            {
                "label": "catch_settle",
                "duration_ms": 350,
                "position": seg4_catch,
                "kp": kp_roll,
                "kd": kd_nom,
                "expects_progress": False,
            },
            {
                "label": "prone_align",
                "duration_ms": 700,
                "position": seg5_prone,
                "kp": kp_soft,
                "kd": kd_nom,
                "expects_progress": True,
            },
        ],
    }


def default_front_restore_profile() -> dict:
    """2-stage interruptible leg-restore profile for forward nose-down / bent-leg prone falls.
    Stage 1 folds knees tightly & centers HipX to clear tangled feet off the floor and
    level the nose-down pitch smoothly; Stage 2 aligns to canonical PRE_POSE for StandUp.
    """
    base = default_left_roll_profile()
    kp_fold = [55.0] * 12
    kp_align = [65.0] * 12
    kd_nom = [2.5] * 12
    seg1_fold = [0.0, -1.25, 2.65] * 4
    seg2_prone = list(PRE_POSE)
    return {
        "name": "lite3_front_restore_v1",
        "bounds": copy.deepcopy(base["bounds"]),
        "damping": list(base["damping"]),
        "initial_position_tolerance": 3.5,
        "stall": copy.deepcopy(base["stall"]),
        "start": {"position": seg2_prone, "kp": kp_fold, "kd": kd_nom},
        "segments": [
            {
                "label": "front_fold_clear",
                "duration_ms": 750,
                "position": seg1_fold,
                "kp": kp_fold,
                "kd": kd_nom,
                "expects_progress": True,
            },
            {
                "label": "prone_align",
                "duration_ms": 700,
                "position": seg2_prone,
                "kp": kp_align,
                "kd": kd_nom,
                "expects_progress": True,
            },
        ],
    }


def default_back_restore_profile() -> dict:
    """2-stage interruptible leg-restore profile for backward sitting-back (rump-down) falls.
    Stage 1 folds front legs gently to lower the chest/nose onto the floor while keeping
    hind legs compactly tucked (avoiding tipping backward onto the spine);
    Stage 2 aligns all 4 legs cleanly to canonical PRE_POSE for StandUp.
    """
    base = default_left_roll_profile()
    kp_lower = [55.0] * 12
    kp_align = [65.0] * 12
    kd_nom = [2.5] * 12
    seg1_lower = [
        0.0, -1.35, 2.60,
        0.0, -1.35, 2.60,
        0.0, -1.55, 2.65,
        0.0, -1.55, 2.65,
    ]
    seg2_prone = list(PRE_POSE)
    return {
        "name": "lite3_back_restore_v1",
        "bounds": copy.deepcopy(base["bounds"]),
        "damping": list(base["damping"]),
        "initial_position_tolerance": 3.5,
        "stall": copy.deepcopy(base["stall"]),
        "start": {"position": seg2_prone, "kp": kp_lower, "kd": kd_nom},
        "segments": [
            {
                "label": "back_front_lower",
                "duration_ms": 800,
                "position": seg1_lower,
                "kp": kp_lower,
                "kd": kd_nom,
                "expects_progress": True,
            },
            {
                "label": "prone_align",
                "duration_ms": 700,
                "position": seg2_prone,
                "kp": kp_align,
                "kd": kd_nom,
                "expects_progress": True,
            },
        ],
    }


def load_model(mjcf_path: Path, hardware_limits: bool = True, obstacle: str | None = None, friction: float = 1.0) -> mujoco.MjModel:
    tree = ET.parse(mjcf_path)
    root = tree.getroot()
    mjcf_dir = mjcf_path.parent

    # Make mesh paths absolute so loading from XML string works cleanly
    for mesh in root.findall(".//mesh"):
        f = mesh.get("file")
        if f:
            mesh.set("file", str((mjcf_dir / f).resolve()))

    # Adjust floor/collision friction if requested
    for geom in root.findall(".//geom"):
        if geom.get("friction"):
            geom.set("friction", f"{friction:.3f} 0.01 0.01")

    if hardware_limits:
        # Match real Lite3 hardware limits from lite3_control_parameters.cpp
        # (Lite3.xml clips HipY at -2.67 whereas real hardware allows -3.50)
        for j in root.findall(".//joint"):
            name = j.get("name", "")
            if "HipY" in name:
                j.set("range", "-3.50 0.320")
            elif "Knee" in name:
                j.set("range", "0.349 2.80")
            elif "HipX" in name:
                j.set("range", "-0.530 0.530")
        for m in root.findall(".//motor"):
            name = m.get("name", "")
            if "Knee" in name:
                m.set("ctrlrange", "-65 65")
            else:
                m.set("ctrlrange", "-40 40")

    if obstacle == "side_wall":
        wb = root.find("worldbody")
        # Rigid wall on the left rolling side (y=+0.28) to block left roll; right side is open
        ET.SubElement(
            wb,
            "geom",
            {
                "name": "block_wall_left",
                "type": "box",
                "pos": "0.0 0.28 0.25",
                "size": "0.60 0.04 0.25",
                "rgba": "0.8 0.2 0.2 0.7",
                "condim": "3",
                "friction": "1.0 0.01 0.01",
            },
        )
    elif obstacle == "right_wall":
        wb = root.find("worldbody")
        # Rigid wall on the right rolling side (y=-0.28) to block right roll; left side is open
        ET.SubElement(
            wb,
            "geom",
            {
                "name": "block_wall_right",
                "type": "box",
                "pos": "0.0 -0.28 0.25",
                "size": "0.60 0.04 0.25",
                "rgba": "0.8 0.2 0.2 0.7",
                "condim": "3",
                "friction": "1.0 0.01 0.01",
            },
        )
    elif obstacle == "both_walls":
        wb = root.find("worldbody")
        # Rigid walls on BOTH left (y=+0.28) and right (y=-0.28): both attempts blocked -> permanent stop
        for name, ypos in (("block_wall_left", "0.28"), ("block_wall_right", "-0.28")):
            ET.SubElement(
                wb,
                "geom",
                {
                    "name": name,
                    "type": "box",
                    "pos": f"0.0 {ypos} 0.25",
                    "size": "0.60 0.04 0.25",
                    "rgba": "0.8 0.2 0.2 0.7",
                    "condim": "3",
                    "friction": "1.0 0.01 0.01",
                },
            )
    elif obstacle == "overhead_bar":
        wb = root.find("worldbody")
        # Low ceiling bar preventing roll/rise on both sides
        ET.SubElement(
            wb,
            "geom",
            {
                "name": "block_bar",
                "type": "box",
                "pos": "0.0 0.0 0.22",
                "size": "0.60 0.60 0.02",
                "rgba": "0.8 0.2 0.2 0.7",
                "condim": "3",
            },
        )
    elif obstacle == "stand_ceiling":
        wb = root.find("worldbody")
        # Full-width ceiling at z=0.27m blocking recovery on both sides
        ET.SubElement(
            wb,
            "geom",
            {
                "name": "stand_block",
                "type": "box",
                "pos": "0.0 0.0 0.27",
                "size": "0.55 0.65 0.02",
                "rgba": "0.8 0.2 0.2 0.7",
                "condim": "3",
            },
        )

    xml_str = ET.tostring(root, encoding="unicode")
    model = mujoco.MjModel.from_xml_string(xml_str)
    model.opt.timestep = 0.001
    return model


class StallDetector:
    def __init__(self, hold_ms: int, motion_rad: float, speed_rad_s: float, attitude_deg: float):
        self.hold_ms = hold_ms
        self.motion_rad = motion_rad
        self.speed_rad_s = speed_rad_s
        self.attitude_deg = attitude_deg
        self.window = []

    def reset(self):
        self.window.clear()

    def update(self, t_ms: int, q: np.ndarray, v: np.ndarray, quat_xyzw: np.ndarray) -> bool:
        self.window.append((t_ms, q.copy(), v.copy(), quat_xyzw.copy()))
        while len(self.window) > 1 and t_ms - self.window[1][0] >= self.hold_ms:
            self.window.pop(0)
        if t_ms - self.window[0][0] < self.hold_ms:
            return False
        q0 = self.window[0][1]
        a0 = self.window[0][3]
        lo = q0.copy()
        hi = q0.copy()
        for _, qi, vi, ai in self.window:
            if attitude_distance_deg(a0, ai) > self.attitude_deg:
                return False
            if np.max(np.abs(vi)) > self.speed_rad_s:
                return False
            lo = np.minimum(lo, qi)
            hi = np.maximum(hi, qi)
            if np.max(hi - lo) > self.motion_rad:
                return False
        return True


def place_on_ground(model: mujoco.MjModel, data: mujoco.MjData, quat_wxyz: list[float], init_q: np.ndarray) -> float:
    """Compute the exact lowest non-penetrating torso z so the robot starts resting
    directly on the floor (zero sky-drop, zero floor penetration)."""
    lo, hi = 0.04, 0.65
    for _ in range(18):
        mid = 0.5 * (lo + hi)
        data.qpos[0:3] = [0.0, 0.0, mid]
        data.qpos[3:7] = quat_wxyz
        data.qpos[7:19] = init_q
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        # Check if any contact with the floor exists
        has_floor_contact = False
        for ci in range(data.ncon):
            c = data.contact[ci]
            if c.geom1 == 0 or c.geom2 == 0:
                has_floor_contact = True
                break
        if has_floor_contact:
            lo = mid
        else:
            hi = mid
    z_touch = hi + 0.002
    data.qpos[0:3] = [0.0, 0.0, z_touch]
    data.qpos[3:7] = quat_wxyz
    data.qpos[7:19] = init_q
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    return z_touch


def initial_joints_for_pose(
    init_roll_deg: float,
    init_pitch_deg: float = 0.0,
    pose_kind: str = "auto",
) -> tuple[np.ndarray, bool]:
    """Return (resting joint angles, use_passive_hold_during_settle) matching realistic Lite3 falls:
    - Supine (|roll| >= 135°): folded supine pose from normal-01.jsonl (pure damping settle)
    - Side-lying (30° <= |roll| < 135°): real Lite3 side-propped tripod pose
    - Forward nose-down / irregular bent prone: nose/chest on ground, bent/crooked legs
    - Backward sitting-back: rump on ground, chest angled up, front legs propped, hind folded
    """
    if pose_kind == "front_bent":
        return np.array([
             0.35, -0.45, 2.25,  -0.32, -0.58, 2.10,
             0.22, -0.82, 1.45,  -0.28, -0.95, 1.60,
        ], dtype=float), True
    if pose_kind == "front_nose":
        return np.array([
             0.32, -0.25, 2.35,  -0.28, -0.35, 2.25,
             0.18, -0.50, 0.90,  -0.18, -0.58, 1.00,
        ], dtype=float), True
    if pose_kind == "front_asym":
        return np.array([
             0.38, -0.35, 1.85,  -0.30, -1.80, 2.50,
             0.22, -0.65, 1.15,  -0.32, -1.55, 2.20,
        ], dtype=float), False
    if pose_kind == "back_sit":
        return np.array([
             0.16, -0.35, 0.85,  -0.16, -0.38, 0.88,
             0.25, -1.95, 2.60,  -0.25, -1.98, 2.62,
        ], dtype=float), True
    if pose_kind == "back_sit_steep":
        return np.array([
             0.20, -0.10, 0.70,  -0.20, -0.12, 0.72,
             0.30, -2.25, 2.68,  -0.30, -2.25, 2.68,
        ], dtype=float), True
    if pose_kind == "back_sit_asym":
        return np.array([
             0.32, -0.25, 0.90,  -0.12, -0.55, 1.15,
             0.28, -1.85, 2.55,  -0.35, -2.10, 2.65,
        ], dtype=float), True

    abs_r = abs(init_roll_deg)
    if abs_r >= 135.0:
        return np.array([
            -0.35, -1.70, 2.70,
             0.38, -1.70, 2.70,
            -0.42, -1.68, 2.70,
             0.42, -1.68, 2.70,
        ], dtype=float), False
    if abs_r >= 30.0:
        # Real Lite3 side-propped tripod joint angles (left-side down, legs form triangle with torso)
        left_side_tripod = [
             0.50, -0.38, 1.00,
            -0.50, -0.39, 1.42,
             0.50, -0.22, 1.00,
            -0.38, -0.42, 1.44,
        ]
        if init_roll_deg < 0.0:
            return np.array(mirror_axis_right(left_side_tripod), dtype=float), True
        return np.array(left_side_tripod, dtype=float), True
    if init_pitch_deg <= -12.0:
        return initial_joints_for_pose(init_roll_deg, init_pitch_deg, "back_sit")
    return initial_joints_for_pose(init_roll_deg, init_pitch_deg, "front_bent")


def select_profile_for_attitude(
    profile_bank: dict[str, dict],
    roll_deg: float,
    pitch_deg: float,
    tilt_deg: float,
) -> tuple[dict, str]:
    """Unified post-settle recovery strategy selector:
    - Side-lying (|roll| >= 35°) or supine (tilt >= 120°): auto left_roll / right_roll
    - Backward sitting-back (|roll| < 35° and pitch <= -12°): back_restore
    - Forward nose-down / irregular bent prone: front_restore
    """
    if abs(roll_deg) >= 35.0 or tilt_deg >= 120.0:
        if -165.0 < roll_deg < 0.0:
            return profile_bank["right"], "right_roll"
        return profile_bank["left"], "left_roll"
    if pitch_deg <= -12.0:
        return profile_bank["back"], "back_restore"
    return profile_bank["front"], "front_restore"


def run_trial(
    profile: dict | None,
    *,
    profile_bank: dict[str, dict] | None = None,
    init_roll_deg: float = 180.0,
    init_pitch_deg: float = 0.0,
    pose_kind: str = "auto",
    friction: float = 1.0,
    stop_at_ms: int | None = None,
    obstacle: str | None = None,
    hardware_limits: bool = True,
    kp_scale: float = 1.0,
    retry_opposite_wait_ms: int = 2000,
    max_roll_attempts: int = 2,
    force_first_strategy: str | None = None,
    view: bool = False,
    speed: float = 1.0,
) -> dict:
    model = load_model(DEFAULT_MJCF, hardware_limits=hardware_limits, obstacle=obstacle, friction=friction)
    data = mujoco.MjData(model)

    # Compute exact floor-touching pose (no sky drop!) with realistic resting legs
    r = math.radians(init_roll_deg)
    p = math.radians(init_pitch_deg)
    cr, sr = math.cos(r / 2.0), math.sin(r / 2.0)
    cp, sp = math.cos(p / 2.0), math.sin(p / 2.0)
    # w, x, y, z for R = Ry(p) * Rx(r)
    quat_wxyz = [cp * cr, cp * sr, sp * cr, -sp * sr]
    init_pose_q, use_passive_hold = initial_joints_for_pose(init_roll_deg, init_pitch_deg, pose_kind)
    place_on_ground(model, data, quat_wxyz, init_pose_q)

    viewer = None
    if view:
        import mujoco.viewer as mj_viewer
        viewer = mj_viewer.launch_passive(model, data)
        viewer.cam.lookat[:] = [0.0, 0.10, 0.20]
        viewer.cam.distance = 1.45
        viewer.cam.azimuth = 135.0
        viewer.cam.elevation = -20.0

    def sync_view(step_idx: int):
        if viewer is None:
            return True
        if not viewer.is_running():
            return False
        if step_idx % 16 == 0:  # ~60 FPS
            viewer.sync()
            time.sleep(0.016 / max(0.1, speed))
        return True

    # Settle naturally on the ground for 0.8s:
    # For side-propped tripod or front/back crooked-leg falls, mild passive gearbox
    # resistance holds the realistic fallen posture on the ground before recovery starts;
    # supine / limp falls settle in pure damping.
    for settle_i in range(800):
        q = data.qpos[7:19]
        v = data.qvel[6:18]
        if use_passive_hold:
            data.ctrl[:] = np.clip(18.0 * (init_pose_q - q) - 3.5 * v, -20.0, 20.0)
        else:
            data.ctrl[:] = -DAMPING_KD * v
        mujoco.mj_step(model, data)
        if not sync_view(settle_i):
            break

    init_settled_q = data.qpos[7:19].copy()
    init_quat_xyzw = quat_wxyz_to_xyzw(data.qpos[3:7])
    init_settled_tilt = tilt_deg_from_xyzw(init_quat_xyzw)
    init_settled_roll = roll_deg_from_xyzw(init_quat_xyzw)
    init_settled_pitch = pitch_deg_from_xyzw(init_quat_xyzw)

    if profile_bank is not None:
        if force_first_strategy is not None:
            key_map = {"left_roll": "left", "right_roll": "right", "front_restore": "front", "back_restore": "back"}
            profile = profile_bank[key_map[force_first_strategy]]
            selected_strategy = force_first_strategy
        else:
            profile, selected_strategy = select_profile_for_attitude(
                profile_bank, init_settled_roll, init_settled_pitch, init_settled_tilt
            )
    else:
        assert profile is not None
        selected_strategy = profile.get("name", "custom")

    segments = profile["segments"]
    stall_cfg = profile["stall"]
    stall = StallDetector(
        stall_cfg["hold_ms"],
        stall_cfg["motion_rad"],
        stall_cfg["speed_rad_s"],
        stall_cfg["attitude_deg"],
    )

    phase = "rolling"
    attempt = 1
    strategies_tried = [selected_strategy]
    roll_start_ms = 0
    total_roll_ms = sum(seg["duration_ms"] for seg in segments)
    effort_bounds = np.array(profile["bounds"]["max_effort"], dtype=float)
    prone_stable_start_ms = None
    prone_fail_start_ms = None
    stand_start_ms = None
    stand_init_q = None
    stand_init_v = None
    stop_reason = None
    stop_ms = None
    first_damping_ms = None
    first_stop_reason = None
    first_stop_ms = None
    first_damping_delay_ms = None
    retry_wait_start_ms = None
    peak_torque = 0.0
    peak_guard_effort_ratio = 0.0
    peak_vel = 0.0
    roll_end_tilt = None

    # First segment starts smoothly from actual settled posture
    first_kp = np.array(profile["start"]["kp"], dtype=float) * kp_scale
    first_kd = np.array(profile["start"]["kd"], dtype=float)
    damping_kd = np.array(profile["damping"], dtype=float)

    def handle_obstacle_stop(reason: str, t_now: int):
        nonlocal phase, stop_reason, stop_ms, first_damping_ms
        nonlocal first_stop_reason, first_stop_ms, first_damping_delay_ms, retry_wait_start_ms
        can_retry = (
            profile_bank is not None
            and attempt < max_roll_attempts
            and selected_strategy in ("left_roll", "right_roll")
            and reason in ("stall", "prone_timeout", "roll_blocked", "effort_fault")
        )
        if can_retry:
            first_stop_reason = reason
            first_stop_ms = t_now
            first_damping_delay_ms = 0
            retry_wait_start_ms = t_now
            phase = "retry_wait"
            stall.reset()
        else:
            stop_reason = reason
            stop_ms = t_now
            phase = "stopped"

    max_ms = 18500
    for t_ms in range(max_ms):
        q = data.qpos[7:19].copy()
        v = data.qvel[6:18].copy()
        quat = quat_wxyz_to_xyzw(data.qpos[3:7])
        tilt = tilt_deg_from_xyzw(quat)
        peak_vel = max(peak_vel, float(np.max(np.abs(v))))

        if stop_at_ms is not None and t_ms == stop_at_ms and phase != "stopped":
            # Operator handset STOP always latches permanently (never auto-retries!)
            stop_reason = "handset_stop"
            stop_ms = t_ms
            phase = "stopped"

        if phase == "retry_wait":
            # Hold pure damping for retry_opposite_wait_ms (default 2000ms), then retry opposite side!
            if t_ms - retry_wait_start_ms >= retry_opposite_wait_ms:
                next_key = "right" if selected_strategy == "left_roll" else "left"
                selected_strategy = "right_roll" if next_key == "right" else "left_roll"
                profile = profile_bank[next_key]
                segments = profile["segments"]
                total_roll_ms = sum(seg["duration_ms"] for seg in segments)
                effort_bounds = np.array(profile["bounds"]["max_effort"], dtype=float)
                init_settled_q = q.copy()
                roll_start_ms = t_ms
                prone_stable_start_ms = None
                prone_fail_start_ms = None
                stall.reset()
                attempt += 1
                strategies_tried.append(selected_strategy)
                phase = "rolling"

        if phase == "rolling":
            elapsed = t_ms - roll_start_ms
            idx = 0
            while idx < len(segments) and elapsed >= segments[idx]["duration_ms"]:
                elapsed -= segments[idx]["duration_ms"]
                idx += 1
            if idx == len(segments):
                phase = "prone_check"
                roll_end_tilt = tilt
                prone_stable_start_ms = None
                prone_fail_start_ms = None
                stall.reset()
            else:
                seg = segments[idx]
                if idx == 0:
                    s_q = init_settled_q
                    s_kp = np.zeros(12)
                    s_kd = damping_kd
                else:
                    s_q = np.array(segments[idx - 1]["position"], dtype=float)
                    s_kp = np.array(segments[idx - 1]["kp"], dtype=float) * kp_scale
                    s_kd = np.array(segments[idx - 1]["kd"], dtype=float)
                t_q = np.array(seg["position"], dtype=float)
                t_kp = np.array(seg["kp"], dtype=float) * kp_scale
                t_kd = np.array(seg["kd"], dtype=float)
                cmd_q, cmd_v, cmd_kp, cmd_kd = quintic_interpolate(
                    s_q, s_kp, s_kd, t_q, t_kp, t_kd, elapsed, seg["duration_ms"]
                )
                if t_ms % 5 == 0:
                    if seg["expects_progress"]:
                        if stall.update(t_ms, q, v, quat):
                            handle_obstacle_stop("stall", t_ms)
                    else:
                        stall.reset()

        if phase == "prone_check":
            cmd_q = np.array(segments[-1]["position"], dtype=float)
            cmd_v = np.zeros(12)
            cmd_kp = np.array(segments[-1]["kp"], dtype=float) * kp_scale
            cmd_kd = np.array(segments[-1]["kd"], dtype=float)
            # Prone confirmation: tilt <= 20 deg and joints converged near prone pose
            joint_err = float(np.max(np.abs(q - np.array(PRE_POSE))))
            if tilt <= 20.0 and joint_err <= 0.65:
                prone_fail_start_ms = None
                if prone_stable_start_ms is None:
                    prone_stable_start_ms = t_ms
                elif t_ms - prone_stable_start_ms >= 500:
                    phase = "standing"
                    stand_start_ms = t_ms
                    stand_init_q = q.copy()
                    stand_init_v = v.copy()
                    stall.reset()
            else:
                prone_stable_start_ms = None
                # Fast blocked-roll detection: if roll ended and torso fell back > 90 deg for 600ms
                if tilt > 90.0:
                    if prone_fail_start_ms is None:
                        prone_fail_start_ms = t_ms
                    elif t_ms - prone_fail_start_ms >= 600:
                        handle_obstacle_stop("roll_blocked", t_ms)
                else:
                    prone_fail_start_ms = None
                if phase == "prone_check" and t_ms - roll_start_ms >= total_roll_ms + 1800:
                    handle_obstacle_stop("prone_timeout", t_ms)

        if phase == "standing":
            st = (t_ms - stand_start_ms) / 1000.0
            cmd_q = np.zeros(12)
            cmd_v = np.zeros(12)
            cmd_kp = np.full(12, STAND_KP * kp_scale)
            cmd_kd = np.full(12, STAND_KD)
            if st <= PRE_STAND_DURATION_S:
                for i in range(12):
                    cmd_q[i] = cubic_pos(stand_init_q[i], stand_init_v[i], PRE_POSE[i], 0.0, st, PRE_STAND_DURATION_S)
                    cmd_v[i] = cubic_vel(stand_init_q[i], stand_init_v[i], PRE_POSE[i], 0.0, st, PRE_STAND_DURATION_S)
                stall.reset()
            else:
                nt = st - PRE_STAND_DURATION_S
                dt = 0.001
                h = cubic_pos(PRE_HEIGHT, 0.0, STAND_HEIGHT, 0.0, nt, STAND_DURATION_S)
                h2 = cubic_pos(PRE_HEIGHT, 0.0, STAND_HEIGHT, 0.0, nt + dt, STAND_DURATION_S)
                hy = hipy_by_height(h)
                hy_v = (hipy_by_height(h2) - hy) / dt
                kn = knee_by_height(h)
                kn_v = (knee_by_height(h2) - kn) / dt
                for leg in range(4):
                    cmd_q[3 * leg + 0] = 0.0
                    cmd_q[3 * leg + 1] = hy
                    cmd_q[3 * leg + 2] = kn
                    cmd_v[3 * leg + 0] = 0.0
                    cmd_v[3 * leg + 1] = hy_v
                    cmd_v[3 * leg + 2] = kn_v

                if t_ms % 5 == 0 and 0.2 < nt < STAND_DURATION_S:
                    if stall.update(t_ms, q, v, quat):
                        stop_reason = "stall"
                        stop_ms = t_ms
                        phase = "stopped"

            if phase == "standing" and st >= PRE_STAND_DURATION_S + STAND_DURATION_S + 0.5:
                if tilt <= 15.0 and data.qpos[2] >= 0.24:
                    phase = "upright_hold"
                    break
                else:
                    stop_reason = "stand_failed"
                    stop_ms = t_ms
                    phase = "stopped"

        if phase not in ("stopped", "retry_wait"):
            tau_unclamped = cmd_kp * (cmd_q - q) + cmd_kd * (cmd_v - v)
            ratio = float(np.max(np.abs(tau_unclamped) / effort_bounds))
            peak_guard_effort_ratio = max(peak_guard_effort_ratio, ratio)
            if ratio > 1.0:
                handle_obstacle_stop("effort_fault", t_ms)

        if phase == "stopped":
            if first_damping_ms is None:
                first_damping_ms = t_ms
            tau = -damping_kd * v
            if t_ms - stop_ms >= 2000:
                break
        elif phase == "retry_wait":
            tau = -damping_kd * v
        else:
            tau = tau_unclamped

        lim = np.array(HW_MAX_TORQUE if hardware_limits else [30.0] * 12)
        tau = np.clip(tau, -lim, lim)
        peak_torque = max(peak_torque, float(np.max(np.abs(tau))))
        data.ctrl[:] = tau
        mujoco.mj_step(model, data)
        if not sync_view(t_ms):
            break

    if viewer is not None and viewer.is_running():
        # Hold the final standing/stopped state visibly for 3 seconds (1000 Hz PD/damping)
        hy = hipy_by_height(STAND_HEIGHT)
        kn = knee_by_height(STAND_HEIGHT)
        q_des = np.array([0.0, hy, kn] * 4)
        lim = np.array(HW_MAX_TORQUE if hardware_limits else [30.0] * 12)
        hold_end = time.monotonic() + 3.0
        while viewer.is_running() and time.monotonic() < hold_end:
            for _ in range(16):
                q = data.qpos[7:19]
                v = data.qvel[6:18]
                if phase == "upright_hold":
                    data.ctrl[:] = np.clip(STAND_KP * kp_scale * (q_des - q) - STAND_KD * v, -lim, lim)
                else:
                    data.ctrl[:] = np.clip(-damping_kd * v, -lim, lim)
                mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.016)
        viewer.close()

    final_tilt = tilt_deg_from_xyzw(quat_wxyz_to_xyzw(data.qpos[3:7]))
    final_h = float(data.qpos[2])
    final_max_vel = float(np.max(np.abs(data.qvel[6:18])))
    return {
        "strategy": selected_strategy,
        "attempts": attempt,
        "strategies_tried": strategies_tried,
        "first_stop_reason": first_stop_reason,
        "first_stop_ms": first_stop_ms,
        "first_damping_delay_ms": first_damping_delay_ms,
        "retry_wait_ms": retry_opposite_wait_ms if attempt > 1 else None,
        "phase": phase,
        "stop_reason": stop_reason,
        "stop_ms": stop_ms,
        "damping_delay_ms": (first_damping_ms - stop_ms) if stop_ms is not None and first_damping_ms is not None else None,
        "init_tilt_deg": round(init_settled_tilt, 1),
        "init_pitch_deg": round(init_settled_pitch, 1),
        "roll_end_tilt_deg": round(roll_end_tilt, 1) if roll_end_tilt is not None else None,
        "final_tilt_deg": round(final_tilt, 1),
        "final_height_m": round(final_h, 3),
        "final_max_joint_vel": round(final_max_vel, 3),
        "peak_torque_nm": round(peak_torque, 2),
        "peak_guard_effort_ratio": round(peak_guard_effort_ratio, 3),
        "peak_joint_vel": round(peak_vel, 2),
    }


def mirror_axis_right(a: list[float]) -> list[float]:
    """Mirror a 12-joint vector FL<->FR, HL<->HR and negate HipX for right-roll."""
    out = [0.0] * 12
    for l_off, r_off in ((0, 3), (6, 9)):
        out[l_off + 0] = -a[r_off + 0]
        out[l_off + 1] = a[r_off + 1]
        out[l_off + 2] = a[r_off + 2]
        out[r_off + 0] = -a[l_off + 0]
        out[r_off + 1] = a[l_off + 1]
        out[r_off + 2] = a[l_off + 2]
    return out


def mirror_profile_right(profile: dict) -> dict:
    m = copy.deepcopy(profile)
    m["name"] = profile.get("name", "roll") + "_right_mirror"
    m["start"]["position"] = mirror_axis_right(m["start"]["position"])
    for seg in m["segments"]:
        seg["position"] = mirror_axis_right(seg["position"])
    return m


def select_profile_for_roll(left_profile: dict, right_profile: dict, init_roll_deg: float) -> tuple[dict, str]:
    if -165.0 < init_roll_deg < 0.0:
        return right_profile, "right"
    return left_profile, "left"


def all_scenarios() -> dict[str, tuple[str, str, dict]]:
    """Map scenario_key -> (category, description, run_trial kwargs)."""
    return {
        # 1. 仰倒与扰动 / 摩擦 / 软增益
        "nominal_180": ("supine", "标准仰倒 180° 左翻 + 站立", dict(init_roll_deg=180.0)),
        "roll_165": ("supine", "偏倾仰倒 165° 左翻 + 站立", dict(init_roll_deg=165.0)),
        "roll_195": ("supine", "偏倾仰倒 195° 左翻 + 站立", dict(init_roll_deg=195.0)),
        "pitch_p10": ("supine", "仰倒 +10° 俯仰扰动", dict(init_roll_deg=180.0, init_pitch_deg=10.0)),
        "pitch_m10": ("supine", "仰倒 -10° 俯仰扰动", dict(init_roll_deg=180.0, init_pitch_deg=-10.0)),
        "low_fric_0.6": ("supine", "低摩擦地面 (μ=0.6) 仰倒左翻", dict(init_roll_deg=180.0, friction=0.6)),
        "high_fric_1.4": ("supine", "高摩擦地面 (μ=1.4) 仰倒左翻", dict(init_roll_deg=180.0, friction=1.4)),
        "soft_kp_0.7": ("supine", "70% 软增益仰倒左翻", dict(init_roll_deg=180.0, kp_scale=0.7)),
        # 2. 多角度左/右侧倒（自动左翻/镜像右翻选向）
        "side_left_59": ("side", "左侧倒 +59° 自动左翻 + 站立", dict(init_roll_deg=59.0)),
        "side_right_59": ("side", "右侧倒 -59° 自动镜像右翻 + 站立", dict(init_roll_deg=-59.0)),
        "side_left_75": ("side", "左侧倒 +75° 自动左翻 + 站立", dict(init_roll_deg=75.0)),
        "side_right_75": ("side", "右侧倒 -75° 自动镜像右翻 + 站立", dict(init_roll_deg=-75.0)),
        "side_left_90": ("side", "左侧倒 +90° 自动左翻 + 站立", dict(init_roll_deg=90.0)),
        "side_right_90": ("side", "右侧倒 -90° 自动镜像右翻 + 站立", dict(init_roll_deg=-90.0)),
        "side_left_110": ("side", "左侧倒 +110° 自动左翻 + 站立", dict(init_roll_deg=110.0)),
        "side_right_110": ("side", "右侧倒 -110° 自动镜像右翻 + 站立", dict(init_roll_deg=-110.0)),
        # 3. 前/后摔倒（机首倒地曲折趴姿 / 向后坐姿）整理腿姿 + 站立
        "front_bent_prone": ("pitch", "前摔机首着地 + 曲折不规则腿姿 → 整理腿姿 + 站立", dict(init_roll_deg=0.0, init_pitch_deg=22.0, pose_kind="front_bent")),
        "front_nose_steep": ("pitch", "前摔大俯仰机首扎地 (+35°) → 整理腿姿 + 站立", dict(init_roll_deg=0.0, init_pitch_deg=35.0, pose_kind="front_nose")),
        "front_asym_tangled": ("pitch", "前摔不对称扭曲（自动按落定姿态选策略）→ 站立", dict(init_roll_deg=10.0, init_pitch_deg=28.0, pose_kind="front_asym")),
        "front_soft_kp": ("pitch", "前摔机首着地 70% 软增益 → 整理腿姿 + 站立", dict(init_roll_deg=0.0, init_pitch_deg=26.0, pose_kind="front_bent", kp_scale=0.7)),
        "back_sit": ("pitch", "后摔坐下姿态 (-35° 臀着地前腿撑) → 落胸收腿 + 站立", dict(init_roll_deg=0.0, init_pitch_deg=-35.0, pose_kind="back_sit")),
        "back_sit_steep": ("pitch", "后摔大仰角坐姿 (-55°) → 落胸收腿 + 站立", dict(init_roll_deg=0.0, init_pitch_deg=-55.0, pose_kind="back_sit_steep")),
        "back_sit_asym": ("pitch", "后摔偏倾不对称坐姿 (roll -10°, pitch -40°) → 站立", dict(init_roll_deg=-10.0, init_pitch_deg=-40.0, pose_kind="back_sit_asym")),
        "back_soft_kp": ("pitch", "后摔坐姿 70% 软增益 → 落胸收腿 + 站立", dict(init_roll_deg=0.0, init_pitch_deg=-35.0, pose_kind="back_sit", kp_scale=0.7)),
        # 4. 翻身/整腿/起立中途 STOP 中断
        "stop_early": ("stop", "翻身早段 (0.8s) STOP 切纯阻尼", dict(init_roll_deg=180.0, stop_at_ms=800)),
        "stop_mid": ("stop", "翻身中段 (2.0s) STOP 切纯阻尼", dict(init_roll_deg=180.0, stop_at_ms=2000)),
        "stop_stand": ("stop", "起立段 (5.5s) STOP 切纯阻尼", dict(init_roll_deg=180.0, stop_at_ms=5500)),
        "stop_front_mid": ("stop", "前摔整腿中段 (0.6s) STOP 切纯阻尼", dict(init_roll_deg=0.0, init_pitch_deg=26.0, pose_kind="front_bent", stop_at_ms=600)),
        "stop_back_mid": ("stop", "后摔坐姿整腿中段 (0.6s) STOP 切纯阻尼", dict(init_roll_deg=0.0, init_pitch_deg=-35.0, pose_kind="back_sit", stop_at_ms=600)),
        # 5. 单侧障碍受阻 → 纯阻尼停等 2s → 换另一侧翻身恢复成功
        "blocked_side_wall": ("obstacle_retry", "左侧墙受阻 → 0ms纯阻尼等2s → 换右侧翻身+站立", dict(init_roll_deg=180.0, obstacle="side_wall")),
        "blocked_right_wall": ("obstacle_retry", "右侧墙受阻 → 0ms纯阻尼等2s → 换左侧翻身+站立", dict(init_roll_deg=180.0, obstacle="right_wall", force_first_strategy="right_roll")),
        # 6. 双侧/顶部均受阻 → 第二次仍失败后永久锁存纯阻尼停下
        "blocked_both_walls": ("blocked", "左右双侧墙受阻 → 换侧仍失败 → 永久停纯阻尼", dict(init_roll_deg=180.0, obstacle="both_walls")),
        "blocked_overhead": ("blocked", "上方低矮顶障受阻 → 换侧仍失败 → 永久停纯阻尼", dict(init_roll_deg=180.0, obstacle="overhead_bar")),
        "blocked_stand_ceil": ("blocked", "全宽顶障受阻 → 换侧仍失败 → 永久停纯阻尼", dict(init_roll_deg=180.0, obstacle="stand_ceiling")),
    }


def main():
    scenarios = all_scenarios()
    scenario_choices = list(scenarios.keys()) + ["nominal", "demo_all", "demo_sides", "demo_pitch", "demo_obstacle", "custom"]

    parser = argparse.ArgumentParser()
    parser.add_argument("--suite", choices=["quick", "all"], default="all")
    parser.add_argument("--export-profile", type=Path, default=None)
    parser.add_argument("--export-all-profiles", type=Path, default=None, help="Directory to export left/front/back profile JSONs")
    parser.add_argument("--view", action="store_true", help="Open interactive 3D MuJoCo viewer window")
    parser.add_argument("--scenario", choices=scenario_choices, default=None, help="Run a single named scenario (or demo_all / demo_sides / demo_pitch / demo_obstacle / custom)")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier in --view mode")
    parser.add_argument("--roll", type=float, default=180.0, help="Custom initial roll angle (deg) for --scenario custom")
    parser.add_argument("--pitch", type=float, default=0.0, help="Custom initial pitch angle (deg) for --scenario custom")
    parser.add_argument("--pose-kind", choices=["auto", "front_bent", "front_nose", "front_asym", "back_sit", "back_sit_steep", "back_sit_asym"], default="auto")
    parser.add_argument("--friction", type=float, default=1.0, help="Custom floor friction for --scenario custom")
    parser.add_argument("--kp-scale", type=float, default=1.0, help="Custom kp scale for --scenario custom")
    parser.add_argument("--stop-ms", type=int, default=None, help="Custom STOP injection time (ms) for --scenario custom")
    args = parser.parse_args()

    left_profile = default_left_roll_profile()
    right_profile = mirror_profile_right(left_profile)
    front_profile = default_front_restore_profile()
    back_profile = default_back_restore_profile()
    profile_bank = {
        "left": left_profile,
        "right": right_profile,
        "front": front_profile,
        "back": back_profile,
    }

    def maybe_export():
        if args.export_profile:
            args.export_profile.parent.mkdir(parents=True, exist_ok=True)
            args.export_profile.write_text(json.dumps(left_profile, indent=2) + "\n", encoding="utf-8")
            print(f"\nExported profile to {args.export_profile}")
        if args.export_all_profiles:
            args.export_all_profiles.mkdir(parents=True, exist_ok=True)
            for fname, prof_obj in (
                ("left_roll_profile.json", left_profile),
                ("front_restore_profile.json", front_profile),
                ("back_restore_profile.json", back_profile),
            ):
                p = args.export_all_profiles / fname
                p.write_text(json.dumps(prof_obj, indent=2) + "\n", encoding="utf-8")
                print(f"Exported {p}")

    if args.scenario is not None or args.view:
        chosen = args.scenario or "nominal_180"
        if chosen == "nominal":
            chosen = "nominal_180"
        if chosen == "demo_all":
            keys = ["front_bent_prone", "back_sit", "nominal_180", "side_left_59", "side_right_90", "stop_mid", "blocked_side_wall", "blocked_both_walls"]
        elif chosen == "demo_sides":
            keys = ["side_left_59", "side_right_59", "side_left_90", "side_right_90", "side_left_110", "side_right_110"]
        elif chosen == "demo_pitch":
            keys = ["front_bent_prone", "front_nose_steep", "back_sit", "back_sit_steep", "stop_front_mid", "stop_back_mid"]
        elif chosen == "demo_obstacle":
            keys = ["blocked_side_wall", "blocked_right_wall", "blocked_both_walls", "blocked_overhead"]
        elif chosen == "custom":
            keys = []
            print(f"\n>>> Custom scenario: roll={args.roll}°, pitch={args.pitch}°, pose={args.pose_kind}, kp_scale={args.kp_scale}", flush=True)
            res = run_trial(
                None,
                profile_bank=profile_bank,
                init_roll_deg=args.roll,
                init_pitch_deg=args.pitch,
                pose_kind=args.pose_kind,
                friction=args.friction,
                kp_scale=args.kp_scale,
                stop_at_ms=args.stop_ms,
                view=args.view,
                speed=args.speed,
            )
            print(f"    Result: {json.dumps(res)}", flush=True)
        else:
            keys = [chosen]

        for key in keys:
            cat, title, kw = scenarios[key]
            print(f"\n>>> [{key}] {title}", flush=True)
            res = run_trial(None, profile_bank=profile_bank, view=args.view, speed=args.speed, **kw)
            print(f"    Result: {json.dumps(res)}", flush=True)
        maybe_export()
        return

    all_ok = True
    groups = [
        ("1. Supine & Perturbed Left-Roll + StandUp", "supine"),
        ("2. Multi-Angle Left/Right Side-Lying Auto-Direction + StandUp", "side"),
        ("3. Forward Nose-Down & Backward Sit-Back Leg-Restore + StandUp", "pitch"),
        ("4. Mid-Recovery STOP Interruption Tests (Roll / Pitch-Restore / Stand)", "stop"),
        ("5. Single-Side Obstacle -> 2s Pure Damping -> Opposite-Side Recovery", "obstacle_retry"),
        ("6. Both-Sides / Overhead Blocked -> 2nd Failure Permanent Stop", "blocked"),
    ]
    for header, cat in groups:
        print(f"\n=== {header} ===")
        items = [(k, v) for k, v in scenarios.items() if v[0] == cat]
        if args.suite == "quick" and cat in ("supine", "side", "pitch"):
            items = items[:2]
        for key, (_, _, kw) in items:
            res = run_trial(None, profile_bank=profile_bank, **kw)
            if cat in ("supine", "side", "pitch"):
                ok = res["phase"] == "upright_hold" and res["attempts"] == 1
            elif cat == "obstacle_retry":
                ok = (
                    res["phase"] == "upright_hold"
                    and res["attempts"] == 2
                    and res["first_damping_delay_ms"] == 0
                    and res["retry_wait_ms"] == 2000
                    and len(set(res["strategies_tried"])) == 2
                )
            elif cat == "stop":
                ok = (
                    res["phase"] == "stopped"
                    and res["attempts"] == 1
                    and res["stop_reason"] == "handset_stop"
                    and res["damping_delay_ms"] == 0
                    and res["final_max_joint_vel"] < 0.5
                )
            else:
                ok = (
                    res["phase"] == "stopped"
                    and res["attempts"] == 2
                    and res["stop_reason"] in ("stall", "prone_timeout", "roll_blocked", "stand_failed", "effort_fault")
                    and res["first_damping_delay_ms"] == 0
                    and res["damping_delay_ms"] == 0
                    and res["final_max_joint_vel"] < 0.5
                )
            all_ok &= ok
            print(f"  [{'PASS' if ok else 'FAIL'}] {key:20s} ({' -> '.join(res['strategies_tried']):24s}): {json.dumps(res)}")

    maybe_export()
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
