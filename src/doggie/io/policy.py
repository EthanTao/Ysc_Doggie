"""官方 Lite3 行走策略（policy.onnx）的 Python runner。

观测 45 维（与官方 Lite3_rl_deploy 一致）：
    base_omega*0.25 (3) | projected_gravity (3) | cmd_vel (3) |
    joint_pos - default (12) | joint_vel*0.05 (12) | last_action (12)
动作 12 维，目标关节角 = default + action * [0.125, 0.25, 0.25]。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

DEFAULT_POSE = np.array([0.0, -0.65, 1.30] * 4, dtype=np.float64)
ACTION_SCALE = np.array([0.125, 0.25, 0.25] * 4, dtype=np.float64)
JOINT_LOWER = np.array([-0.53, -3.50, 0.349] * 4, dtype=np.float64)
JOINT_UPPER = np.array([0.53, 0.320, 2.80] * 4, dtype=np.float64)
MAX_CMD = np.array([0.8, 0.8, 0.8], dtype=np.float64)
KP = 30.0
KD = 1.0
OMEGA_SCALE = 0.25
DOF_VEL_SCALE = 0.05
DECIMATION = 12


class PolicyController:
    def __init__(self, path: Path, max_cmd=None) -> None:
        import onnxruntime as ort

        self.session = ort.InferenceSession(
            str(path), providers=["CPUExecutionProvider"]
        )
        self.input_name = self.session.get_inputs()[0].name
        self.max_cmd = (
            MAX_CMD.copy() if max_cmd is None else np.asarray(max_cmd, dtype=np.float64)
        )
        if self.max_cmd.shape != (3,):
            raise ValueError("max_cmd must have 3 elements [vx, vy, wz]")
        self.last_action = np.zeros(12, dtype=np.float64)
        self.target = DEFAULT_POSE.copy()

    def act(
        self,
        quat_wxyz: np.ndarray,
        omega_body: np.ndarray,
        joint_pos: np.ndarray,
        joint_vel: np.ndarray,
        cmd_vel: np.ndarray,
    ) -> np.ndarray:
        w, x, y, z = quat_wxyz / np.linalg.norm(quat_wxyz)
        rot = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
            ]
        )
        projected_gravity = rot.T @ np.array([0.0, 0.0, -1.0])
        obs = np.concatenate(
            [
                omega_body * OMEGA_SCALE,
                projected_gravity,
                np.asarray(cmd_vel, dtype=np.float64) * self.max_cmd,
                joint_pos - DEFAULT_POSE,
                joint_vel * DOF_VEL_SCALE,
                self.last_action,
            ]
        ).astype(np.float32)

        action = self.session.run(None, {self.input_name: obs[None]})[0][0]
        self.last_action = action.astype(np.float64)
        self.target = np.clip(
            DEFAULT_POSE + self.last_action * ACTION_SCALE, JOINT_LOWER, JOINT_UPPER
        )
        return self.target
