from __future__ import annotations

import math

from .types import FallEvent, Observation


class FallDetector:
    """摔倒判定（纯逻辑，不依赖 MuJoCo / ROS）。

    判据结构（详见 docs/01-plan.md 第 4 章）：
      第一层 姿态门限：|roll| 或 |pitch| 超过阈值；
      第二层 持续确认（三选一）：
        a) 姿态超限持续 dwell_s；
        b) 躯干高度低于 height_min_m 持续 height_dwell_s（趴地/仰翻时姿态角≈0，必须靠高度）；
        c) 快速通道：倾角 > fast_path_deg 持续 fast_path_dwell_s；
      触发后事件锁存，直到恢复完成由 FSM 调用 clear()。
    """

    def __init__(self, cfg: dict) -> None:
        self.rate = float(cfg.get("rate_hz", 200.0))
        self.fc = float(cfg.get("lowpass_cutoff_hz", 10.0))
        self.roll_thr = math.radians(float(cfg.get("roll_threshold_deg", 50.0)))
        self.pitch_thr = math.radians(float(cfg.get("pitch_threshold_deg", 50.0)))
        self.fast_thr = math.radians(float(cfg.get("fast_path_deg", 75.0)))
        self.fast_dwell = float(cfg.get("fast_path_dwell_s", 0.15))
        self.dwell = float(cfg.get("dwell_s", 0.30))
        self.use_height = bool(cfg.get("use_height", True))
        self.height_min = float(cfg.get("height_min_m", 0.10))
        self.height_dwell = float(cfg.get("height_dwell_s", 0.30))
        self.type_pitch = math.radians(float(cfg.get("fall_type_pitch_deg", 60.0)))
        self.type_roll = math.radians(float(cfg.get("fall_type_roll_deg", 60.0)))

        self._alpha = 1.0 - math.exp(-2.0 * math.pi * self.fc / self.rate)
        self.reset()

    # ------------------------------------------------------------------ state
    def reset(self) -> None:
        self._fr = 0.0
        self._fp = 0.0
        self._init = False
        self._att_since: float | None = None
        self._h_since: float | None = None
        self._fast_since: float | None = None
        self.latched = False
        self.event: FallEvent | None = None
        self.retriggers = 0

    def clear(self) -> None:
        """恢复完成后由 FSM 调用：解除锁存，重新武装。"""
        if self.latched:
            self.retriggers += 1
        self.latched = False
        self.event = None
        self._att_since = None
        self._h_since = None
        self._fast_since = None

    @property
    def filtered_roll_deg(self) -> float:
        return math.degrees(self._fr)

    @property
    def filtered_pitch_deg(self) -> float:
        return math.degrees(self._fp)

    # ------------------------------------------------------------------ logic
    def _classify(self, obs: Observation) -> str:
        r, p = self._fr, self._fp
        if abs(p) > self.type_pitch:
            return "front" if p > 0 else "back"
        if abs(r) > self.type_roll:
            return "side_left" if r > 0 else "side_right"
        if self.use_height and obs.base_height < self.height_min:
            return "flat"
        return "other"

    def _confirm(self, obs: Observation, cue: str) -> bool:
        self.latched = True
        self.event = FallEvent(
            t=obs.t,
            kind=self._classify(obs),
            cue=cue,
            roll_deg=math.degrees(self._fr),
            pitch_deg=math.degrees(self._fp),
            height=obs.base_height,
        )
        return True

    def update(self, obs: Observation) -> bool:
        """输入一帧观测；返回是否（在本帧）确认摔倒。锁存后恒返回 True。"""
        if self._init:
            self._fr += self._alpha * (obs.roll - self._fr)
            self._fp += self._alpha * (obs.pitch - self._fp)
        else:
            self._fr, self._fp, self._init = obs.roll, obs.pitch, True

        if self.latched:
            return True

        att_bad = abs(self._fr) > self.roll_thr or abs(self._fp) > self.pitch_thr
        tilt = max(abs(self._fr), abs(self._fp))
        h_bad = self.use_height and obs.base_height < self.height_min

        # c) 快速通道
        if tilt > self.fast_thr:
            if self._fast_since is None:
                self._fast_since = obs.t
            if obs.t - self._fast_since >= self.fast_dwell:
                return self._confirm(obs, "fast_path")
        else:
            self._fast_since = None

        # a) 姿态超限 + 驻留
        if att_bad:
            if self._att_since is None:
                self._att_since = obs.t
            if obs.t - self._att_since >= self.dwell:
                return self._confirm(obs, "attitude")
        else:
            self._att_since = None

        # b) 高度异常 + 驻留（趴地 / 仰翻姿态角≈0 时起作用）
        if h_bad:
            if self._h_since is None:
                self._h_since = obs.t
            if obs.t - self._h_since >= self.height_dwell:
                return self._confirm(obs, "height")
        else:
            self._h_since = None

        return False
