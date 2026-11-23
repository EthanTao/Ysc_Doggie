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
        d) 高速翻转通道：姿态越限 + |omega| >= rate_fast_path_rad_s
           持续 rate_fast_dwell_s（抓完整 360° 翻转的高速段）；
      触发后事件锁存，直到恢复完成由 FSM 调用 clear()。
    """

    def __init__(self, cfg: dict) -> None:
        self.rate = float(cfg.get("rate_hz", 200.0))
        self.fc = float(cfg.get("lowpass_cutoff_hz", 10.0))
        self.roll_thr = math.radians(float(cfg.get("roll_threshold_deg", 50.0)))
        self.pitch_thr = math.radians(float(cfg.get("pitch_threshold_deg", 50.0)))
        self.fast_thr = math.radians(float(cfg.get("fast_path_deg", 75.0)))
        self.fast_dwell = float(cfg.get("fast_path_dwell_s", 0.15))
        self.rate_fast_thr = float(cfg.get("rate_fast_path_rad_s", 5.0))
        self.rate_fast_dwell = float(cfg.get("rate_fast_dwell_s", 0.05))
        self.dwell = float(cfg.get("dwell_s", 0.30))
        self.use_height = bool(cfg.get("use_height", True))
        self.height_min = float(cfg.get("height_min_m", 0.10))
        self.height_dwell = float(cfg.get("height_dwell_s", 0.30))
        # 判定“已恢复直立、可重置事件起点”的高度；比趴地门限略高，
        # 防止全翻落地后 h≈0.10 m 徘徊时过早清掉起始姿态。
        self.stable_height = float(
            cfg.get("stable_height_m", self.height_min + 0.02)
        )
        self.rec_roll = math.radians(float(cfg.get("recover_roll_deg", 35.0)))
        self.rec_pitch = math.radians(float(cfg.get("recover_pitch_deg", 35.0)))
        self.rec_dwell = float(cfg.get("recover_dwell_s", 0.50))
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
        self._rate_fast_since: float | None = None
        self._rec_since: float | None = None
        self._first_bad: float | None = None
        self._onset_rpy: tuple[float, float, float] | None = None
        self._peak_roll = 0.0
        self._peak_pitch = 0.0
        self.latched = False
        self.in_hysteresis_band = False
        self.event: FallEvent | None = None
        self.retriggers = 0

    def clear(self) -> None:
        """恢复完成后由 FSM 调用：解除锁存，重新武装。"""
        if self.latched:
            self.retriggers += 1
        self.latched = False
        self.in_hysteresis_band = False
        self.event = None
        self._att_since = None
        self._h_since = None
        self._fast_since = None
        self._rate_fast_since = None
        self._rec_since = None
        self._first_bad = None
        self._onset_rpy = None

    @property
    def filtered_roll_deg(self) -> float:
        return math.degrees(self._fr)

    @property
    def filtered_pitch_deg(self) -> float:
        return math.degrees(self._fp)

    # ------------------------------------------------------------------ logic
    def _classify(self, obs: Observation) -> str:
        # 优先用“首个判据越限时”的重力方向分类。完整 360° 翻转会让欧拉角
        # 在 ±90° 附近换轴（pitch 90° 后表现为 roll=180°），用全程峰值会把
        # 后翻误判成前翻；重力在机体系的投影不会因绕圈而改变语义。
        if self._onset_rpy is not None:
            roll, pitch, _ = self._onset_rpy
            gx = math.sin(pitch)
            gy = -math.sin(roll) * math.cos(pitch)
            if abs(gx) >= math.sin(self.type_pitch) and abs(gx) >= abs(gy):
                return "front" if gx > 0 else "back"
            if abs(gy) >= math.sin(self.type_roll):
                return "side_left" if gy < 0 else "side_right"

        # 回退：未知初始姿态时用历史峰值（兼容早期数据/静态注入）。
        r, p = self._peak_roll, self._peak_pitch
        if abs(p) >= self.type_pitch:
            return "front" if p > 0 else "back"
        if abs(r) >= self.type_roll:
            return "side_left" if r > 0 else "side_right"
        if self.use_height and obs.base_height < self.height_min:
            return "flat"
        return "other"

    def _confirm(self, obs: Observation, cue: str) -> bool:
        self.latched = True
        delay = obs.t - self._first_bad if self._first_bad is not None else 0.0
        self.event = FallEvent(
            t=obs.t,
            kind=self._classify(obs),
            cue=cue,
            roll_deg=math.degrees(self._fr),
            pitch_deg=math.degrees(self._fp),
            height=obs.base_height,
            latency_s=round(delay, 3),
        )
        return True

    def update(self, obs: Observation) -> bool:
        """输入一帧观测；返回是否（在本帧）确认摔倒。锁存后恒返回 True。"""
        if self._init:
            self._fr += self._alpha * (obs.roll - self._fr)
            self._fp += self._alpha * (obs.pitch - self._fp)
        else:
            self._fr, self._fp, self._init = obs.roll, obs.pitch, True

        if abs(self._fr) > abs(self._peak_roll):
            self._peak_roll = self._fr
        if abs(self._fp) > abs(self._peak_pitch):
            self._peak_pitch = self._fp

        # 回差跟踪：只有连续 recover_dwell_s 满足 |roll|<recover_roll 且 |pitch|<recover_pitch 才算退出迟滞带
        in_rec = abs(self._fr) < self.rec_roll and abs(self._fp) < self.rec_pitch
        stable = in_rec and (
            not self.use_height or obs.base_height > self.stable_height
        )
        if in_rec:
            if self._rec_since is None:
                self._rec_since = obs.t
            if obs.t - self._rec_since >= self.rec_dwell:
                self.in_hysteresis_band = False
        else:
            self._rec_since = None

        # 只有“姿态回正且高度正常”持续足够久，才认为上一次扰动结束。
        # 完整 360° 翻转中机身会短暂扫过小角度，但高度仍低，不能清掉
        # 起始姿态/起始时刻，否则会把全翻误分类成 flat。
        if stable and obs.t - (self._rec_since or obs.t) >= self.rec_dwell:
            if not self.latched:
                self._first_bad = None
                self._onset_rpy = None
                self._peak_roll = 0.0
                self._peak_pitch = 0.0

        if self.latched:
            self.in_hysteresis_band = True
            return True

        att_bad = abs(self._fr) > self.roll_thr or abs(self._fp) > self.pitch_thr
        tilt = max(abs(self._fr), abs(self._fp))
        h_bad = self.use_height and obs.base_height < self.height_min

        # 记录「首个判据越限」的时刻，用于区分「确认时延」与「运行到此刻的时间」
        if att_bad or h_bad or tilt > self.fast_thr:
            if self._first_bad is None:
                self._first_bad = obs.t
            if self._onset_rpy is None:
                self._onset_rpy = (obs.roll, obs.pitch, obs.yaw)

        # c0) 高速翻转通道：姿态越限 + 高角速度，短驻留即确认。
        omega_norm = math.sqrt(sum(float(w) * float(w) for w in obs.omega))
        if att_bad and omega_norm >= self.rate_fast_thr:
            if self._rate_fast_since is None:
                self._rate_fast_since = obs.t
            if obs.t - self._rate_fast_since >= self.rate_fast_dwell:
                return self._confirm(obs, "rate_fast")
        else:
            self._rate_fast_since = None

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
