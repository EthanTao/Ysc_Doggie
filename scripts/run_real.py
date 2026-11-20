#!/usr/bin/env python3
"""实机常驻监控与恢复（ROS2）。

安全设计：**默认只读干跑**。只有显式加 `--enable-actuation` 才会向官方状态机
下发 DAMPING / LIEDOWN / STANDUP 指令；干跑模式只打印「将要下发什么」。

典型用法：
    # 1) 只读验证话题与姿态（吊架保护，不下发指令）
    python scripts/run_real.py --duration 60

    # 2) 静态零偏标定 2 s（机器人直立静置）
    python scripts/run_real.py --duration 30 --calibrate 2

    # 3) 受控倾斜/恢复（必须先在 config/recovery.yaml 配好 state_cmd_topic）
    python scripts/run_real.py --duration 300 --enable-actuation

按 docs/09-real-machine.md 的四阶段 SOP 执行，现场保持急停可用。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from doggie.core.detector import FallDetector  # noqa: E402
from doggie.core.recovery_fsm import RecoveryFSM  # noqa: E402
from doggie.core.safety import SafetyMonitor  # noqa: E402
from doggie.core.types import Mode  # noqa: E402
from doggie.io.ros2_backend import Ros2Backend  # noqa: E402
from doggie.supervisor import Supervisor  # noqa: E402
from doggie.utils.logger import TrialLogger, load_config  # noqa: E402


def calibrate(backend: Ros2Backend, seconds: float) -> None:
    print(f"静态零偏标定：保持机器人直立静止 {seconds:.1f} s ...")
    rolls: list[float] = []
    pitches: list[float] = []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        backend.tick()
        rp = backend.raw_attitude()
        if rp is not None:
            rolls.append(rp[0])
            pitches.append(rp[1])
    if not rolls:
        print("标定失败：没有收到 IMU 数据")
        return
    roll = sum(rolls) / len(rolls)
    pitch = sum(pitches) / len(pitches)
    backend.set_attitude_bias(roll, pitch)
    print(
        f"零偏已扣除：roll {roll * 57.2958:+.2f}°  pitch {pitch * 57.2958:+.2f}° "
        f"（{len(rolls)} 帧）"
    )


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default=str(ROOT / "config" / "recovery.yaml"))
    ap.add_argument("--duration", type=float, default=120.0, help="运行时长（秒）")
    ap.add_argument("--enable-actuation", action="store_true", help="允许下发运动指令")
    ap.add_argument("--yes", "-y", action="store_true", help="跳过启用指令前的确认")
    ap.add_argument("--calibrate", type=float, default=0.0, help="静态零偏标定秒数")
    ap.add_argument("--log-dir", default="", help="日志目录；留空自动生成")
    ap.add_argument("--det-hz", type=float, default=200.0)
    args = ap.parse_args()

    cfg = load_config(Path(args.config))
    dry_run = not args.enable_actuation
    run_id = f"real_{datetime.now():%Y%m%d_%H%M%S}"
    run_dir = Path(args.log_dir) if args.log_dir else ROOT / "data" / "raw" / run_id

    try:
        import rclpy
    except ImportError:
        raise SystemExit(
            "未找到 rclpy：请在机器人/ROS2 环境（source /opt/ros/<distro>/setup.bash）中运行；\n"
            "纯仿真回归请用 python -m pytest -q 或 scripts/watch_sim.py。"
        )

    rclpy.init()
    backend = None
    stopped = False
    try:
        backend = Ros2Backend(cfg, dry_run=dry_run)
        for note in backend.notes:
            print(f"[warn] {note}")

        print(f"等待 /IMU_DATA 与 /JOINTS_DATA ...（dry_run={dry_run}）", flush=True)
        if not backend.wait_for_data(timeout=10.0):
            print("10 s 内没有收到完整传感器数据，退出。请检查话题名与消息类型。")
            return

        if args.calibrate > 0:
            calibrate(backend, args.calibrate)

        detector = FallDetector(cfg["detector"])
        fsm = RecoveryFSM(cfg["fsm"], detector)
        safety = SafetyMonitor(cfg["safety"])

        def on_reset(t: float) -> None:
            ok = fsm.manual_reset(t)
            safety.reset()
            print(f"[reset] manual_reset -> {ok}", flush=True)

        backend.on_reset = on_reset
        logger = TrialLogger(run_dir)
        sup = Supervisor(
            backend,
            detector,
            fsm,
            logger=logger,
            safety=safety,
            status_cb=backend.publish_status,
            status_hz=20.0,
        )

        if not dry_run and not args.yes:
            print("\n即将使能运动指令：DAMPING / LIEDOWN / STANDUP 会真实驱动关节。")
            print("确认软垫、吊架、急停手柄已就位，且现场有保护人员。")
            if input("输入 yes 继续：").strip().lower() != "yes":
                print("已取消。")
                return

        print(f"开始运行 {args.duration:.0f} s（Ctrl+C 退出，退出前会切回阻尼）")
        try:
            result = sup.run(args.duration, det_hz=args.det_hz, continuous=True)
            stopped = True
        except KeyboardInterrupt:
            print("\n收到 Ctrl+C，提前结束；已记录的数据仍会落盘。")
            result = fsm.summary()
            result["interrupted"] = True

        print("\n===== 实机运行结果 =====")
        print(json.dumps(result, indent=2, ensure_ascii=False))
        logger.finish(
            {
                "run_id": run_id,
                "date": datetime.now().isoformat(timespec="seconds"),
                "backend": "ros2",
                "dry_run": dry_run,
                "config_hash_note": "see config/recovery.yaml",
                "result": result,
            }
        )
        print(f"\nlogs : {run_dir}")
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，退出。")
    finally:
        if backend is not None:
            try:
                if not dry_run:
                    print("[safety] 退出前切回阻尼 (JointDamping)")
                    backend.set_mode(Mode.DAMPING)
                    backend.spin(0.2)
            except Exception as exc:  # 退出路径不再抛异常
                print(f"[warn] 退出切阻尼失败: {exc}")
            backend.shutdown()
        rclpy.try_shutdown()
        if not stopped and not dry_run:
            print("[warn] 运行未正常结束，请确认机器人当前状态。")


if __name__ == "__main__":
    main()
