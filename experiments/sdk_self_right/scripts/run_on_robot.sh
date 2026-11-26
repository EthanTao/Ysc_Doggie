#!/usr/bin/env bash
# Robot-local wrapper for lite3_interruptible_recovery (ROS 2 Foxy ARM64).
# Auto-recovery is OFF by default at boot; start explicitly with `start` and
# cancel anytime with `stop` / `cancel` (or automatically ends at shutdown).
set -euo pipefail

WS="${LITE3_INTERRUPTIBLE_WS:-$HOME/lite3_interruptible_ws}"
AUDIT_DIR="${LITE3_SDK_AUDIT_DIR:-$HOME/.lite3-sdk-audits}"
LOCK_DIR="${LITE3_SDK_LOCK_DIR:-$HOME/.lite3-recovery-attempts}"
PID_FILE="/tmp/lite3-auto-recovery.pid"
LOG_LINK="/tmp/lite3-auto-recovery.latest.jsonl"
PROFILE_DEFAULT="$WS/recovery_sdk/config/left_roll_profile.json"

usage() {
  cat <<'EOF'
Usage: run_on_robot.sh <subcommand> [extra args...]

Persistent service commands (OFF by default at boot):
  start [--stable-wait-ms 2000] [--allow-prone-shortcut] [--kp-scale 1.0]
                                       Enable continuous auto-recovery in background until
                                       robot shutdown or `stop`/`cancel`
  stop | cancel                        Disable continuous auto-recovery immediately
  status                               Check whether auto-recovery is active & show recent events

Single-trial / diagnostic commands:
  dry-run [--duration 60]              Read-only telemetry & STOP channel check (no motion)
  stop-test [--duration 30]            Sling-only micro-motion STOP & <=20ms damping test
  recover [--kp-scale 1.0] [--stand-mode native-handoff|sdk] [--allow-prone-shortcut]
                                       Single-shot recovery trial (creates durable lock)
  archive-lock                         Archive unfinished-sdk-recovery after on-site inspection
EOF
}

cmd="${1:-}"
if [[ -z "$cmd" || "$cmd" == "-h" || "$cmd" == "--help" ]]; then
  usage
  exit 0
fi
shift

is_running() {
  [[ -f "$PID_FILE" ]] || return 1
  local pid
  pid="$(cat "$PID_FILE" 2>/dev/null || true)"
  [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null
}

case "$cmd" in
  archive-lock)
    marker="$LOCK_DIR/unfinished-sdk-recovery"
    if [[ ! -e "$marker" ]]; then
      echo "No active lock at $marker"
      exit 0
    fi
    dst="$marker.reviewed-$(date -u +%Y%m%dT%H%M%SZ)"
    mv -T -- "$marker" "$dst"
    echo "Archived lock to $dst"
    exit 0
    ;;
  stop|cancel)
    if ! is_running; then
      rm -f "$PID_FILE"
      echo "Auto-recovery is already OFF."
      exit 0
    fi
    pid="$(cat "$PID_FILE")"
    echo "Stopping auto-recovery (PID $pid)..."
    kill -TERM "$pid" 2>/dev/null || true
    for _ in $(seq 1 40); do
      if ! kill -0 "$pid" 2>/dev/null; then
        rm -f "$PID_FILE"
        echo "Auto-recovery stopped cleanly."
        exit 0
      fi
      sleep 0.25
    done
    echo "Process still exiting after 10s; force killing PID $pid..."
    kill -KILL "$pid" 2>/dev/null || true
    rm -f "$PID_FILE"
    exit 0
    ;;
  status)
    if is_running; then
      pid="$(cat "$PID_FILE")"
      echo "Auto-recovery: ON (PID $pid)"
      if [[ -L "$LOG_LINK" && -f "$LOG_LINK" ]]; then
        echo "Audit log: $(readlink -f "$LOG_LINK")"
        echo "Recent events:"
        grep -v '"event":"sample"' "$LOG_LINK" | tail -n 10 || true
      fi
    else
      echo "Auto-recovery: OFF"
    fi
    exit 0
    ;;
esac

set +u
source /opt/ros/foxy/setup.bash
source "$WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

mkdir -p "$AUDIT_DIR"
stamp="$(date -u +%Y%m%dT%H%M%S%N)"

case "$cmd" in
  start)
    if is_running; then
      echo "Auto-recovery is already ON (PID $(cat "$PID_FILE"))."
      exit 0
    fi
    audit_file="$AUDIT_DIR/auto-service-$stamp.jsonl"
    console_log="$AUDIT_DIR/auto-service-$stamp.console.log"
    bin="$WS/install/lite3_transfer/lib/lite3_transfer/lite3_interruptible_recovery"
    nohup "$bin" \
      --mode recover \
      --continuous \
      --stand-mode native-handoff \
      --confirm-sling \
      --profile "$PROFILE_DEFAULT" \
      --lock-dir "$LOCK_DIR" \
      --audit "$audit_file" \
      "$@" >"$console_log" 2>&1 < /dev/null &
    pid=$!
    echo "$pid" > "$PID_FILE"
    ln -sfn "$audit_file" "$LOG_LINK"
    sleep 1.0
    if ! kill -0 "$pid" 2>/dev/null; then
      rm -f "$PID_FILE"
      echo "Failed to start auto-recovery. Console output:" >&2
      cat "$console_log" >&2
      exit 1
    fi
    echo "Auto-recovery STARTED (PID $pid)."
    echo "  Audit log: $audit_file"
    echo "  To check:  bash $WS/scripts/run_on_robot.sh status"
    echo "  To cancel: bash $WS/scripts/run_on_robot.sh stop"
    ;;
  dry-run)
    exec ros2 run lite3_transfer lite3_interruptible_recovery \
      --mode dry-run \
      --audit "$AUDIT_DIR/dry-run-$stamp.jsonl" \
      "$@"
    ;;
  stop-test)
    exec ros2 run lite3_transfer lite3_interruptible_recovery \
      --mode stop-test \
      --confirm-sling \
      --lock-dir "$LOCK_DIR" \
      --audit "$AUDIT_DIR/stop-test-$stamp.jsonl" \
      "$@"
    ;;
  recover)
    exec ros2 run lite3_transfer lite3_interruptible_recovery \
      --mode recover \
      --stand-mode native-handoff \
      --confirm-sling \
      --profile "$PROFILE_DEFAULT" \
      --lock-dir "$LOCK_DIR" \
      --audit "$AUDIT_DIR/recover-$stamp.jsonl" \
      "$@"
    ;;
  *)
    echo "Unknown subcommand: $cmd" >&2
    usage >&2
    exit 2
    ;;
esac
