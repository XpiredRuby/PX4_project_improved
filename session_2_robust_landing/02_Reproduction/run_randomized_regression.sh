#!/usr/bin/env bash
set -Eeuo pipefail

PX4_ROOT=/mnt/f/PX4
STATUS="$PX4_ROOT/internal/randomized-regression-status.txt"
RUN_LOG="$PX4_ROOT/logs/randomized-regression-current.log"
SITL_PID=
ARCHIVE=

write_status() {
    {
        echo "$1"
        echo "ARCHIVE=$ARCHIVE"
        echo "DETAIL=${2:-}"
        echo "TIME=$(date -Is)"
    } > "$STATUS"
}

cleanup() {
    bash "$PX4_ROOT/internal/stop-px4.sh" >/dev/null 2>&1 || true
    if [[ -n "$SITL_PID" ]]; then
        wait "$SITL_PID" 2>/dev/null || true
    fi
}

fail() {
    rc=$?
    write_status FAILED "exit_code=$rc; inspect $RUN_LOG"
    cleanup
    exit "$rc"
}
trap fail ERR INT TERM
trap cleanup EXIT

MODE="${1:-random}"
shift || true
case "$MODE" in
    random)
        export PX4_INIT_MODE=random
        if [[ $# -ge 1 && -n "$1" ]]; then
            export PX4_SPAWN_SEED="$1"
        else
            unset PX4_SPAWN_SEED || true
        fi
        ;;
    manual)
        [[ $# -eq 6 ]]
        export PX4_INIT_MODE=manual
        export PX4_SPAWN_X="$1"
        export PX4_SPAWN_Y="$2"
        export PX4_SPAWN_Z="$3"
        export PX4_SPAWN_ROLL_DEG="$4"
        export PX4_SPAWN_PITCH_DEG="$5"
        export PX4_SPAWN_YAW_DEG="$6"
        ;;
    *)
        echo "usage: $0 random [seed] | manual x y z roll_deg pitch_deg yaw_deg" >&2
        exit 2
        ;;
esac

: > "$RUN_LOG"
write_status STARTING_SITL "mode=$MODE"
cleanup
: > "$PX4_ROOT/logs/px4-sitl-current.log"
bash "$PX4_ROOT/internal/px4-sitl-job.sh" headless >> "$RUN_LOG" 2>&1 &
SITL_PID=$!

ready=0
for _ in $(seq 1 400); do
    if grep -Fq 'Ready for takeoff!' "$PX4_ROOT/logs/px4-sitl-current.log" 2>/dev/null; then
        ready=1
        break
    fi
    kill -0 "$SITL_PID" 2>/dev/null || break
    sleep 0.1
done
[[ "$ready" -eq 1 ]]
kill -0 "$SITL_PID" 2>/dev/null
sleep 1

write_status RUNNING_MISSION "mode=$MODE"
timeout 280s bash "$PX4_ROOT/internal/research-mission-job.sh" \
    jerk randomized >> "$RUN_LOG" 2>&1
ARCHIVE="$(tr -d '\r\n' < "$PX4_ROOT/logs/last-research-run.txt")"
[[ -d "$ARCHIVE" ]]
grep -Fq 'SUCCESS' "$PX4_ROOT/internal/research-mission-status.txt"
cp "$PX4_ROOT/logs/px4-sitl-current.log" "$ARCHIVE/px4-sitl.log"
grep -Fq 'Disarmed by landing' "$ARCHIVE/px4-sitl.log"
grep -Fq '[runner] PX4 native landing/disarm confirmed' "$ARCHIVE/controller.log"
[[ -f "$ARCHIVE/spawn_config.json" ]]
python3 "$PX4_ROOT/research/analysis/analyze_randomized_run.py" \
    "$ARCHIVE" >> "$RUN_LOG" 2>&1
[[ -f "$ARCHIVE/randomized_metrics.json" ]]

write_status COMPLETE "SUCCESS with randomized initialization and native landing"
printf '%s\n' "$ARCHIVE"
