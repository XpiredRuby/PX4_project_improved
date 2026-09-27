#!/usr/bin/env bash
set -Eeuo pipefail

PX4_ROOT=/mnt/f/PX4
BASE="$PX4_ROOT/research/analysis/run_randomized_regression.sh"
STATUS="$PX4_ROOT/internal/visible-proof-status.txt"
LOG="$PX4_ROOT/logs/visible-proof-current.log"
TMP_SCRIPT="$(mktemp /tmp/run-randomized-gui-proof.XXXXXX.sh)"
export PX4_POST_DISARM_HOLD_S=10

cleanup() {
    rm -f "$TMP_SCRIPT"
}
trap cleanup EXIT INT TERM

sed 's/px4-sitl-job.sh" headless/px4-sitl-job.sh" gui/' "$BASE" > "$TMP_SCRIPT"
chmod +x "$TMP_SCRIPT"
{
    echo "RUNNING"
    echo "SEED=42"
    echo "POST_DISARM_HOLD_S=$PX4_POST_DISARM_HOLD_S"
    echo "TIME=$(date -Is)"
} > "$STATUS"
: > "$LOG"

if bash "$TMP_SCRIPT" random 42 >> "$LOG" 2>&1; then
    archive="$(tr -d '\r\n' < "$PX4_ROOT/logs/last-research-run.txt")"
    {
        echo "COMPLETE"
        echo "SEED=42"
        echo "POST_DISARM_HOLD_S=$PX4_POST_DISARM_HOLD_S"
        echo "ARCHIVE=$archive"
        echo "TIME=$(date -Is)"
    } > "$STATUS"
else
    rc=$?
    {
        echo "FAILED"
        echo "SEED=42"
        echo "EXIT_CODE=$rc"
        echo "TIME=$(date -Is)"
    } > "$STATUS"
    exit "$rc"
fi
