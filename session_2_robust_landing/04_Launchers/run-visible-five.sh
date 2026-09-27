#!/usr/bin/env bash
set -u

PX4_ROOT=/mnt/f/PX4
export PX4_POST_DISARM_HOLD_S=10
BASE="$PX4_ROOT/research/analysis/run_randomized_regression.sh"
BATCH_STATUS="$PX4_ROOT/internal/visible-five-status.txt"
BATCH_LOG="$PX4_ROOT/logs/visible-five-current.log"
RESULTS="$PX4_ROOT/logs/visible-five-results.tsv"
TMP_SCRIPT="$(mktemp /tmp/run-randomized-gui.XXXXXX.sh)"

cleanup() {
    rm -f "$TMP_SCRIPT"
    bash "$PX4_ROOT/internal/stop-px4.sh" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

sed 's/px4-sitl-job.sh" headless/px4-sitl-job.sh" gui/' "$BASE" > "$TMP_SCRIPT"
chmod +x "$TMP_SCRIPT"

tests=(
    "random-seed-7|random|7"
    "random-seed-42|random|42"
    "random-seed-2026|random|2026"
    "random-seed-31415|random|31415"
    "manual-boundary|manual|1|-1|5|2|-2|90"
)

: > "$BATCH_LOG"
printf 'test\tresult\tarchive\n' > "$RESULTS"
pass=0
fail=0
index=0

for spec in "${tests[@]}"; do
    index=$((index + 1))
    IFS='|' read -r -a fields <<< "$spec"
    label="${fields[0]}"
    args=("${fields[@]:1}")

    {
        echo "RUNNING"
        echo "TEST=$index/5"
        echo "LABEL=$label"
        echo "PASS=$pass"
        echo "FAIL=$fail"
        echo "TIME=$(date -Is)"
    } > "$BATCH_STATUS"
    echo "=== visible test $index/5: $label started $(date -Is) ===" >> "$BATCH_LOG"

    bash "$TMP_SCRIPT" "${args[@]}" >> "$BATCH_LOG" 2>&1
    rc=$?
    archive="$(tr -d '\r\n' < "$PX4_ROOT/logs/last-research-run.txt" 2>/dev/null || true)"
    if [[ "$rc" -eq 0 ]]; then
        pass=$((pass + 1))
        printf '%s\tPASS\t%s\n' "$label" "$archive" >> "$RESULTS"
        echo "=== visible test $index/5: $label PASS $archive ===" >> "$BATCH_LOG"
    else
        fail=$((fail + 1))
        printf '%s\tFAIL(rc=%s)\t%s\n' "$label" "$rc" "$archive" >> "$RESULTS"
        echo "=== visible test $index/5: $label FAIL rc=$rc ===" >> "$BATCH_LOG"
    fi
done

{
    echo "COMPLETE"
    echo "TEST=5/5"
    echo "LABEL=finished"
    echo "PASS=$pass"
    echo "FAIL=$fail"
    echo "RESULTS=$RESULTS"
    echo "TIME=$(date -Is)"
} > "$BATCH_STATUS"

if [[ "$fail" -ne 0 ]]; then
    exit 1
fi
