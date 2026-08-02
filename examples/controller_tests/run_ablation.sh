#!/usr/bin/env bash
#
# run_ablation.sh — march every tuning parameter outward, for all controllers.
#
# Prerequisite: results/ablation/<ctrl>/mm_<km>/reference.json, written by
#   python3 evaluations/select_reference.py
# The march is defined relative to that point, so it refuses to start without it.
#
# Usage
#   ./run_ablation.sh                       # all three controllers, both km
#   ./run_ablation.sh --only eigen,pd
#   ./run_ablation.sh --dry-run
#   ./run_ablation.sh --max-steps 12        # shorter arms
#
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SWEEPS="${SCRIPT_DIR}/sweeps"
RESULTS="${SCRIPT_DIR}/results"

: "${ISAACSIM_PATH:=${HOME}/isaacsim}"
ISAAC_PY="${ISAACSIM_PATH}/python.sh"

ONLY=""
DRY_RUN=0
DURATION=12
SLOTS=16
MAX_STEPS=24
MOTOR_MOMENTS="0.0 2.7e-7"
EXTRA=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --only)          ONLY="$2"; shift 2 ;;
        --dry-run)       DRY_RUN=1; shift ;;
        --duration)      DURATION="$2"; shift 2 ;;
        --slots)         SLOTS="$2"; shift 2 ;;
        --max-steps)     MAX_STEPS="$2"; shift 2 ;;
        --motor-moments) MOTOR_MOMENTS="$2"; shift 2 ;;
        --ratio)         EXTRA="${EXTRA} --ratio $2"; shift 2 ;;
        --extend-factor) EXTRA="${EXTRA} --extend-factor $2"; shift 2 ;;
        -h|--help)       sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
done

[[ -x "$ISAAC_PY" ]] || { echo "Isaac python not found: $ISAAC_PY" >&2; exit 1; }

missing=0
for ctrl in eigen pd mpc; do
    [[ -n "$ONLY" && ",${ONLY}," != *",$ctrl,"* ]] && continue
    for mm in $MOTOR_MOMENTS; do
        d=$(printf 'mm_%.1e' "$mm")
        [[ -f "${RESULTS}/ablation/${ctrl}/${d}/reference.json" ]] || {
            echo "missing reference: ${RESULTS}/ablation/${ctrl}/${d}/reference.json" >&2
            missing=1; }
    done
done
[[ $missing -eq 0 ]] || {
    echo "run: python3 ${SCRIPT_DIR}/evaluations/select_reference.py" >&2; exit 1; }

isaac_python() {
    env -u ROS_VERSION -u ROS_PYTHON_VERSION -u ROS_DISTRO \
        -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u PYTHONPATH \
        -u CMAKE_PREFIX_PATH \
        LD_LIBRARY_PATH="$(echo "${LD_LIBRARY_PATH:-}" | tr ':' '\n' \
            | grep -v '^/opt/ros/' | paste -sd':' -)" \
        "$ISAAC_PY" "$@"
}

selected() { [[ -z "$ONLY" ]] && return 0; [[ ",${ONLY}," == *",$1,"* ]]; }

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${RESULTS}/ablation_logs/${STAMP}"
mkdir -p "$LOG_DIR"

echo "=============================================================="
echo " Ablation march — ${STAMP}"
echo " ${SLOTS} slots, ${DURATION}s per run, up to ${MAX_STEPS} steps per arm"
echo " km ladder : ${MOTOR_MOMENTS}"
echo " results   : ${RESULTS}/ablation"
echo " logs      : ${LOG_DIR}"
[[ $DRY_RUN -eq 1 ]] && echo " MODE      : DRY RUN"
echo "=============================================================="

declare -a NAMES STATUS ELAPSED
OVERALL=0

for ctrl in eigen pd mpc; do
    selected "$ctrl" || continue
    log="${LOG_DIR}/${ctrl}.log"
    cmd="${SWEEPS}/ablation_explore.py --controller ${ctrl} \
--motor-moments ${MOTOR_MOMENTS} --slots ${SLOTS} --duration ${DURATION} \
--max-steps ${MAX_STEPS} ${EXTRA} --headless"

    echo
    echo "--------------------------------------------------------------"
    echo ">>> ${ctrl}"
    echo "    log: ${log}"
    echo "    \$ isaac_python ${cmd}"
    echo "--------------------------------------------------------------"

    if [[ $DRY_RUN -eq 1 ]]; then
        NAMES+=("$ctrl"); STATUS+=("dry-run"); ELAPSED+=(0); continue
    fi

    start=$SECONDS
    # shellcheck disable=SC2086
    ( cd "$REPO_ROOT" && isaac_python $cmd ) >"$log" 2>&1
    status=$?
    took=$((SECONDS - start))

    NAMES+=("$ctrl"); ELAPSED+=("$took")
    if [[ $status -eq 0 ]]; then
        STATUS+=("ok"); echo "    done in ${took}s"
        grep -ah "noise floor\|arms broke" "$log" | sed 's/.*\] //' | sed 's/^/      /'
    else
        STATUS+=("FAILED(${status})"); OVERALL=1
        echo "    FAILED (exit ${status}) after ${took}s — see ${log}"
        tail -n 15 "$log" | sed 's/^/      /'
    fi
done

echo
echo "=============================================================="
printf '%-8s %-14s %s\n' CONTROLLER STATUS ELAPSED
for i in "${!NAMES[@]}"; do
    printf '%-8s %-14s %ss\n' "${NAMES[$i]}" "${STATUS[$i]}" "${ELAPSED[$i]}"
done
echo "results: ${RESULTS}/ablation"
echo "=============================================================="
exit $OVERALL
