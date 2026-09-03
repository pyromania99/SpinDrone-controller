#!/usr/bin/env bash
#
# run_outer_sweep.sh — genetic search over the OUTER position loop as well as
# the inner one, for all three controllers.
#
# Why this is separate from run_all_sweeps.sh
# ───────────────────────────────────────────
# The outer-loop runs write to the same sweep names as the pinned-outer ones
# (genetic_eigen_sweep, ...), so they would overwrite them. SWEEP_RESULTS_ROOT
# redirects the whole tree to results/outer/ instead, which keeps the pinned
# archives intact as the control arm — the free-vs-pinned delta per controller
# is the result this exercise exists to produce, and it needs both.
#
# Usage
#   ./run_outer_sweep.sh                      # all three controllers
#   ./run_outer_sweep.sh --only eigen,pd
#   ./run_outer_sweep.sh --dry-run
#   ./run_outer_sweep.sh --generations 40 --pop-size 24
#
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
SWEEPS="${REPO_ROOT}/examples/sweeps"
RESULTS="${REPO_ROOT}/examples/results"
OUTER_ROOT="${RESULTS}/outer"

: "${ISAACSIM_PATH:=${HOME}/isaacsim}"
ISAAC_PY="${ISAACSIM_PATH}/python.sh"

ONLY=""
DRY_RUN=0
DURATION=12
POP=16
GENERATIONS=30
# The same two spin rates the robustness axes use, so the configurations this
# produces can be carried straight into them without a task change.
MOTOR_MOMENTS="0.0 2.7e-7"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --only)          ONLY="$2"; shift 2 ;;
        --dry-run)       DRY_RUN=1; shift ;;
        --duration)      DURATION="$2"; shift 2 ;;
        --pop-size)      POP="$2"; shift 2 ;;
        --generations)   GENERATIONS="$2"; shift 2 ;;
        --motor-moments) MOTOR_MOMENTS="$2"; shift 2 ;;
        -h|--help)       sed -n '2,18p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
done

[[ -x "$ISAAC_PY" ]] || { echo "Isaac python not found: $ISAAC_PY" >&2; exit 1; }

isaac_python() {
    env -u ROS_VERSION -u ROS_PYTHON_VERSION -u ROS_DISTRO \
        -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u PYTHONPATH \
        -u CMAKE_PREFIX_PATH \
        SWEEP_RESULTS_ROOT="$OUTER_ROOT" \
        LD_LIBRARY_PATH="$(echo "${LD_LIBRARY_PATH:-}" | tr ':' '\n' \
            | grep -v '^/opt/ros/' | paste -sd':' -)" \
        "$ISAAC_PY" "$@"
}

selected() { [[ -z "$ONLY" ]] && return 0; [[ ",${ONLY}," == *",$1,"* ]]; }

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${RESULTS}/outer_logs/${STAMP}"
mkdir -p "$LOG_DIR" "$OUTER_ROOT"

echo "=============================================================="
echo " Outer-loop genetic sweep — ${STAMP}"
echo " genes     : inner + wn_xy, zeta_xy (+ ki_xy, ki_z for mpc)"
echo " pop ${POP}, ${GENERATIONS} generations, ${DURATION}s per run"
echo " km ladder : ${MOTOR_MOMENTS}"
echo " results   : ${OUTER_ROOT}   (pinned archives untouched)"
echo " logs      : ${LOG_DIR}"
[[ $DRY_RUN -eq 1 ]] && echo " MODE      : DRY RUN"
echo "=============================================================="

declare -a NAMES STATUS ELAPSED
OVERALL=0

for ctrl in eigen pd mpc; do
    selected "$ctrl" || continue
    log="${LOG_DIR}/${ctrl}.log"
    cmd="${SWEEPS}/genetic_sweep.py --controller ${ctrl} --sweep-outer \
--motor-moments ${MOTOR_MOMENTS} --pop-size ${POP} \
--generations ${GENERATIONS} --duration ${DURATION} --headless"

    echo
    echo "--------------------------------------------------------------"
    echo ">>> ${ctrl}  —  inner + outer loop"
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
echo "results: ${OUTER_ROOT}"
echo "logs:    ${LOG_DIR}"
echo "=============================================================="
exit $OVERALL
