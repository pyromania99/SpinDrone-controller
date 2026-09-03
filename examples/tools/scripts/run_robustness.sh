#!/usr/bin/env bash
#
# run_robustness.sh — drive the robustness axes across every controller.
#
# The sweeps in sweeps/ search for gains. This drives sweeps/robustness_sweep.py,
# which does not search: it takes configurations already on the Pareto front and
# degrades the world around them until they stop flying.
#
# Usage
#   ./run_robustness.sh --axis inertia                 # all three controllers
#   ./run_robustness.sh --axis wind --wind-profile gust
#   ./run_robustness.sh --axis inertia --only eigen,pd
#   ./run_robustness.sh --axis lag --dry-run
#
# Each controller writes results/robustness/<controller>/<axis>/ with a
# long-format robustness_archive.csv and a reduced breakdown.csv. A failing
# controller does not stop the others.
#
# Prerequisite: results/pareto_top/pareto_top.json must be current. It is the
# only source of operating points, and a robustness number is meaningless
# unless you know which configuration it describes — the driver refuses to
# start rather than fall back to a default.
#
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
SWEEP="${REPO_ROOT}/examples/sweeps/robustness_sweep.py"
RESULTS="${REPO_ROOT}/examples/results"

: "${ISAACSIM_PATH:=${HOME}/isaacsim}"
ISAAC_PY="${ISAACSIM_PATH}/python.sh"

AXIS=""
ONLY=""
DRY_RUN=0
DURATION=12
TOP=3
MOTOR_MOMENTS="0.0 2.7e-7"
EXTRA=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --axis)           AXIS="$2"; shift 2 ;;
        --only)           ONLY="$2"; shift 2 ;;
        --dry-run)        DRY_RUN=1; shift ;;
        --duration)       DURATION="$2"; shift 2 ;;
        --top)            TOP="$2"; shift 2 ;;
        --motor-moments)  MOTOR_MOMENTS="$2"; shift 2 ;;
        --wind-profile)   EXTRA="${EXTRA} --wind-profile $2"; shift 2 ;;
        --magnitudes)     EXTRA="${EXTRA} --magnitudes $2"; shift 2 ;;
        --seeds)          EXTRA="${EXTRA} --seeds $2"; shift 2 ;;
        -h|--help)        sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
done

[[ -n "$AXIS" ]] || { echo "--axis is required (inertia|mass|lag|wind)" >&2; exit 2; }
[[ -x "$ISAAC_PY" ]] || { echo "Isaac python not found: $ISAAC_PY" >&2; exit 1; }
[[ -f "${RESULTS}/pareto_top/pareto_top.json" ]] || {
    echo "missing ${RESULTS}/pareto_top/pareto_top.json — run" >&2
    echo "  python3 ${REPO_ROOT}/examples/tools/evaluations/select_pareto_top.py" >&2
    exit 1
}

# Isaac Sim ships Python 3.11 and conflicts with a sourced ROS 2 environment.
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
LOG_DIR="${RESULTS}/robustness_logs/${STAMP}"
mkdir -p "$LOG_DIR"

echo "=============================================================="
echo " Robustness run — ${STAMP}"
echo " axis      : ${AXIS}${EXTRA}"
echo " duration  : ${DURATION}s per run   top ${TOP} configs"
echo " km ladder : ${MOTOR_MOMENTS}"
echo " logs      : ${LOG_DIR}"
[[ $DRY_RUN -eq 1 ]] && echo " MODE      : DRY RUN"
echo "=============================================================="

declare -a NAMES STATUS SECONDS_EACH
OVERALL=0

for ctrl in eigen pd mpc; do
    selected "$ctrl" || continue
    log="${LOG_DIR}/${ctrl}.log"
    # shellcheck disable=SC2086
    cmd="${SWEEP} --controller ${ctrl} --axis ${AXIS} \
--motor-moments ${MOTOR_MOMENTS} --top ${TOP} --duration ${DURATION} \
${EXTRA} --headless"

    echo
    echo "--------------------------------------------------------------"
    echo ">>> ${ctrl}  —  ${AXIS}"
    echo "    log: ${log}"
    echo "    \$ isaac_python ${cmd}"
    echo "--------------------------------------------------------------"

    if [[ $DRY_RUN -eq 1 ]]; then
        NAMES+=("$ctrl"); STATUS+=("dry-run"); SECONDS_EACH+=(0); continue
    fi

    start=$SECONDS
    # shellcheck disable=SC2086
    ( cd "$REPO_ROOT" && isaac_python $cmd ) >"$log" 2>&1
    status=$?
    elapsed=$((SECONDS - start))

    NAMES+=("$ctrl"); SECONDS_EACH+=("$elapsed")
    if [[ $status -eq 0 ]]; then
        STATUS+=("ok"); echo "    done in ${elapsed}s"
    else
        STATUS+=("FAILED(${status})"); OVERALL=1
        echo "    FAILED (exit ${status}) after ${elapsed}s — see ${log}"
        tail -n 15 "$log" | sed 's/^/      /'
    fi
done

echo
echo "=============================================================="
printf '%-10s %-14s %s\n' CONTROLLER STATUS ELAPSED
for i in "${!NAMES[@]}"; do
    printf '%-10s %-14s %ss\n' "${NAMES[$i]}" "${STATUS[$i]}" "${SECONDS_EACH[$i]}"
done
echo "logs: ${LOG_DIR}"
echo "=============================================================="
exit $OVERALL
