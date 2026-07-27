#!/usr/bin/env bash
#
# run_all_sweeps.sh — drive every controller sweep from one place.
#
# The sweeps are long GPU jobs that each need Isaac Sim's own interpreter and a
# ROS-free environment (see the isaac_run function in ~/.bashrc — that is a
# shell function, not an executable, so it cannot be called from a script; the
# relevant parts are reproduced in isaac_python() below).
#
# Usage
#   ./run_all_sweeps.sh --list                  # show the sweeps and exit
#   ./run_all_sweeps.sh --dry-run               # print commands, run nothing
#   ./run_all_sweeps.sh                         # run all of them, headless
#   ./run_all_sweeps.sh --only eigen-ab,pd-grid # run a subset
#   ./run_all_sweeps.sh --no-headless           # watch it in the viewport
#   ./run_all_sweeps.sh --duration 25           # override sim seconds per run
#   ./run_all_sweeps.sh --goal-radius 3         # longer lateral leg per run
#   ./run_all_sweeps.sh --spawn-altitude 1.5    # old air start (no takeoff)
#
# Each sweep writes its own log under results/sweep_logs/<stamp>/. A failing
# sweep does not stop the others; the summary at the end reports which failed.
#
# The task: every run is a ground start, a takeoff, and a translation to a goal
# GOAL_RADIUS metres from the spawn XY in a random (seeded) direction. Runs used
# to spawn AT their goal, which left zero initial position error — the gains
# being swept could not move the score at all, and it collapsed onto the
# steady-state altitude droop. See examples/utils/sweep_task.py.
#
# Duration note: 12 s, chosen from a measured sweep of window lengths.
#   - yaw spin-up is first-order with τ ≈ 1.8 s, so 12 s is 6.7τ and the
#     steady-state spin estimate is within 0.3 % of its 25 s value
#   - the altitude loop has a much slower, lightly damped mode with a ~11.4 s
#     period; 12 s covers one full cycle, 6 s covered barely half
#   - discrimination between a good and a bad config (score ratio) is 84x at
#     12 s against 37x at 6 s and 127x at 25 s
# Those numbers were measured on the old zero-offset task. The window must also
# outlast the gyroscopic coning instability, whose onset is set by the yaw
# spin-up (~3-4 s) and which took 9 s to become terminal from a standing start
# at km=2e-7. A 2 m lateral leg excites the mode ~3 s earlier, so 12 s clears it
# with roughly 2x margin — thin, but the score need not be sensitive: a crash is
# charged the arena diagonal for its missing tail and lands at 20-38 against
# 0.05-0.16 for a healthy run.
# NOTE: --duration is honoured properly now. It previously did not survive
# contact with the loop: world.step() advances one physics step (1/250 s) while
# the loops counted 1/60 s per iteration, so --duration 25 delivered 6 s.
#
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SWEEPS="${SCRIPT_DIR}/sweeps"
RESULTS="${SCRIPT_DIR}/results"

: "${ISAACSIM_PATH:=${HOME}/isaacsim}"
ISAAC_PY="${ISAACSIM_PATH}/python.sh"

# ── Defaults ─────────────────────────────────────────────────────────────────
HEADLESS="--headless"
DRY_RUN=0
DO_LIST=0
ONLY=""
ARCHIVE=1
DURATION=12
BACKFILL=1

# Task geometry, passed to every sweep so they cannot drift apart.
# SPAWN_ALTITUDE empty means "use the sweep default", which is a ground start.
GOAL_RADIUS=2.0
GOAL_SEED=7
SPAWN_ALTITUDE=""

# Motor moments = the spin knob. Measured: r_ss ≈ 4.9e7 · km, so this ladder
# spans 0 → 29.3 rad/s of steady-state spin.
MOTOR_MOMENTS="0.0 1e-7 2e-7 2.7e-7 4e-7 6e-7"

# (a, b) grid for the Eigen sweeps. b runs past the fastest spin rate
# (29.3 rad/s at km=6e-7) so the b≈spin-rate region is inside the grid rather
# than beyond its ceiling, which is what the old b≤15 box could not express.
A_VALUES="2 6 10 15 20 30 40 60"
B_VALUES="0 4 8 12 16 20 24 28 32 40"

# Refined b axis for eigen-ab-fine. The wide sweep answered the question it was
# built for — b does NOT track spin rate; survival sits at b = 0-8 for every
# motor moment, and b = 12-24 is dead everywhere. What it could not resolve is
# the bottom of the range: the genetic search's best individuals cluster at
# b ≈ 1.3-2.7, and a grid with 4.0 spacing steps straight over that. These
# values concentrate resolution there and keep a few larger ones for context.
B_VALUES_FINE="0 0.25 0.5 0.75 1.0 1.5 2.0 2.5 3.0 4.0 6.0 8.0"

# Archive that eigen-ab-fine takes its pinned Kp_att/Kd_att from. Must be an
# EIGEN archive — Kp_att is a commanded angular rate here (1/s, optimum 3-4)
# but a thrust differential in the Cascaded PD controller (N/rad, optimum
# 25-55), so a PD archive is rejected by gain_seed rather than misapplied.
SEED_FROM="${RESULTS}/genetic_eigen_sweep"

# Eigen attitude-gain grid. Distinct from the PD one: Kp_att commands an
# angular rate here (1/s, searched optimum 3-4) rather than a thrust
# differential (N/rad, optimum 25-55), so the PD ladder of 10-50 would sit
# entirely above the Eigen controller's working range.
EIGEN_KP_VALUES="0.5 1 2 3 4 5 7"
EIGEN_KD_VALUES="1 2 4 6 8 10 14"

# PD attitude-gain grid. Kd ceiling raised 1.0 → 3.0 to match PD_GENES; the old
# ceiling sat below the spinning optimum (Kd ≈ 1.4) and made the grid look far
# worse than the GA when the real difference was the box.
KP_ATT_VALUES="10 15 20 25 30 40 50"
KD_ATT_VALUES="0.2 0.5 1.0 1.5 2.0 3.0"

GA_POP=16
GA_GENERATIONS=30

usage() { sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        --list)         DO_LIST=1; shift ;;
        --dry-run)      DRY_RUN=1; shift ;;
        --only)         ONLY="$2"; shift 2 ;;
        --headless)     HEADLESS="--headless"; shift ;;
        --no-headless)  HEADLESS="--no-headless"; shift ;;
        --no-archive)   ARCHIVE=0; shift ;;
        --no-backfill)  BACKFILL=0; shift ;;
        --duration)     DURATION="$2"; shift 2 ;;
        --goal-radius)  GOAL_RADIUS="$2"; shift 2 ;;
        --goal-seed)    GOAL_SEED="$2"; shift 2 ;;
        --spawn-altitude) SPAWN_ALTITUDE="$2"; shift 2 ;;
        --pop-size)     GA_POP="$2"; shift 2 ;;
        --generations)  GA_GENERATIONS="$2"; shift 2 ;;
        -h|--help)      usage; exit 0 ;;
        *) echo "unknown option: $1" >&2; usage; exit 2 ;;
    esac
done

# Task arguments shared by every sweep. Built after parsing so the CLI
# overrides above are picked up.
TASK_ARGS="--goal-radius ${GOAL_RADIUS} --goal-seed ${GOAL_SEED}"
if [[ -n "$SPAWN_ALTITUDE" ]]; then
    TASK_ARGS="${TASK_ARGS} --spawn-altitude ${SPAWN_ALTITUDE}"
fi

# ── Sweep registry: name | results subdir | command ──────────────────────────
# Kept as parallel arrays so the file stays bash-3 compatible.
# Order matters: eigen-ga runs FIRST so the grid sweeps that follow can pin
# their non-swept gains to what it found (--seed-from). Running eigen-ab first
# is what left it stuck on the stale Kp_att=0.7 default, where only 12 % of its
# grid cells flew. A seed failure is fatal, so if eigen-ga fails the dependent
# sweeps stop loudly rather than silently reverting to that default.
NAMES=(
    eigen-ga
    eigen-ab
    eigen-ab-fine
    eigen-kpkd
    eigen-explore
    pd-ga
    pd-grid
    mpc
)
SUBDIRS=(
    genetic_eigen_sweep
    eigen_sweep
    eigen_sweep
    eigen_sweep
    eigen_explore
    genetic_pd_sweep
    pd_sweep
    mpc_sweep
)
describe() {
    case "$1" in
        eigen-ab)      echo "Eigen M_dyn (a,b) grid — the b-vs-spin-rate plane" ;;
        eigen-kpkd)    echo "Eigen (Kp_att,Kd_att) grid at pinned M_dyn" ;;
        eigen-explore) echo "Eigen (a,b) Halton space-filling exploration" ;;
        eigen-ga)      echo "Eigen NSGA-II genetic search (4 genes)" ;;
        pd-grid)       echo "Cascaded PD attitude-gain grid" ;;
        pd-ga)         echo "Cascaded PD NSGA-II genetic search (3 genes)" ;;
        mpc)           echo "LPV-MPC parameter sweep" ;;
        eigen-ab-fine) echo "Eigen (a,b) rerun: refined b in [0,8], gains seeded from the eigen GA" ;;
    esac
}
command_for() {
    case "$1" in
        eigen-ab)
            echo "${SWEEPS}/eigen_stability_sweep.py \
--motor-moments ${MOTOR_MOMENTS} \
--a-values ${A_VALUES} --b-values ${B_VALUES} \
--seed-from ${SEED_FROM} --run-tag wide \
--duration ${DURATION} ${TASK_ARGS} ${HEADLESS}" ;;
        eigen-kpkd)
            echo "${SWEEPS}/eigen_stability_sweep.py \
--motor-moments ${MOTOR_MOMENTS} \
--kp-values ${EIGEN_KP_VALUES} --kd-values ${EIGEN_KD_VALUES} \
--duration ${DURATION} ${TASK_ARGS} ${HEADLESS}" ;;
        eigen-explore)
            echo "${SWEEPS}/eigen_ab_explore.py \
--motor-moments ${MOTOR_MOMENTS} \
--samples 256 --sampling halton \
--duration ${DURATION} ${TASK_ARGS} ${HEADLESS}" ;;
        eigen-ga)
            echo "${SWEEPS}/genetic_sweep.py --controller eigen \
--motor-moments ${MOTOR_MOMENTS} \
--pop-size ${GA_POP} --generations ${GA_GENERATIONS} \
--duration ${DURATION} ${TASK_ARGS} ${HEADLESS}" ;;
        pd-grid)
            echo "${SWEEPS}/pd_sweep.py \
--motor-moments ${MOTOR_MOMENTS} \
--kp-att-values ${KP_ATT_VALUES} --kd-att-values ${KD_ATT_VALUES} \
--duration ${DURATION} ${TASK_ARGS} ${HEADLESS}" ;;
        pd-ga)
            echo "${SWEEPS}/genetic_sweep.py --controller pd \
--motor-moments ${MOTOR_MOMENTS} \
--pop-size ${GA_POP} --generations ${GA_GENERATIONS} \
--duration ${DURATION} ${TASK_ARGS} ${HEADLESS}" ;;
        mpc)
            echo "${SWEEPS}/mpc_sweep.py --duration ${DURATION} ${TASK_ARGS} ${HEADLESS}" ;;
        # Runs last so it can seed from the genetic_eigen_sweep archive that
        # eigen-ga writes earlier in this same run.
        eigen-ab-fine)
            echo "${SWEEPS}/eigen_stability_sweep.py \
--motor-moments ${MOTOR_MOMENTS} \
--a-values ${A_VALUES} --b-values ${B_VALUES_FINE} \
--seed-from ${SEED_FROM} --run-tag fine \
--duration ${DURATION} ${TASK_ARGS} ${HEADLESS}" ;;
    esac
}

selected() {
    [[ -z "$ONLY" ]] && return 0
    [[ ",${ONLY}," == *",$1,"* ]]
}

if [[ $DO_LIST -eq 1 ]]; then
    printf '%-15s %-22s %s\n' NAME "RESULTS SUBDIR" DESCRIPTION
    for i in "${!NAMES[@]}"; do
        printf '%-15s %-22s %s\n' "${NAMES[$i]}" "${SUBDIRS[$i]}" "$(describe "${NAMES[$i]}")"
    done
    exit 0
fi

if [[ ! -x "$ISAAC_PY" ]]; then
    echo "Isaac Sim python not found or not executable: $ISAAC_PY" >&2
    echo "Set ISAACSIM_PATH to your Isaac Sim install and retry." >&2
    exit 1
fi

# Isaac Sim ships Python 3.11 and conflicts with a sourced ROS 2 environment,
# so strip ROS from the environment exactly as isaac_run does before launching.
isaac_python() {
    env -u ROS_VERSION -u ROS_PYTHON_VERSION -u ROS_DISTRO \
        -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u PYTHONPATH \
        -u CMAKE_PREFIX_PATH \
        LD_LIBRARY_PATH="$(echo "${LD_LIBRARY_PATH:-}" | tr ':' '\n' \
            | grep -v '^/opt/ros/' | paste -sd':' -)" \
        "$ISAAC_PY" "$@"
}

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${RESULTS}/sweep_logs/${STAMP}"
mkdir -p "$LOG_DIR"

echo "=============================================================="
echo " Controller sweep run — ${STAMP}"
echo " repo      : ${REPO_ROOT}"
echo " isaac     : ${ISAAC_PY}"
echo " duration  : ${DURATION}s per run   ${HEADLESS}"
echo " task      : ground start → takeoff → ${GOAL_RADIUS}m lateral (goal seed ${GOAL_SEED})"
echo " logs      : ${LOG_DIR}"
[[ $DRY_RUN -eq 1 ]] && echo " MODE      : DRY RUN (nothing will execute)"
echo "=============================================================="

# Archive prior results so a rerun never silently destroys the previous set.
if [[ $ARCHIVE -eq 1 && $DRY_RUN -eq 0 ]]; then
    ARCHIVE_DIR="${RESULTS}/_archive_${STAMP}"
    for i in "${!NAMES[@]}"; do
        selected "${NAMES[$i]}" || continue
        src="${RESULTS}/${SUBDIRS[$i]}"
        if [[ -d "$src" ]]; then
            mkdir -p "$ARCHIVE_DIR"
            if [[ ! -e "${ARCHIVE_DIR}/${SUBDIRS[$i]}" ]]; then
                cp -a "$src" "${ARCHIVE_DIR}/${SUBDIRS[$i]}"
                echo "archived ${SUBDIRS[$i]} → ${ARCHIVE_DIR}/"
            fi
        fi
    done
fi

declare -a RESULT_NAMES RESULT_STATUS RESULT_SECONDS
OVERALL=0

for i in "${!NAMES[@]}"; do
    name="${NAMES[$i]}"
    selected "$name" || continue

    cmd="$(command_for "$name")"
    log="${LOG_DIR}/${name}.log"

    echo
    echo "--------------------------------------------------------------"
    echo ">>> ${name}  —  $(describe "$name")"
    echo "    log: ${log}"
    echo "    \$ isaac_python ${cmd}"
    echo "--------------------------------------------------------------"

    if [[ $DRY_RUN -eq 1 ]]; then
        RESULT_NAMES+=("$name"); RESULT_STATUS+=("dry-run"); RESULT_SECONDS+=(0)
        continue
    fi

    start=$SECONDS
    # shellcheck disable=SC2086
    ( cd "$REPO_ROOT" && isaac_python $cmd ) >"$log" 2>&1
    status=$?
    elapsed=$((SECONDS - start))

    RESULT_NAMES+=("$name"); RESULT_SECONDS+=("$elapsed")
    if [[ $status -eq 0 ]]; then
        RESULT_STATUS+=("ok")
        echo "    done in ${elapsed}s"
    else
        RESULT_STATUS+=("FAILED(${status})")
        OVERALL=1
        echo "    FAILED (exit ${status}) after ${elapsed}s — see ${log}"
        echo "    last lines:"
        tail -n 15 "$log" | sed 's/^/      /'
    fi
done

# Recompute every metric from the freshly written telemetry, so the run ends
# with one CSV covering all sweeps regardless of which ones were selected.
if [[ $BACKFILL -eq 1 && $DRY_RUN -eq 0 ]]; then
    echo
    echo ">>> backfilling metrics from all .npz under results/"
    python3 "${SCRIPT_DIR}/evaluations/backfill_metrics.py" \
        --root "$RESULTS" --duration "$DURATION" \
        --out "${LOG_DIR}/backfilled_metrics.csv" 2>&1 | sed 's/^/    /'
fi

echo
echo "=============================================================="
printf '%-15s %-14s %s\n' SWEEP STATUS ELAPSED
for i in "${!RESULT_NAMES[@]}"; do
    printf '%-15s %-14s %ss\n' "${RESULT_NAMES[$i]}" "${RESULT_STATUS[$i]}" "${RESULT_SECONDS[$i]}"
done
echo "logs: ${LOG_DIR}"
echo "=============================================================="
exit $OVERALL
