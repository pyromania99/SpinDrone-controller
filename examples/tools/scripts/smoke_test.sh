#!/usr/bin/env bash
#
# smoke_test.sh — run every sweep repeatedly at minimal size and check it works.
#
# Why repeat rather than run once
# ───────────────────────────────
# A single pass proves the happy path only. The bug that killed the last full
# run was a NameError inside the crash-logging branch: it needed a vehicle to
# actually hit the ground before it fired, so the sweep ran fine for 149 s and
# then died. Repeating with varied gains makes rare branches — crashes, OOB,
# all-fail columns, seeding failures — much more likely to execute before hours
# of GPU time are committed.
#
# What each iteration asserts (beyond exit status)
#   1. process exited 0
#   2. no Traceback / NameError / py stderr in the log
#   3. the expected archive CSV was written
#   4. logged flight duration matches --duration within 10 %
#      (this is what silently broke: --duration 25 was delivering 6 s)
#   5. every logged per-rotor thrust is within the physical ceiling
#
# Usage
#   ./smoke_test.sh                 # 10 iterations of every sweep
#   ./smoke_test.sh -n 3            # 3 iterations
#   ./smoke_test.sh --only eigen-ga,pd-grid
#   ./smoke_test.sh --duration 12
#
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
SWEEPS="${REPO_ROOT}/examples/sweeps"
: "${ISAACSIM_PATH:=${HOME}/isaacsim}"
ISAAC_PY="${ISAACSIM_PATH}/python.sh"

N=10
DURATION=12
ONLY=""
OUT_ROOT="${REPO_ROOT}/examples/results/_smoke"

while [[ $# -gt 0 ]]; do
    case "$1" in
        -n|--iterations) N="$2"; shift 2 ;;
        --duration)      DURATION="$2"; shift 2 ;;
        --only)          ONLY="$2"; shift 2 ;;
        -h|--help)       sed -n '2,30p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown option: $1" >&2; exit 2 ;;
    esac
done

[[ -x "$ISAAC_PY" ]] || { echo "Isaac python not found: $ISAAC_PY" >&2; exit 1; }

# Every sweep launched from here writes into the smoke tree, never the real one.
#
# This is not a tidiness measure. Five of the seven sweeps had no way to
# redirect their output and wrote straight into results/<sweep>/mm_<km>/, and
# the smoke configs below pin --motor-moments 4e-7 — so each smoke run silently
# REPLACED the real km=4e-7 archives with its own 2x2 grid. Measured after one
# run: pd_sweep/mm_4.0e-07 held 4 rows where the real grid had 42, and
# genetic_pd_sweep/mm_4.0e-07 held 12 where the real search had written 496.
# The smoke test reported success both times; nothing downstream can tell a
# 4-row archive from a 42-row one except by counting them.
#
# SWEEP_RESULTS_ROOT is read by examples/utils/sweep_task.results_root(), which
# every sweep now resolves its output through.
SMOKE_RESULTS="${OUT_ROOT}/tree"

isaac_python() {
    env -u ROS_VERSION -u ROS_PYTHON_VERSION -u ROS_DISTRO \
        -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u PYTHONPATH \
        -u CMAKE_PREFIX_PATH \
        SWEEP_RESULTS_ROOT="$SMOKE_RESULTS" \
        LD_LIBRARY_PATH="$(echo "${LD_LIBRARY_PATH:-}" | tr ':' '\n' \
            | grep -v '^/opt/ros/' | paste -sd':' -)" \
        "$ISAAC_PY" "$@"
}

NAMES=(eigen-ga eigen-ab eigen-kpkd eigen-explore pd-ga pd-grid mpc)

# Minimal configs. Gains are varied per iteration (via $i) so different
# stability regimes — including crashes — get exercised across the 10 runs.
cmd_for() {
    local name="$1" i="$2" tag="smoke_${1}_${2}"
    case "$name" in
        eigen-ga)
            echo "${SWEEPS}/genetic_sweep.py --controller eigen \
--motor-moments 4e-7 --pop-size 4 --generations 2 \
--duration ${DURATION} --seed $((40 + i)) --headless" ;;
        eigen-ab)
            echo "${SWEEPS}/eigen_stability_sweep.py --motor-moments 4e-7 \
--a-values $((2 + i * 6)) $((10 + i * 5)) --b-values 0 2 \
--kp-att 3.5 --kd-att 2.6 --run-tag ${tag} \
--duration ${DURATION} --no-validate-pareto --headless" ;;
        eigen-kpkd)
            echo "${SWEEPS}/eigen_stability_sweep.py --motor-moments 4e-7 \
--kp-values 0.5 $((1 + i)) --kd-values 2 $((4 + i)) --run-tag ${tag} \
--duration ${DURATION} --no-validate-pareto --headless" ;;
        eigen-explore)
            echo "${SWEEPS}/eigen_ab_explore.py --motor-moments 4e-7 \
--samples 4 --sampling halton --pop-size 4 --seed $((40 + i)) \
--duration ${DURATION} --headless" ;;
        pd-ga)
            echo "${SWEEPS}/genetic_sweep.py --controller pd \
--motor-moments 4e-7 --pop-size 4 --generations 2 \
--duration ${DURATION} --seed $((40 + i)) --headless" ;;
        pd-grid)
            echo "${SWEEPS}/pd_sweep.py --motor-moments 4e-7 \
--kp-att-values $((10 + i * 4)) $((20 + i * 3)) --kd-att-values 0.5 1.5 \
--duration ${DURATION} --no-validate-pareto --headless" ;;
        mpc)
            echo "${SWEEPS}/mpc_sweep.py --motor-moments 4e-7 \
--q-values $((5 + i * 5)) $((10 + i * 5)) --r-values 0.01 0.05 \
--duration ${DURATION} --headless" ;;
    esac
}

selected() { [[ -z "$ONLY" ]] && return 0; [[ ",${ONLY}," == *",$1,"* ]]; }

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="${OUT_ROOT}/${STAMP}"
mkdir -p "$LOG_DIR"

echo "=============================================================="
echo " SMOKE TEST — ${N} iterations per sweep, --duration ${DURATION}s"
echo " logs: ${LOG_DIR}"
echo "=============================================================="

declare -a SUM_NAME SUM_PASS SUM_FAIL
OVERALL=0

for name in "${NAMES[@]}"; do
    selected "$name" || continue
    pass=0; fail=0
    echo
    echo "── ${name} ──────────────────────────────────────────────"
    for ((i = 0; i < N; i++)); do
        log="${LOG_DIR}/${name}_${i}.log"
        cmd="$(cmd_for "$name" "$i")"
        before=$(date +%s)
        # shellcheck disable=SC2086
        ( cd "$REPO_ROOT" && isaac_python $cmd ) >"$log" 2>&1
        status=$?
        elapsed=$(( $(date +%s) - before ))

        problems=""
        [[ $status -ne 0 ]] && problems+="exit=${status} "
        if grep -qE "Traceback|NameError|AttributeError|TypeError:|KeyError|py stderr" "$log" 2>/dev/null; then
            problems+="exception "
        fi
        if grep -qE "seed-from failed" "$log" 2>/dev/null; then
            problems+="seed-failed "
        fi
        # duration + thrust-ceiling assertions on whatever telemetry was written
        # --root follows the redirected tree. Without it this validated
        # whatever happened to be freshest under the REAL results/, which
        # after the isolation fix is no longer what the smoke run produced.
        check=$(python3 "${REPO_ROOT}/examples/tools/evaluations/check_smoke_run.py" \
                    --root "$SMOKE_RESULTS" --duration "$DURATION" 2>/dev/null)
        [[ -n "$check" ]] && problems+="$check "

        if [[ -z "$problems" ]]; then
            pass=$((pass + 1)); printf "  [%2d] ok   (%3ds)\n" "$i" "$elapsed"
        else
            fail=$((fail + 1)); OVERALL=1
            printf "  [%2d] FAIL (%3ds)  %s\n" "$i" "$elapsed" "$problems"
            grep -m3 -E "Traceback|NameError|Error\] \[__main__\]" "$log" 2>/dev/null | sed 's/^/        /'
        fi
    done
    SUM_NAME+=("$name"); SUM_PASS+=("$pass"); SUM_FAIL+=("$fail")
done

echo
echo "=============================================================="
printf '%-16s %6s %6s\n' SWEEP PASS FAIL
for i in "${!SUM_NAME[@]}"; do
    printf '%-16s %6s %6s\n' "${SUM_NAME[$i]}" "${SUM_PASS[$i]}" "${SUM_FAIL[$i]}"
done
echo "logs: ${LOG_DIR}"
[[ $OVERALL -eq 0 ]] && echo "ALL SMOKE TESTS PASSED" || echo "SMOKE TESTS FAILED — do not start the full run"
echo "=============================================================="
exit $OVERALL
