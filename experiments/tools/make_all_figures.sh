#!/bin/bash
# Regenerate every figure in experiments/ from the raw logs. Run from the repo root.
#
#   bash experiments/tools/make_all_figures.sh
#
# Deterministic: re-running reproduces the committed PNGs. Level-1 figures come from the
# preserved npz copies, levels 2/3 from the ulogs under px4_runs/.
set -e
cd "$(dirname "$0")/../.."
E=experiments
R=px4_runs
L1=examples/results/controller_eval/eigen/statistics/km6e7_refly

px4() { python3 $E/tools/plot_run.py "$@"; }

echo "== level 1 hover (re-flight) =="
python3 $E/tools/plot_level1.py \
    --arm "seat OFF:$L1/eigen_off.npz" \
    --arm "seat FIXED:$L1/eigen_fixed.npz" \
    --arm "seat ADAPTIVE:$L1/eigen_adaptive.npz" \
    --out $E/hover_level1_km6e7_seat_off_fixed_adaptive --prefix km6e7_refly \
    --rotor-tau 0.02 --zoom 0 30 --settle-from 60 --alt-threshold 1.0 --no-crash-cut \
    --title "Level 1 hover, km=6e-7, rotor_tau 20 ms — RE-FLOWN 2026-09-17 (full record, see note)"
# --no-crash-cut: this arm never reaches a terminal crash, it limit-cycles. See KM6E7_LEVEL1.md.

echo "== level 2 hover =="
H2=$E/hover_level2_px4_groundtruth
px4 --arm "seat OFF:$R/L2_airframe_20260918_003456" --arm "seat ON (adaptive):$R/L2_airframe_20260918_003645" \
    --out $H2 --prefix l2_hover_lag20ms --zoom 0 25 --settle-from 13 \
    --title "Level 2 hover (PX4 + ground truth), km=6e-7 (r~29.5), rotor_tau 20 ms"
px4 --arm "seat OFF:$R/L2_airframe_20260917_234829" --arm "seat ON (adaptive):$R/L2_airframe_20260917_235022" \
    --out $H2 --prefix l2_hover_nolag --zoom 0 25 --settle-from 20 \
    --title "Level 2 hover, same spin, rotor_tau 0 (PX4 pipeline lag only)"
px4 --arm "seat OFF:$R/L2_airframe_20260918_004209" --arm "seat ON (adaptive):$R/L2_airframe_20260918_004401" \
    --out $H2 --prefix l2_hover_r20_lag20ms --zoom 0 30 --settle-from 20 \
    --title "Level 2 hover at r=20 (the circle's spin), rotor_tau 20 ms — the seat RESCUES"

echo "== level 3 hover =="
H3=$E/hover_level3_px4_estimator
px4 --arm "seat OFF:$R/L3_eigen_20260918_003829" --arm "seat ON (adaptive):$R/L3_eigen_20260918_004016" \
    --out $H3 --prefix l3_hover_lag20ms --zoom 0 25 --settle-from 15 \
    --title "Level 3 hover (PX4 + estimator), km=6e-7 (r~29.5), rotor_tau 20 ms"
px4 --arm "seat OFF:$R/L3_eigen_20260917_235215" --arm "seat ON (adaptive):$R/L3_eigen_20260917_235407" \
    --out $H3 --prefix l3_hover_nolag --zoom 0 25 --settle-from 20 \
    --title "Level 3 hover, same spin, rotor_tau 0 (PX4 pipeline lag only)"
px4 --arm "seat OFF:$R/L3_eigen_20260918_004550" --arm "seat ON (adaptive):$R/L3_eigen_20260918_004741" \
    --out $H3 --prefix l3_hover_r20_lag20ms --zoom 0 30 --settle-from 20 \
    --title "Level 3 hover at r=20, rotor_tau 20 ms — both survive, but only one holds attitude"

echo "== level 2 trajectory (fast circle) =="
T2=$E/trajectory_level2_px4_groundtruth_fast_circle
px4 --arm "seat OFF:$R/L2_airframe_20260918_004932" --arm "seat ON (adaptive):$R/L2_airframe_20260918_005128" \
    --out $T2 --prefix l2_lag20ms --zoom 0 25 --settle-from 25 \
    --title "Level 2 (PX4 + ground truth), fast circle, r=20 rad/s, rotor_tau 20 ms"
px4 --arm "seat OFF:$R/L2_airframe_20260917_235601" --arm "seat ON (adaptive):$R/L2_airframe_20260917_235804" \
    --out $T2 --prefix l2_nolag --zoom 0 25 --settle-from 25 \
    --title "Level 2, same circle, rotor_tau 0 (PX4 pipeline lag only)"

echo "== level 3 trajectory (fast circle) =="
T3=$E/trajectory_level3_px4_estimator_fast_circle
px4 --arm "seat OFF:$R/L3_eigen_20260918_000004" --arm "seat ON:$R/L3_eigen_20260918_000208" \
    --arm "seat + adaptive b:$R/L3_eigen_20260918_000420" \
    --out $T3 --prefix l3_nolag --zoom 0 25 --settle-from 25 \
    --title "Level 3 (PX4 + estimator), fast circle, r=25 rad/s, rotor_tau 0"
px4 --arm "seat OFF:$R/L3_eigen_20260918_005324" --arm "seat ON:$R/L3_eigen_20260918_005519" \
    --out $T3 --prefix l3_lag20ms --zoom 0 25 --settle-from 12 \
    --title "Level 3, same circle, rotor_tau 20 ms - neither arm survives"

echo "== level 2 circle, rotor_tau 0.02, seat ON: the SAME COMMAND four times =="
# The "seat rescues level 2" headline reproduces once in four. Plotting the repeats
# together is the only honest way to show a bimodal outcome.
px4 --arm "run 1 - crashed 19.9s:$R/L2_airframe_20260918_005128" \
    --arm "run 2 - FLEW:$R/L2_airframe_20260918_005844" \
    --arm "run 3 - crashed 22.4s:$R/L2_airframe_20260918_010040" \
    --arm "run 4 - crashed 24.6s:$R/L2_airframe_20260918_010234" \
    --out $T2 --prefix l2_lag20ms_repeats --zoom 0 30 --settle-from 25 --no-crash-cut \
    --title "Level 2 circle, rotor_tau 20 ms, seat ADAPTIVE - four runs of one command"

echo
echo "== analysis sheets (tiles the PNGs above, reads no logs) =="
python3 $E/tools/make_analysis_grids.py

echo
echo "NOT regenerated (no raw data survives, or not produced by these tools):"
echo "  hover_level1_.../km6e7_{compare,attitude,attitude_adaptive}.png   archived 2026-09-10, logs overwritten"
echo "  trajectory_level1_*/                                             logs overwritten"
echo "  trajectory_level3_.../l3_{combined_panels,zoom_response}.png      built ad hoc 2026-09-14"
echo "  supporting_b_pole_theory_and_archive/                             analytic + archived sweep"
