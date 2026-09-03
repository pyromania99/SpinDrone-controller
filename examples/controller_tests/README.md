# Controller tests

Sweeps, evaluations and plots for the spinning-drone controllers — searching the
gain space and turning the results into numbers and pictures.

The controllers themselves live in `examples/utils/` (`eigen_controller.py`,
`cascaded_pd_controller.py`, `lpv_mpc_controller.py`), and `controller_factory.py`
turns a controller name plus a gain dict into an instance — start there rather than
constructing one by hand. To fly a single vehicle interactively rather than sweep
anything, see `../flight/`.

## Running things

Two kinds of script, and the distinction matters because getting it wrong looks
like an import error rather than a usage error:

- **Anything that flies** needs Isaac Sim, so it is launched with `isaac_run`.
  That is everything under `sweeps/`, the two `probe_*` scripts and
  `batch_compare.py` in `evaluations/`, and `eigen_kp_sweep.py` in `archive/`.
- **Anything that reads telemetry off disk** is plain Python, run with `python3`.
  That is all of `visualization/` and the rest of `evaluations/`.

```bash
isaac_run examples/controller_tests/sweeps/genetic_sweep.py --help
python3   examples/controller_tests/visualization/plot_results.py --help
```

Every script takes `--help` and documents its own flags there. This file is a map,
not a manual: the previous version reproduced flag lists and output paths, and all
of them were wrong within a few months.

## Layout

### `sweeps/` — search the gain space (Isaac)

| script | what it does |
|---|---|
| `genetic_sweep.py` | NSGA-II Pareto search over Eigen, PD and LPV-MPC gains. The main tuning entry point. |
| `eigen_stability_sweep.py` | Eigen M_dyn eigenvalue-placement stability sweep. |
| `eigen_ab_explore.py` | Space-filling exploration of the Eigen M_dyn (a, b) plane. |
| `ablation_explore.py` | Marches each tuning parameter outward until flight breaks. |
| `robustness_sweep.py` | How far a tuned controller survives inertia mismatch, sensor lag and wind. |
| `pd_sweep.py` / `mpc_sweep.py` | 2-D gain sweeps for the PD and LPV-MPC baselines. |

### `evaluations/` — turn runs into numbers

| script | what it does |
|---|---|
| `benchmarks.py` | Compare Cascaded PD, Eigen and LPV-MPC head to head. |
| `select_pareto_top.py` | Pick the top-N Pareto configurations per controller. |
| `select_reference.py` | Pick the operating point the ablation marches out from. |
| `backfill_metrics.py` | Recompute every metric from `.npz` telemetry already on disk. |
| `rebuild_breakdown.py` | Recompute `breakdown.csv` from a robustness archive. |
| `check_smoke_run.py` | Assert the physical invariants on freshly written telemetry. |
| `tuning_equivalence.py` | Analytical check that CascadedPD and Eigen are equivalently tuned. |
| `test_rotor_failure.py` | Checks the rotor-failure injection and the mixer desaturation policies. Runs without Isaac. |
| `probe_reset.py`, `probe_robustness.py` | One-off probes establishing facts the studies depend on (Isaac). |
| `batch_compare.py` | Controller comparison across several `motor_moment` values (Isaac). |

### `visualization/` — plots and reports

`plot_results.py` is the general one: point it at a `.npz` or a tree of them and it
prints summary metrics and writes per-run plots. The rest are specific to one study —
`pareto_comparison.py`, `robustness_report.py`, `ablation_report.py`,
`eigen_plot_summary.py`, `eigen_plot_heatmap.py`, `interactive_dashboard.py` (Plotly),
and `coning_plot.py` for the spin-axis and coning geometry of a spinning run.

### `archive/`

Superseded, kept for the record rather than because anything calls them.
`eigen_kp_sweep.py` was step 3 of the manual tuning workflow before
`genetic_sweep.py` replaced it; `compare_controllers.py` was the original
Eigen-vs-CascadedPD harness.

This folder was called `legacy/` and also held `controllers.py`, which is not
legacy at all — it now lives in `../flight/` with the other launchers.

## Output

Everything is written under `examples/controller_tests/results/`, including the
telemetry from the two launchers in `../flight/` that fly this project's
controllers. It is generated and gitignored — nothing in there is a source of truth, and any of it can
be rebuilt from the scripts above. Raw per-run telemetry is `*_statistics.npz`
(`time`, `p`, `desired_p`, `ep`, and further keys depending on the controller);
sweeps additionally write CSV summaries and PNG plots alongside it.
