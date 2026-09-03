# Tools

Everything that supports flight without doing it. Nothing in here puts a vehicle
in the air except the two Isaac probes noted below — that is `../flight/` for one
vehicle and `../sweeps/` for many.

| folder | what is in it |
|---|---|
| `evaluations/` | Turning runs into numbers: head-to-head benchmarks, Pareto selection, metric backfill, invariant checks. |
| `visualization/` | Plots and reports: `plot_results.py` for any `.npz`, plus study-specific reports (Pareto, robustness, ablation, coning, an interactive Plotly dashboard). |
| `px4_tuning/` | The PX4 analysis toolkit — reads ulogs after the fact. Has its own README. |
| `scripts/` | The bash drivers that chain the sweeps together. |
| `archive/` | Superseded, kept for the record rather than because anything calls them. |

Most of it is plain `python3`. The exceptions, which need `isaac_run`:
`evaluations/probe_reset.py`, `evaluations/probe_robustness.py`,
`evaluations/batch_compare.py`, `archive/eigen_kp_sweep.py`, and the two probes
in `px4_tuning/` that read values out of a running simulator.

`evaluations/test_rotor_failure.py` is worth knowing about: it checks the
rotor-failure injection and the mixer desaturation policies, and runs without
Isaac, so it is the one thing here you can execute as a quick sanity check.

## `scripts/`

| script | what it drives |
|---|---|
| `run_all_sweeps.sh` | Every controller sweep from one place, with `--list`, `--dry-run` and `--only`. |
| `run_ablation.sh` | The ablation study end to end. |
| `run_outer_sweep.sh` | The outer position loop, into its own results root so it cannot overwrite the main archives. |
| `run_robustness.sh` | The robustness study across its axes. |
| `smoke_test.sh` | A fast end-to-end check. Redirects `SWEEP_RESULTS_ROOT` so a smoke run cannot replace real archives with its 2x2 grid. |

They call Isaac through their own `isaac_python()` function rather than the
`isaac_run` shell alias, since a shell function cannot be invoked from a script.

## Output

Everything reads from and writes to `../results/`, the same anchored tree the
sweeps and launchers use. Resolve it in Python with
`sweep_task.results_root()` rather than deriving it from `__file__` — these
folders sit at different depths, and a caller-relative root would split one
archive into three.
