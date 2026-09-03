# Tools

Everything that supports flight rather than being the point of it. Flying one
vehicle is `../flight/`; flying a population to search a gain space is
`../sweeps/`. Most of what is here reads results that those produced.

A handful of scripts here do start a simulator — the probes that read values out
of a running vehicle, a superseded sweep, and one comparison harness — but they
exist to answer a question about the setup, not to produce flight results.

| folder | what is in it |
|---|---|
| `evaluations/` | Turning runs into numbers: head-to-head benchmarks, Pareto selection, metric backfill, invariant checks. |
| `visualization/` | Plots and reports: `plot_results.py` for any `.npz`, plus study-specific ones — `pareto_comparison.py`, `robustness_report.py`, `ablation_report.py`, `coning_plot.py`, `eigen_plot_summary.py`, `eigen_plot_heatmap.py` and the Plotly `interactive_dashboard.py`. |
| `px4_tuning/` | The PX4 analysis toolkit — reads ulogs after the fact. Has its own README. |
| `scripts/` | The bash drivers that chain the sweeps together. |
| `archive/` | Superseded, kept for the record rather than because anything calls them. |
| `trajectories/` | Reference trajectory CSVs (ellipses, relay steps, back-and-forth). Input data, not results — a controller is handed one by path, and nothing resolves this directory implicitly. |

Most of it is plain `python3`. Two exceptions:

- **Needs `isaac_run`** (it starts a simulator): `evaluations/probe_reset.py`,
  `evaluations/probe_robustness.py`, `evaluations/batch_compare.py`,
  `archive/eigen_kp_sweep.py`, and the two probes in `px4_tuning/`.
- **Needs `scipy`**, which is in Isaac's interpreter but not necessarily in a
  bare `python3`: `evaluations/tuning_equivalence.py`. Run it with
  `~/isaacsim/python.sh` if `import scipy` fails.

Every script here that takes arguments answers `--help` without starting a
simulator, including the `isaac_run` ones — the Isaac imports are deferred past
argparse precisely so you can read a flag list without booting anything.

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
