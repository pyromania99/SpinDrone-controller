# Controller test scripts

This folder contains scripts for running controller sweeps in Isaac Sim and post-processing the logged results.

The two most common steps are:

1. Run an Eigen stability sweep with `eigen_stability_sweep.py` (generates raw `.npz` logs + sweep summaries)
2. Visualize the logged trajectories with `plot_results.py` (generates per-run plots next to each `.npz`)

---
Contropllers are stored in examples/utils

## 1) Run the Eigen stability sweep

Script: `examples/controller_tests/eigen_stability_sweep.py`

Run from the repository root using `isaac_run`:

### A. ωn / ζ sweep mode (default)

Sweeps the controller’s *placed* eigenvalues by varying $\omega_n$ (and optionally $\zeta$). One drone is spawned per $\omega_n$ value (parallel), and the script runs serially over `--motor-moments`.

Example (single ζ):

```bash
isaac_run examples/controller_tests/eigen_stability_sweep.py \
  --motor-moments 6e-7 \
  --omega-n-values 2 4 6 8 10 12 \
  --zeta 1.0 \
  --duration 20 \
  --goal-mode random_xy \
  --kp-att 7.0 --kd-att 9.0
```

Example (multiple ζ values):

```bash
isaac_run examples/controller_tests/eigen_stability_sweep.py \
  --motor-moments 1e-7 2.7e-7 4e-7 \
  --omega-n-values 2 4 6 8 10 12 \
  --zeta-values 0.7 1.0 1.3 \
  --duration 15
```

Optional: add `--headless` for a non-graphical run.

### B. Direct M_dyn (a, b) sweep mode (`--a-values` + `--b-values`)

This mode sweeps the M_dyn entries directly:

- `a` = diagonal term (called `wn_mdyn` in code)
- `b` = off-diagonal term (called `b_mdyn` in code)

Parallelism: one drone per `a` (parallel), one pass per `b` (serial), and one batch per motor moment (serial).

Example:

```bash
isaac_run examples/controller_tests/eigen_stability_sweep.py \
  --a-values 2 4 6 8 10 \
  --b-values 0 2 4 6 8 10 \
  --motor-moments 6e-7 \
  --duration 20 \
  --goal-mode random_xy \
  --kp-att 7.0 --kd-att 9.0
```

---

## Outputs from `eigen_stability_sweep.py`

All outputs go under:

- `examples/controller_tests/results/`

The sweep writes two kinds of outputs:

- **Raw per-run logs**: `*_statistics.npz` files (one per drone/run)
- **Sweep summaries**: CSV tables + PNG plots/heatmaps (one set per motor moment)

### A. ωn / ζ sweep outputs

Base directory (per motor moment):

- `examples/controller_tests/results/eigen_sweep/wn_eta_sweep/mm_<km>/`

Where `<km>` is formatted as scientific notation with one decimal (Python `:.1e`), e.g. `mm_6.0e-07`.

**Single ζ (no `--zeta-values`)** writes:

- `mm_<km>/scores.csv`
- `mm_<km>/score_plot.png` (bar chart of stability score vs $\omega_n$)
- `mm_<km>/on<omega_n>_statistics.npz` (one file per $\omega_n$)

**Multiple ζ (`--zeta-values ...`)** writes:

- `mm_<km>/zeta_<zeta>/scores.csv`
- `mm_<km>/zeta_<zeta>/on<omega_n>_statistics.npz`
- `mm_<km>/eta_compare_heatmap.png` (score heatmap across ζ and $\omega_n$)
- `mm_<km>/eta_compare_scores.csv` (same data as the heatmap)
- `mm_<km>/eta_compare_effort_heatmap.png` (control-effort heatmap)
- `mm_<km>/eta_compare_effort.csv`

### B. Direct (a, b) sweep outputs

Base directory (per motor moment):

- `examples/controller_tests/results/eigen_sweep/ab_sweep/mm_<km>/`

Writes sweep summaries:

- `mm_<km>/scores.csv` (2-D table in `a \ b` layout)
- `mm_<km>/effort.csv`
- `mm_<km>/scores_heatmap.png`
- `mm_<km>/effort_heatmap.png`

And raw logs per `b` value:

- `mm_<km>/b_<b>/a<a>_statistics.npz`

---

## 2) Plot trajectories and errors with `plot_results.py`

Script: `examples/controller_tests/plot_results.py`

This script reads `*_statistics.npz` files, prints summary metrics to stdout, and generates plots.

### A. Plot a whole sweep folder (recommended)

In batch mode, the script searches recursively for all `*_statistics.npz` under `--batch-root`.

Example (plot just one `mm_...` sweep):

```bash
python examples/controller_tests/plot_results.py \
  --batch-root examples/controller_tests/results/eigen_sweep/mm_6.0e-07 \
  --no-show
```

Example (plot an (a,b) sweep):

```bash
python examples/controller_tests/plot_results.py \
  --batch-root examples/controller_tests/results/eigen_sweep/ab_sweep/mm_6.0e-07 \
  --no-show
```

**Batch-mode outputs** (saved next to each source `.npz` by default):

- `<npz_path_without_extension>_time_series.png`
- `<npz_path_without_extension>_xy.png`
- `<npz_path_without_extension>_yaw_rate.png` (only if the log contains `yaw_rate`)

### B. Plot a single `.npz` file

```bash
python examples/controller_tests/plot_results.py \
  --file examples/controller_tests/results/eigen_sweep/ab_sweep/mm_6.0e-07/b_0.000/a2.000_statistics.npz
```

To save plots in single-file mode, also pass `--save-prefix`:

```bash
python examples/controller_tests/plot_results.py \
  --file examples/controller_tests/results/eigen_sweep/ab_sweep/mm_6.0e-07/b_0.000/a2.000_statistics.npz \
  --save-prefix examples/controller_tests/results/tmp/a2.000
```

---

## What’s inside `*_statistics.npz`

Each `*_statistics.npz` is a NumPy archive containing time-series arrays. At minimum `plot_results.py` expects:

- `time`: shape `(N,)`
- `p`: shape `(N,3)` position
- `desired_p`: shape `(N,3)` reference position
- `ep`: shape `(N,3)` position error

Additional keys may be present depending on the controller/logging (e.g. `yaw_rate`, `motor_thrusts`, etc.).
