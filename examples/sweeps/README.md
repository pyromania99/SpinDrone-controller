# Sweeps

Searching the gain space. These fly too — that is what makes them slow — but they
fly a population rather than one vehicle, so they sit apart from `../flight/`.

All of them need Isaac Sim to run, so launch them with `isaac_run`. Reading
their flags does not: the Isaac imports are deferred past argparse, so `--help`
works under a plain `python3` without booting a simulator.

```bash
python3    examples/sweeps/genetic_sweep.py --help   # just the flags
isaac_run  examples/sweeps/genetic_sweep.py ...      # actually fly it
```

| script | what it does |
|---|---|
| `genetic_sweep.py` | NSGA-II Pareto search over Eigen, PD and LPV-MPC gains. The main tuning entry point — Stage 1 of the ladder in `notes/NEXT_STEPS.md`. |
| `eigen_stability_sweep.py` | Eigen M_dyn eigenvalue-placement stability sweep. |
| `eigen_ab_explore.py` | Space-filling exploration of the Eigen M_dyn (a, b) plane. |
| `ablation_explore.py` | Marches each tuning parameter outward until flight breaks. |
| `robustness_sweep.py` | How far a tuned controller survives inertia mismatch, sensor lag and wind. |
| `pd_sweep.py` / `mpc_sweep.py` | 2-D gain sweeps for the PD and LPV-MPC baselines. |

The controllers themselves are in `../utils/`; `controller_factory.py` turns a
controller name plus a gain dict into an instance, and `sweep_task.py` owns the
flight task every sweep evaluates — spawn, target and failure definitions live
there once so a change lands everywhere.

## Output

Everything goes to `../results/`, one anchored tree shared with the launchers in
`../flight/`, regardless of which script wrote it. It is generated and
gitignored: nothing in there is a source of truth and any of it can be rebuilt.

Set `SWEEP_RESULTS_ROOT` to redirect the whole tree — which is how
`../tools/scripts/smoke_test.sh` avoids overwriting real archives with a 2x2
grid, a loss that is silent and total if it happens.

Turning this output into numbers and pictures is `../tools/evaluations/` and
`../tools/visualization/`.
