# Flight logs — level 1 hover, km=6e-7

Level 1 has no PX4 and therefore no ulog: the flight is the `.npz` the eigen controller
writes. `km6e7_refly/` holds the three arms of the **2026-09-17 re-flight**.

| file | what it is |
|---|---|
| `eigen_{off,fixed,adaptive}.npz` | the flight — 30000 samples, 120 s. Field list in [`../RUN_LOG.md`](../RUN_LOG.md). |
| `{off,fixed,adaptive}_console.txt` | the Isaac console, distilled to the lines that say what was flown. |

| file | arm | outcome |
|---|---|---|
| `km6e7_refly/eigen_off.npz` | `--seat off` | departs 24.0 s, then limit-cycles, 11 altitude crossings |
| `km6e7_refly/eigen_fixed.npz` | `--seat fixed` | crashes 4.1 s |
| `km6e7_refly/eigen_adaptive.npz` | `--seat adaptive` | flies the full 120 s |

**The distilled console is worth reading before the npz.** It carries the line
`[seat] seat=adaptive law=cross k=2 thmax=1.40 rmin=5 ksign=+1` — the seat gains, which is
exactly what the 2026-09-10 originals failed to record and what
[`../RUN_LOG.md`](../RUN_LOG.md) flags as the open confound in the re-flight comparison.

**The 2026-09-10 originals have no surviving data at all** — level 1 writes every run to one
fixed path and the next run overwrites it. Only the archived figures remain.

These are copies; the originals stay in
`examples/results/controller_eval/eigen/statistics/km6e7_refly/`, which is gitignored.
