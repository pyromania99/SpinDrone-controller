# PX4 tuning toolkit

Everything that reads a flight *after* it happened, plus the two probes that read
values out of a running simulator. Nothing here flies a vehicle — the launchers and
the offboard scripts moved to `../flight/`.

The findings these produced are written up in `notes/FINDINGS.md`; the plan that
uses them is `notes/NEXT_STEPS.md`.

## Log analysis — plain `python3`

| script | what it answers |
|---|---|
| `flight_health.py` | Is this flight healthy, and which mode is eating it? Run it on everything first. |
| `loop_latency.py` | Loop delay and `K_eff` from a ulog (four fitting bugs fixed — see FINDINGS §9). |
| `measure_motor_lag.py` | `T` and `tau_eff` from a hardware ulog. |
| `mode_compare.py` | The same mode measurement across a PX4 `.ulg` and a Pegasus `.npz`, handling the FLU↔FRD flip. |
| `replay_law.py` | Recompute what the eigen controller *should* have commanded from the logged state, and compare against what it did. |
| `ulog_to_npz.py` | Convert a ulog into the `.npz` layout the rest of the tooling reads. |
| `ga_to_candidates.py` | Turn a GA archive into a `gain_search.py` candidate list. Enforces the cascade separation the GA has no term for, and does not carry `Kd_att` across by default. |

## Simulator probes — run these with `isaac_run`

They start Isaac to interrogate the vehicle, then exit.

| script | what it answers |
|---|---|
| `derive_px4_params.py` | Reads mass, inertia and geometry out of the running sim and emits the corresponding PX4 parameter values. |
| `probe_geometry.py` | Reads the actual rotor geometry and checks PX4 agrees with it. |

## Working files

`*.json` in this directory (candidate sets, per-candidate results) is gitignored:
it is the working state of whichever search is in flight, regenerable from a
`genetic_archive.csv` via `ga_to_candidates.py`.
