# Flying something

Every way of putting a vehicle in the air. `fly.py` is the way in — it takes a
level and does the rest.

Nothing here searches a gain space; that is `../sweeps/`. Nothing here reads a
ulog after the fact; that is `../tools/px4_tuning/`.

## `fly.py` — pick a level

```bash
fly.py --level 1                                  # Pegasus + Python
fly.py --level 2 --motor-moment 2.7e-7 --hold 90  # PX4, on ground truth
fly.py --level 3 --motor-moment 2.7e-7 --hold 90  # same candidate, estimator in
```

| level | what is in the loop |
|---|---|
| **1 — Pegasus + Python** | The controller is a Python `Backend` reading simulator state directly. No PX4, no estimator, no transport delay — whatever it measures is the control law and the plant, nothing else. |
| **2 — PX4 + ground truth** | Real PX4: mixer, `CommandFrontEnd` limits, allocator, normalized torque. `MC_CTRL_GT=1` feeds it `vehicle_attitude_groundtruth`, so the estimator is out. |
| **3 — PX4, estimator in** | `MC_CTRL_GT=0`. EKF2 back in the loop. |

They are a **ladder**, and the value of each rung is its *difference* from the one
below:

- **Level 2 − Level 1 = PX4's implementation cost.** The mismatches deliberately
  left unmodelled become a measured number instead of an assumption.
- **Level 3 − Level 2 = the estimator's cost** at that spin rate. This is the
  number worth having before committing to hardware.

A figure from one level alone says very little. `notes/NEXT_STEPS.md` calls these
Stage 1, Stage 3 and Stage 4, and carries the mismatch ledger recording which
plant differences are still open.

Levels 2 and 3 launch their own sim by default. To fly one you started yourself —
to watch it in the GUI, say — use `--attach`:

```bash
# terminal 1
isaac_run examples/flight/sim_px4.py --keep-log px4_runs/mine \
    --motor-moment 2.7e-7 --duration 400
# terminal 2
python3 examples/flight/fly.py --level 2 --attach --run-dir px4_runs/mine --hold 90
```

## Why there is still more than one file

Levels 1 and 2/3 fly under **Isaac's** interpreter (`python.sh`); the offboard
half of levels 2 and 3 drives PX4 over MAVLink and needs system `python3` with
pymavlink. One process cannot be both. So `fly.py` stays plain `python3` and
launches the Isaac side as a subprocess — the same thing `gain_search.py` has
always done. The files below are those halves. They run standalone, but `fly.py`
is the front door.

| file | role |
|---|---|
| `sim_pegasus.py` | Isaac side of level 1. Flags for controller, motor moment, failure mode and mixer; anything omitted is prompted for when run directly, so it keeps its interactive mode. |
| `sim_px4.py` | Isaac side of levels 2 and 3. Spawns the vehicle and runs PX4 SITL. It does **not** arm — it just puts a flyable sim on the wire. `--keep-log` keeps the ulog. |
| `offboard.py` | The flight profile: arm, take off, hold, land, then point at the ulog. `fly.py` calls this for levels 2 and 3; runnable alone against a running sim. |
| `px4_driver.py` | The shared `PX4Driver`: connect, arm, offboard setpoints, live parameter set, and `check_airframe()`, which refuses to fly the wrong airframe. Imported by everything here — keep it a sibling. |
| `simlaunch.py` | Finds Isaac's interpreter, clears the lockstep port, kills the process tree afterwards. Shared by `fly.py` and `gain_search.py` so the cleanup has one implementation. |

Every script here answers `--help` under a plain `python3`, including the three
that need Isaac to actually run: their Isaac imports sit below argparse, so
reading a flag list never boots a simulator.

## The other two entry points

These fly, but they are not "pick a level and fly once", so they stay separate.

| script | what it does |
|---|---|
| `gain_search.py` | Stages 3–4 in batch: one fresh PX4 launch per candidate, flown and scored. Use it for a candidate list; use `fly.py` for one flight. |
| `motor_failure.py` | Kills one rotor once the vehicle has settled and watches. `--desaturation` picks what the mixer gives up when a rotor saturates — it defaults to `collective` here, which is **not** what the ladder flies, so its results are not directly comparable to a level-1 sweep. |

`run_inner_loop.py` and `run_step_id.py` are offboard diagnostics against a
running sim: which stage of the eigen controller loses stability under spin, and
a repeatable step sequence for per-axis system identification.

## Output

Level-1 telemetry goes to `../results/`, the one anchored tree shared with every
sweep. PX4 runs write their ulog under `px4_runs/`. Both are gitignored.
