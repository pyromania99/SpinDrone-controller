# Flying something

Every way of putting a vehicle in the air, in one place: the launchers for each
simulation level, and the offboard scripts that drive an already-running PX4 over
MAVLink.

Nothing here searches a gain space — that is `../controller_tests/`. Nothing here
reads a ulog after the fact — that is `../px4_tuning/`.

## The three levels

They form a ladder, and **the value of each rung is its difference from the one
below it**. Run them in order or the numbers mean nothing on their own.

| level | launch | what is in the loop |
|---|---|---|
| **1 — Pegasus + Python** | `isaac_run examples/flight/controllers.py` | The controller is a Python `Backend` reading simulator state directly. No PX4, no estimator, no transport delay. |
| **2 — PX4 + ground truth** | `isaac_run examples/flight/single_vehicle.py` then `fly_one.py --ground-truth` | Real PX4: mixer, `CommandFrontEnd` limits, allocator, normalized torque. `MC_CTRL_GT=1` feeds it `vehicle_attitude_groundtruth`, so the estimator is out. |
| **3 — PX4 without ground truth** | same, without `--ground-truth` | `MC_CTRL_GT=0`. EKF2 back in the loop. |

- **Level 2 − Level 1 = PX4's implementation cost.** The mismatches deliberately
  left unmodelled become a measured number instead of an assumption.
- **Level 3 − Level 2 = the estimator's cost** at that spin rate. This is the
  number worth having before committing to hardware.

`notes/NEXT_STEPS.md` calls these Stage 1, Stage 3 and Stage 4 and carries the
mismatch ledger that says which plant differences are still open.

## Launchers — run these with `isaac_run`

They start Isaac Sim, so they need Isaac's own interpreter.

| script | level | what it does |
|---|---|---|
| `controllers.py` | 1 | Interactive: prompts for a controller, motor moment, failure mode and mixer policy, then flies it in the GUI. Also the Pegasus-only half of every PX4 comparison, which is why it must model the same plant the sweeps do. |
| `motor_failure.py` | 1 | Kills one rotor once the vehicle has settled and watches. `--desaturation` picks what the mixer gives up when a rotor saturates. |
| `single_vehicle.py` | 2, 3 | Spawns the vehicle and runs PX4 SITL. It does **not** arm — it just puts a flyable sim on the wire for the offboard scripts below. `--keep-log` keeps the ulog. |

Telemetry from the level-1 launchers is written to
`../controller_tests/results/`, with the rest of the level-1 output, rather than
next to these scripts.

## Offboard control — plain `python3`

These talk to an already-running PX4 over MAVLink. They do not start a simulator,
so launch one first and point them at it.

| script | what it does |
|---|---|
| `px4_driver.py` | The shared `PX4Driver`: connect, arm, offboard setpoints, live parameter set, and `check_airframe()` which refuses to fly the wrong airframe. Imported by everything else here — keep it a sibling. |
| `fly_one.py` | Arm, take off, hold, land **one** already-running sim, then score its ulog. The middle piece between `single_vehicle.py` (which never arms) and `gain_search.py` (which owns its own launching). Use it when you want to watch a single flight. |
| `gain_search.py` | Stages 3–4: one fresh launch per candidate, flown and scored. Owns both the launching and the flying, so it is the batch path rather than the watch-it path. |
| `run_inner_loop.py` | Isolates which stage of the eigen controller loses stability under spin. |
| `run_step_id.py` | Flies a repeatable step sequence for per-axis system identification. |

Typical level-2 session, in two terminals:

```bash
# terminal 1 — put a sim on the wire
isaac_run examples/flight/single_vehicle.py --keep-log px4_runs/mytest \
    --motor-moment 2.7e-7 --duration 400

# terminal 2 — fly it once, on ground truth
python3 examples/flight/fly_one.py --run-dir px4_runs/mytest --hold 90 --ground-truth
```

Drop `--ground-truth` for level 3. PX4 runs write to `px4_runs/`, which is
gitignored.
