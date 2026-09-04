# Flying something

Every way of putting a vehicle in the air. `fly.py` is the way in — it takes a
level and does the rest.

Nothing here searches a gain space; that is `../sweeps/`. Nothing here reads a
ulog after the fact; that is `../tools/px4_tuning/`.

## Every script here

| script | run with | what it does |
|---|---|---|
| **`fly.py`** | `python3` | **The front door.** `--level 1|2|3` picks a rung of the ladder below and dispatches to the right pair of scripts. Launches its own sim, or `--attach`es to one you started yourself. Start here. |
| `sim_pegasus.py` | `isaac_run` | Isaac side of **level 1**. Climbs to altitude over the spawn and holds it — no lateral leg, so the profile matches levels 2/3. `--lateral` translates to the goal after takeoff; add `--no-takeoff-first` for the single diagonal the archived sweeps used. A pure-Python controller flying inside Isaac Sim directly — no PX4. Flags for controller, motor moment, failure mode and mixer policy; anything left unset is prompted for interactively when run directly, so its old interactive mode still works if you run it by hand. |
| `sim_px4.py` | `isaac_run` | Isaac side of **levels 2 and 3**. Spawns the vehicle and boots a real PX4 SITL process talking MAVLink to it. Does **not** arm or fly anything — it just puts a flyable sim on the wire for `offboard.py` to fly. `--keep-log` keeps the ulog. |
| `offboard.py` | `python3` | The actual flight profile for levels 2/3: arm, take off, hold, land, then point at the resulting ulog. Talks to an already-running PX4 over MAVLink. `fly.py` calls this automatically; run it directly if you launched `sim_px4.py` yourself and want to fly it without `fly.py` owning the process. |
| `px4_driver.py` | *(library)* | The shared `PX4Driver` class every offboard script here imports: connect, arm, offboard setpoints, live parameter get/set, and `check_airframe()`, which refuses to fly the wrong airframe rather than fly it wrong. Not run on its own — keep it a sibling of whatever imports it. |
| `simlaunch.py` | *(library)* | Shared plumbing for starting/stopping an Isaac + PX4 subprocess: finds Isaac's interpreter, clears the PX4 lockstep TCP port before launching, kills the whole process tree afterwards. Used by `fly.py` and `gain_search.py` so that cleanup logic exists once. Not run on its own. |
| `gain_search.py` | `python3` | Batch version of levels 2/3: one **fresh** PX4 launch per gain candidate in a list, each flown and scored automatically. Use this instead of `fly.py` when you have many candidates rather than one flight to check by hand. |
| `motor_failure.py` | `isaac_run` | Level-1 rotor-failure study: flies to a hover target, waits for it to settle, then kills one rotor and watches. `--desaturation` picks what the mixer sacrifices when a rotor saturates — defaults to `collective` here (**not** what the ladder flies elsewhere), so its results aren't directly comparable to a level-1 sweep. |
| `run_inner_loop.py` | `python3` | Offboard diagnostic against a running sim: isolates which *stage* of the eigen controller (rate loop vs. attitude loop) loses stability under a given spin rate. |
| `run_step_id.py` | `python3` | Offboard diagnostic against a running sim: flies a repeatable step-input sequence per axis, for system identification (measuring the plant's actual response rather than assuming it). |

Every script here answers `--help` under a plain `python3` — including the
three that need Isaac to actually *run* — because their Isaac imports sit
below argparse. Reading a flag list never boots a simulator.

## The three levels

Every level is **headed by default** — `fly.py` flies a single flight, and the
usual reason to fly one is to watch it. Pass `--headless` for an unattended run.
Batch work (`gain_search.py`) is headless regardless.

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

```bash
fly.py --level 1                                  # Pegasus + Python
fly.py --level 2 --motor-moment 2.7e-7 --hold 90  # PX4, on ground truth
fly.py --level 3 --motor-moment 2.7e-7 --hold 90  # same candidate, estimator in
```

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
always done.

## When a level 2/3 run fails

`fly.py` writes the sim's own stdout/stderr to `<run-dir>/sim.log` and prints
that path on launch. If the flight dies with "PX4 never appeared", the reason is
in that file — it is the sim that failed, not the offboard half, and `fly.py`
now tails it for you rather than leaving you with the symptom.

## Output

Level-1 telemetry goes to `../results/`, the one anchored tree shared with every
sweep. PX4 runs write their ulog under `px4_runs/`, alongside `sim.log`. Both
are gitignored.
