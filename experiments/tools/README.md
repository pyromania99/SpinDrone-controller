# Plotting tools

Two scripts that produce **the same four figures** for any run, at any level, so the levels
can be laid side by side without worrying that a difference is a plotting artifact.

| script | input | used by |
|---|---|---|
| [`plot_run.py`](plot_run.py) | a PX4 run directory (reads its `.ulg`) | levels 2 and 3 |
| [`plot_level1.py`](plot_level1.py) | a level-1 `eigen_statistics.npz` | level 1 |

`plot_level1.py` converts the npz into the dict layout `plot_run.py` builds from a ulog and
then calls **`plot_run.py`'s own figure functions** — the panels are drawn by identical
code, not by two implementations that happen to look alike.

## The four figures

| figure | question it answers |
|---|---|
| `*_compare.png` | altitude, \|p,q\|, tilt, position error, torque, spin + θ_s. *Did it fly, and how did it fail?* |
| `*_attitude.png` | roll / pitch / yaw / roll–pitch phase portrait. *What shape was the failure?* |
| `*_seat.png` | θ_s alone, with α and θ_b. *What did the adaptation converge to?* |
| `*_trajectory.png` | flown vs commanded path, following distance, speed, altitude. *Did it track?* (skipped when there is no trajectory setpoint) |
| `*_attitude_adaptive.png` | the **seat-on arm alone**, rescaled: roll, pitch, unwrapped yaw, phase portrait. *How well did the surviving arm actually hold attitude?* |

`*_attitude_adaptive.png` exists because a shared-axis attitude figure is useless for the
arm that survives — the diverging arms reach ±180° and squash the survivor to a flat line.
Rescaled to one arm, the same data resolves to a fraction of a degree. The arm is picked
automatically as the one whose `seat_theta` is actually non-zero.

## Usage

```bash
# levels 2 / 3 — one --arm per run directory
python3 experiments/tools/plot_run.py \
    --arm "seat OFF:px4_runs/L2_eigen_20260913_201119" \
    --arm "seat ON:px4_runs/L2_eigen_20260913_201321" \
    --out experiments/trajectory_level2_px4_groundtruth_fast_circle \
    --prefix l2_lag20ms --zoom 0 25 --settle-from 25 --title "..."

# level 1 — one --arm per npz; note the lower altitude threshold
python3 experiments/tools/plot_level1.py \
    --arm "seat OFF:.../eigen_off.npz" --arm "seat ADAPTIVE:.../eigen_adaptive.npz" \
    --out experiments/hover_level1_... --prefix km6e7_refly \
    --rotor-tau 0.02 --zoom 0 15 --settle-from 60 --alt-threshold 1.0 --title "..."
```

`--only compare attitude seat trajectory` restricts which figures are written.

## Decisions baked into these scripts

Each of these exists because getting it wrong produced a wrong answer at some point in this
project. They are not stylistic.

- **Ground truth for all attitude and position analysis.** Level 2 flies on it anyway; for
  level 3 the estimator is what is under test, so scoring it against its own estimate is
  circular. `vehicle_attitude` was also unusable above ~20 rad/s until the EKF2
  output-predictor fix, and reported 86° tilt where truth was 18.7°.
- **Garbage timestamps are filtered and re-sorted on every topic.** `mc_controller_status`
  and `trajectory_setpoint` carry occasional `UINT64_MAX` values; `np.interp` needs sorted
  x and returns silent garbage (exact zeros, once) otherwise.
- **`airborne()` returns a crossing count, not just a duration.** A first/last-time-above-
  threshold duration cannot distinguish a stable flight from a bounce sequence — that
  mistake produced a wrong "seat off flew 41.6 s" at level 3, where the real trace has 12
  crossings.
- **ψ_model's spin rate is the median over the *airborne* window of the longest-flying
  arm.** Over a whole record, a crashed arm sits at r = 0 and drags the median to zero,
  making the model line read 0.000.
- **Position error is blanked while the commanded altitude is below 1 m.** Before the
  trajectory streams, the setpoint is (0,0,0) and the "error" is just distance from the
  origin, which is not a tracking measurement.
- **Panels are trimmed to the flight window.** The pre-arm 1e-6 rad/s noise floor otherwise
  sets the y-range of every log-scale panel.
- **α is drawn faint per-tick with a 1 s NaN-aware rolling mean on top.** NaN means *the
  allocator gate froze the update*, not "unmeasurable", so those ticks are dropped from the
  mean rather than counted as zero. The legend reports the frozen percentage.
- **Tilt, not roll/pitch magnitude, is the attitude error.** `arccos` of the body-z/world-z
  dot product is frame-independent, which matters when yaw is deliberately spinning.

## Regenerating everything

```bash
bash experiments/tools/make_all_figures.sh
```

Rebuilds every figure in the tree that has surviving raw data, from the repo root, and
prints what it deliberately does not rebuild (the archived level-1 figures and the level-1
trajectory work, whose logs were overwritten). Deterministic.

## Gotchas when extending them

- **Orbit radius must be measured about the fitted circle centre**, not the origin. The
  level-2/3 reference circle is centred near (0, 10); measuring about (0, 0) turns
  11.2 ± 0.5 m into 14.3 ± 6.4 m.
- **The level-1 npz has no raw attitude.** `actual = desired_angles[:, :2] − eR[:, :2]`,
  and `desired_angles[:, 2]` is the *measured* yaw, not a desired one. Reversing the
  subtraction silently flips the sign of every attitude panel.
- **PX4 parameters must be read as `initial_parameters` overlaid with `changed_parameters`.**
  Most runs boot with the seat disabled and are configured over MAVLink afterwards, so
  `initial_parameters` alone reports the wrong arm.
- **The level-1 altitude threshold is 1 m, not 4 m** — level 1 hovers at 1.5 m.
- **`np.unwrap` is not NaN-safe**, and the flight-window trim puts NaNs at the *start* of
  every trimmed series. One NaN poisons the whole cumulative sum, which silently blanked
  the yaw panel of every trimmed figure. Use `unwrap_nan()`.
- **`np.any(theta != 0)` is not a test for "the seat was running"** — `NaN != 0` is True,
  so trimmed NaNs make it fire on the seat-*off* arm. Use `_seat_is_active()`, which
  compares finite samples only.
- **A recovered dip is not a crash, and a bounce is not a recovery.** `crash_time()` judges
  recovery on body rates, not altitude: the vehicle must get back above hover height *and*
  hold \|p,q\| under 1 rad/s for 3 s. Altitude alone cannot separate the level-2 run that
  dips to 0.11 m and flies on for 40 s from the level-3 run that bounces to 8.4 m without
  ever flying again.
