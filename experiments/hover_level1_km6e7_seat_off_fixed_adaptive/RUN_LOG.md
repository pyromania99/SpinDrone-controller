# Run log — hover, level 1, km=6e-7, seat off / fixed / adaptive

**Dates flown:** 2026-09-10 (compare + attitude figures, 21:57 and 22:00) and
2026-09-11 (adaptive-only attitude figure, 00:15).
**Findings document:** [`KM6E7_LEVEL1.md`](KM6E7_LEVEL1.md).
**Commit the work landed in:** `a592ed3` — *"wrote some adaptive laws to take care of lag
issues at high spin. validated upto 30rad/s at 20ms lag. graph of flight attached."*

---

## Condition

| | |
|---|---|
| level | 1 — Pegasus + Python eigen law, no PX4, no estimator |
| controller | `eigen` |
| `--motor-moment` (km) | `6e-7` → r_ss ≈ 29.4 rad/s (rule: r_ss ≈ 4.9e7·km); **measured −29.5** |
| `--rotor-tau` | `0.020` (first-order rotor/ESC pole; level 1 has **no** transport delay) |
| profile | hover at 1.5 m over the spawn point, **no lateral demand** (`--lateral` omitted) |
| duration | 120 s |
| rotor failure | off |
| arms | `--seat off`, `--seat fixed`, `--seat adaptive` |

`ψ_model = atan(r·T) = atan(29.5 × 0.020) = 0.533 rad`, which is the dotted line on the
θ_s panel. Note this **excludes** the `r·tau_a` term in `Seat.psi_model` (`tau_a` defaults
to 2 ms); with it the line would read 0.592. Level 1 has no transport delay, so 0.533 is
the right convention here — but check which one a figure used before comparing θ_s to it.

## Commands

Seat-off arm, verbatim from shell history:

```bash
python3 examples/flight/fly.py --level 1 --controller eigen \
    --motor-moment 6e-7 --failure-mode off --rotor-tau 0.020 \
    --duration 120 --seat off
```

The other two arms are the same line with `--seat fixed` and `--seat adaptive`.
Add `--headless` for batch.

**Not recorded:** the exact `--seat-gains` for the adaptive arm. Those runs were launched
from a subshell that did not write to `~/.bash_history`, and no config file or commit
captures them. `Seat`'s constructor defaults are:

```
k=2.0  theta_max=1.4  r_min=5.0  law=cross  k_sign=+1  tau_a=0.002  T=<--rotor-tau>
```

**How to tell if a re-run matched:** θ_s should overshoot to ≈ 0.38 rad at t ≈ 7.5 s and
ring down to **≈ 0.157 rad** over ~8 cycles of a 7.4 s mode. If it converges somewhere else
or rings differently, `k` is the thing that differed — sweep that first. `k` is
**dimensional** for the CROSS law (it absorbs the units of `|xd||xa|`), so it does not
transfer between operating amplitudes; see the CAUTION block in `Seat.adapt`'s docstring.

## Raw data — GONE

Level 1 writes to a single fixed path:

```
examples/results/controller_eval/eigen/statistics/eigen_statistics.npz
```

**Every subsequent level-1 run overwrites it.** The file currently there is from the
2026-09-13 trajectory work. Nothing from these three flights survives except the figures.

If you re-run: copy the npz out after *each arm*, before starting the next.

### Rebuilding the figures from a fresh npz

| panel | field |
|---|---|
| altitude | `p[:,2]` |
| \|p,q\| | `hypot(angular_velocity[:,0], angular_velocity[:,1])` |
| \|eR\| | `norm(eR, axis=1)` |
| xy position error | `hypot(ep[:,0], ep[:,1])` |
| \|tau_xy\| | `hypot(torque[:,0], torque[:,1])` |
| spin rate r, θ_s | `yaw_rate`, `seat_theta` |
| α (misalignment) | `seat_alpha` — NaN means the gate froze the update, not "unmeasurable" |
| **roll, pitch** | **`desired_angles[:,0:2] − eR[:,0:2]`** |
| **yaw** | **`desired_angles[:,2]`**, then `np.unwrap` |

The attitude reconstruction is the non-obvious step: **the npz holds no raw attitude.**
`eigen_controller.py:512` logs `roll_error = desired_roll − roll`, so actual = desired −
error; and the third slot of `desired_angles` is the *measured* yaw, not a desired one
(`eigen_controller.py:736`).

## Code as of this run

- `examples/utils/seat.py` — `Seat` (modes OFF/FIXED/ADAPTIVE, laws ANGLE/CROSS)
- `examples/utils/eigen_controller.py:660–715` — where the seat is applied: `xd = tau/J`
  componentwise, `xa` rebuilt by `WrenchObserver` from the measured rate minus modelled
  drift, then `tau[:2] = seat.rotate(tau)` followed by a re-clip
- `examples/flight/sim_pegasus.py:506` — `_make_seat`, which returns `None` (not a disabled
  object) for the `off` arm, so that arm genuinely has no seat

## Figures

| file | what it is |
|---|---|
| `km6e7_compare.png` | 8 panels, all three arms: altitude, \|p,q\|, \|eR\|, xy error, torque, spin+θ_s |
| `km6e7_attitude.png` | roll/pitch/yaw + phase portrait, all three arms |
| `km6e7_attitude_adaptive.png` | the same for the adaptive arm alone, rescaled (±1.6° instead of ±180°) |

A byte-identical copy of `km6e7_attitude_adaptive.png` is still committed at the repository
root from `a592ed3`. Harmless, but it is a duplicate.

## Caveat on the numbers in the findings document

Every quantitative figure in `KM6E7_LEVEL1.md` §4a (lobe peaks, 7.2 s period, ζ ≈ 0.09,
θ_s = 0.157/0.38) is **read off the rendered PNGs**, because the npz is gone. They are good
to roughly the last digit shown and the qualitative structure is unambiguous — but
re-derive them from data before using them anywhere load-bearing.

---

# Addendum, 2026-09-17 — the re-flight

Flown to replace the pixel-read §4a numbers with measured ones. **It did not reproduce the
archived flights** — see the "RE-FLIGHT" section of
[`KM6E7_LEVEL1.md`](KM6E7_LEVEL1.md) for the comparison table and what it means.

## What was run

```bash
for ARM in off fixed adaptive; do
    rm -f examples/results/controller_eval/eigen/statistics/eigen_statistics.npz
    python3 examples/flight/fly.py --level 1 --controller eigen \
        --motor-moment 6e-7 --failure-mode off --rotor-tau 0.020 \
        --duration 120 --seat "$ARM" --headless
    cp examples/results/controller_eval/eigen/statistics/eigen_statistics.npz \
       examples/results/controller_eval/eigen/statistics/km6e7_refly/eigen_${ARM}.npz
done
```

Sequential, one sim at a time, ~2.8 min per arm. `Seat` defaults throughout
(`k=2.0, theta_max=1.4, r_min=5.0, law=cross, k_sign=+1`, `T` from `--rotor-tau`).

## Raw data — KEPT this time

```
examples/results/controller_eval/eigen/statistics/km6e7_refly/
    eigen_off.npz  eigen_fixed.npz  eigen_adaptive.npz     (8.2 MB each, 30000 samples, 120 s)
    off.log  fixed.log  adaptive.log                        (Isaac console)
```

Copying the npz out after **each** arm is the whole practice. The archived flights were lost
to exactly this.

## Figures

```bash
python3 experiments/tools/plot_level1.py \
    --arm "seat OFF:.../eigen_off.npz" \
    --arm "seat FIXED:.../eigen_fixed.npz" \
    --arm "seat ADAPTIVE:.../eigen_adaptive.npz" \
    --out experiments/hover_level1_km6e7_seat_off_fixed_adaptive --prefix km6e7_refly \
    --rotor-tau 0.02 --zoom 0 30 --settle-from 60 --alt-threshold 1.0 \
    --title "Level 1 hover, km=6e-7, rotor_tau 20 ms — RE-FLOWN 2026-09-17 (current build)"

python3 experiments/tools/analyse_envelope.py .../eigen_adaptive.npz --rotor-tau 0.02
```

`--alt-threshold 1.0` matters: level 1 hovers at 1.5 m, so the level-2/3 default of 4 m
would report every arm as never airborne.

## Measured outcome

| arm | settled spin | departure | outcome |
|---|---|---|---|
| off | −29.48 rad/s | \|p,q\| > 1 at **t = 24.0 s** | departs, then **bounces** — 11 altitude crossings, still near 1.5 m at t = 120 s |
| fixed | (crashed before spin-up) | **t = 4.1 s** | terminal, on the ground by 6.3 s |
| adaptive | −29.48 rad/s | never | **flew the full 120 s**, 1 crossing |

`analyse_envelope.py` on the adaptive arm:

```
psi_model = atan(rT) = 0.5330 rad          (tau_a excluded: level 1 has no transport delay)
roll envelope: first 8 lobes at 7.3 13.9 19.6 25.1 30.8 36.5 42.9 48.9 s
               spacing 5.94 +/- 0.38 s  ->  0.168 Hz
               peaks 1.75 1.03 0.80 0.69 0.60 0.51 0.48 0.50 deg
               floor 0.308 deg, log decrement 0.652 -> zeta = 0.103
               last 10% 0.249 deg vs 0.385 deg mid-run -> STILL DECAYING at t=120 s
theta_s:       peak +0.2863 rad at t = 8.1 s
               converged +0.0672 +/- 0.0004 rad
               converged / psi_model = 0.126
```

## The unexplained part

The seat-off arm departs at 24.0 s here against 13.7 s in the archived figure, and bounces
instead of staying down. `examples/utils/{eigen_controller,seat}.py` and
`examples/flight/sim_pegasus.py` changed by 213 lines in commit `8c48c10`, between the two
flights — the `--max-lateral-accel` clamp and a position-loop D-term rework. **Not
bisected.** If an exact seat-off failure time ever matters, bisect `8c48c10`.

The archived `--seat-gains` are still unrecorded, so part of the θ_s gap (0.157 → 0.067) may
be a different `k` rather than the code change. `k` is dimensional for the CROSS law.
