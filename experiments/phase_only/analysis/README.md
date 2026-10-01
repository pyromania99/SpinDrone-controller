# analysis — the experiment figures, tiled side by side

Nothing here is a new flight. Every tile is a PNG lifted unmodified (only scaled) out of an
experiment folder and pasted into a labelled sheet, so a sheet can never disagree with the
experiment it came from. Built by
[`../tools/make_analysis_grids.py`](../../tools/make_analysis_grids.py); re-run it after
regenerating any source figure.

```bash
python3 experiments/tools/make_analysis_grids.py                    # the ten sheets below
python3 experiments/tools/make_analysis_grids.py --all-conditions   # + the wide grids
python3 experiments/tools/make_analysis_grids.py --tile-width 900   # smaller files
```

| folder | profile | columns on the sheet |
|---|---|---|
| [`hover/`](hover/) | climb and hold, no trajectory, r ≈ 29.5 | **levels 1, 2 and 3** at ~17–20 ms |
| [`circle/`](circle/) | R = 10 m, 5 m/s circle | levels 2 and 3, `nolag` |

One sheet per view — `compare`, `attitude`, `seat`, `trajectory`, `attitude_adaptive` —
with **the rungs as the columns**, so a question ("what did θ_s converge to?") is one image
side by side rather than three tabs.

## α is on every arm as of 2026-09-17

The seat sheets used to show the misalignment α only for the arm whose adaptation was
running; the seat-OFF and seat-FIXED traces were empty, and the panel blamed a frozen
allocator gate. That was wrong — α had never been computed on those arms. Both controllers
now measure it unconditionally and the affected runs were re-flown. The measurement applies
nothing, and the level-1 seat-OFF arm was re-flown and diffed against its previous log to
prove the flight is unchanged. See [`../tools/README.md`](../../tools/README.md).

## Scope

**The lowest-delay arm of each rung, and nothing else.** For levels 2 and 3 that is the
`nolag` arms; for level 1 it is its only hover, whose 20 ms rotor pole comes to the same
phase (see [`hover/README.md`](hover/README.md)). Level 1 is on the hover sheet only — it
never flew the fast circle. The `lag20ms` and r ≈ 20 arms are not built by default;
`--all-conditions` adds the wider grids over every lag and spin condition as
`*_by_level_and_lag.png`. The spec for those is kept in the tool, so that is a flag, not a
rewrite.

---

## The one thing to fix in your head before reading any sheet

**`nolag` does not mean zero lag.** The figure names say how much *rotor* pole was modelled
and nothing about the rest of the loop. PX4 carries ~17 ms of transport delay plus ~14 ms of
pipeline filtering whether or not a rotor pole exists, so:

| name in the files | what it actually is | read it as |
|---|---|---|
| `nolag` (`rotor_tau = 0`) | PX4's own pipeline, no rotor pole | **~17 ms** |
| `lag20ms` (`rotor_tau = 0.02`) | the same pipeline **plus** a 20 ms rotor pole | **~37 ms** |
| level 1, `rotor_tau = 0.02` | a 20 ms rotor pole and nothing else — **no PX4 in the loop** | **~20 ms** |

The names are kept everywhere, because they are what the files are called and what the
RUN_LOGs reproduce. Only the reading changes, and the sheets carry both.

θ_s values do not transfer between level 1 and the PX4 rungs: level 1 converges to
0.067 rad and levels 2 and 3 to −0.41…−0.91 rad at comparable spin. **Do not read anything
into that sign flip** — whether the Python law and the PX4 implementation define θ_s the
same way is not established anywhere in this tree, and no sheet here tests it.

## circle_l3_envelope/ — added 2026-09-18

A third folder, and the only one whose columns are **conditions rather than rungs**. Every
cell is level 3 on the identical R = 10 m, 5 m/s circle; left to right the spin rises and
then a rotor pole is added:

| column | condition | result |
|---|---|---|
| 1 | r ≈ 25.9, no pole (~19.5 ms) | all three arms fly — no rescue to see, and the seat slightly worsens tracking |
| 2 | **r ≈ 29.5, no pole (~19.5 ms)** | **seat OFF crashes 35.4 s at 32.6 m off path; seat ON flies 52.2 s at 5.04 m** |
| 3 | r ≈ 25.8, 20 ms pole (~39.5 ms) | neither arm survives; the seat buys ~8 s and no tracking |

Column 2 is the level-3 trajectory rescue. Column 3 is why it needs the pole absent: a
20 ms pole adds amplitude **droop** (0.86 at this spin) on top of rotation, and the seat
cancels rotation only.

**Read these on following distance, not airborne time** — in column 3 the seat-ON arm stays
up longer than seat-OFF while ending further off the path. Both arms in column 2 are n = 1.

*(In column 2's path panel the axes are set by the seat-OFF arm departing to x ≈ 300 m, so
the seat-ON circle is compressed near the origin; the following-distance panel beside it is
the readable comparison.)*
