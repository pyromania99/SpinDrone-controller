#!/usr/bin/env python3
"""plot_phase_time.py - the roll-pitch phase portrait, coloured by time.

A duplicate of the bottom-right panel of `*_attitude_adaptive.png`, on its own axes and
with the one thing that panel cannot show: WHEN each loop was flown. Drawn flat, the
portrait is an annulus and every orbit lies on top of every other, so a ringdown and a
steady limit cycle look identical. Coloured by time they do not.

COLOUR
    One hue, light -> dark, early -> late (a sequential encoding for a sequential
    quantity), and the hue is the red the rest of the level-1 figures already use. The
    light end is truncated at 0.25 so early samples stay legible on white. Never a
    rainbow map here: hue changes would read as categories.

Usage:
    python3 plot_phase_time.py --npz path/to/eigen_adaptive.npz \\
        --out DIR --name km6e7_refly_phase_time --settle-from 60 \\
        --label "seat ADAPTIVE" --title "Level 1 hover, km=6e-7, rotor_tau 20 ms"
"""
from __future__ import annotations

import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap, Normalize

import plot_level1 as L


def time_ramp(name="Reds", lo=0.25, hi=1.0, n=256):
    """The house red as a sequential ramp, with the near-white end cut off."""
    base = plt.get_cmap(name)
    return LinearSegmentedColormap.from_list(f"{name}_{lo}_{hi}",
                                             base(np.linspace(lo, hi, n)))


def fig_phase_time(t, roll, pitch, out, title, label="", settle_from=None, full=False):
    """roll vs pitch over the window, as a line coloured by time."""
    ok = np.isfinite(roll) & np.isfinite(pitch)
    if not ok.any():
        return None
    t_lo, t_hi = t[ok][0], t[ok][-1]
    sf = t_lo if full else (settle_from if settle_from is not None
                            else t_lo + 0.6 * (t_hi - t_lo))
    if not (t_lo <= sf < t_hi):
        sf = t_lo + 0.6 * (t_hi - t_lo)
    m = ok & (t >= sf)

    x, y, tm = roll[m], pitch[m], t[m]
    seg = np.stack([np.column_stack([x[:-1], y[:-1]]),
                    np.column_stack([x[1:], y[1:]])], axis=1)

    fig, ax = plt.subplots(figsize=(8.2, 7.4))
    norm = Normalize(tm[0], tm[-1])
    lc = LineCollection(seg, cmap=time_ramp(), norm=norm, linewidths=0.35, alpha=0.85)
    lc.set_array(tm[:-1])
    ax.add_collection(lc)
    ax.autoscale_view()
    ax.set_aspect("equal", "datalim")
    ax.set_xlabel("roll [deg]")
    ax.set_ylabel("pitch [deg]")
    window = "full flight" if full else f"t $\\geq$ {sf:.0f} s"
    ax.set_title(f"roll–pitch phase portrait, coloured by time ({window})", fontsize=11)
    ax.grid(alpha=.3)
    fig.suptitle(title, fontsize=13)

    cb = fig.colorbar(lc, ax=ax, pad=0.02, fraction=0.045)
    cb.set_label("t [s]")
    # The colorbar is the legend, so say which way it runs in words too.
    cb.ax.text(0.5, 1.01, "late", transform=cb.ax.transAxes, ha="center",
               va="bottom", fontsize=8, color="0.35")
    cb.ax.text(0.5, -0.015, "early", transform=cb.ax.transAxes, ha="center",
               va="top", fontsize=8, color="0.35")

    rng = max(np.ptp(x), np.ptp(y))
    fig.text(0.5, 0.005,
             f"{label}   |   {window}: roll {x.mean():+.2f}±{x.std():.2f}°, "
             f"pitch {y.mean():+.2f}±{y.std():.2f}°, full range {rng:.2f}°",
             ha="center", fontsize=9, color="0.35")
    fig.tight_layout(rect=(0, 0.02, 1, 0.97))
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--npz", required=True, help="a level-1 eigen_*.npz")
    ap.add_argument("--out", required=True, help="output directory")
    ap.add_argument("--name", required=True, help="output basename, no extension")
    ap.add_argument("--label", default="")
    ap.add_argument("--title", default="")
    ap.add_argument("--rotor-tau", type=float, default=0.020)
    ap.add_argument("--settle-from", type=float, default=60.0)
    ap.add_argument("--full", action="store_true",
                    help="draw the whole flight instead of the settled window")
    a = ap.parse_args()

    d = L.load_npz(a.npz, a.rotor_tau)
    os.makedirs(a.out, exist_ok=True)
    got = fig_phase_time(d["t_att"], np.degrees(d["roll"]), np.degrees(d["pitch"]),
                         f"{a.out}/{a.name}.png", a.title or a.name, a.label,
                         a.settle_from, a.full)
    print("wrote", got)


if __name__ == "__main__":
    main()
