#!/usr/bin/env python3
"""
coning_plot.py — the spin axis and coning geometry of a run, from its telemetry.

What this extracts
──────────────────
Under a sustained yaw spin the vehicle stops holding an attitude and instead
settles into a cone: the body z-axis (the thrust direction) leans off vertical by
a fixed angle and that lean sweeps a full circle in the world frame once per
revolution. Two numbers describe it:

    cone half-angle   the lean of the body z-axis off the world vertical [deg]
    spin axis         the direction of the angular-velocity vector

The distinction is the whole point. In a relaxed-hover equilibrium the SPIN AXIS
stays put — pinned to the world vertical — while the BODY z-axis cones around it.
The vehicle is a spinning top: it cannot hold attitude, but because the lean
rotates with the body, the lateral thrust component averages to nothing over a
revolution and position is held anyway.

Reading the panels
──────────────────
A  cone half-angle against time. A rotor failure shows up here as a step.
B  the body z-axis tip on the unit sphere, projected to world XY. A point means
   level flight; a circle means coning, and its radius is sin(half-angle).
C  body rates p, q, r. r is the spin; p and q are the coning signature.
D  cone half-angle against the spin axis's own tilt off vertical. When the two
   diverge, the vehicle is coning about a stable axis rather than tumbling.

Usage
  python3 examples/tools/visualization/coning_plot.py \\
      examples/controller_tests/results/controller_eval/eigen/statistics/eigen_statistics.npz

  python3 .../coning_plot.py <npz> --cut 9.05 --out /tmp/coning.png
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Categorical slots 1-3 of the reference palette (blue / orange / aqua). Only
# the first three are used: that subset is the documented all-pairs-validated
# set, which matters here because panel B is a scatter.
C_BLUE, C_ORANGE, C_AQUA = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK_2, GRID = "#0b0b0b", "#52514e", "#d9d8d4"


def parse_args():
    p = argparse.ArgumentParser(description="Spin axis and coning geometry")
    p.add_argument("npz", type=str, help="statistics npz from any controller run")
    p.add_argument("--cut", type=float, default=None,
                   help="Time [s] of the rotor failure. Read from the file when "
                        "it was recorded; otherwise inferred from where the "
                        "roll/pitch rate leaves zero, or omitted entirely.")
    p.add_argument("--out", type=str, default=None,
                   help="PNG path. Default: alongside the npz as <name>_coning.png")
    p.add_argument("--tail", type=float, default=4.0,
                   help="Seconds at the end treated as the settled window.")
    return p.parse_args()


def body_z_in_world(roll, pitch, yaw):
    """Unit body z-axis expressed in world coordinates, ZYX Euler."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    # R = Rz(yaw) Ry(pitch) Rx(roll), third column.
    bx, by, bz = sp * cr, -sr, cp * cr
    return np.stack([cy * bx - sy * by, sy * bx + cy * by, bz], axis=1)


def omega_in_world(w, roll, pitch, yaw):
    """Body-frame angular velocity rotated into world coordinates."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    R = np.empty((len(roll), 3, 3))
    R[:, 0, 0] = cy * cp
    R[:, 0, 1] = cy * sp * sr - sy * cr
    R[:, 0, 2] = cy * sp * cr + sy * sr
    R[:, 1, 0] = sy * cp
    R[:, 1, 1] = sy * sp * sr + cy * cr
    R[:, 1, 2] = sy * sp * cr - cy * sr
    R[:, 2, 0] = -sp
    R[:, 2, 1] = cp * sr
    R[:, 2, 2] = cp * cr
    return np.einsum("nij,nj->ni", R, w)


def load(path: Path):
    d = np.load(str(path), allow_pickle=True)
    t = np.asarray(d["time"], dtype=float)
    # Actual attitude: eR is (desired - actual), logged alongside desired.
    da, eR = np.asarray(d["desired_angles"]), np.asarray(d["eR"])
    roll, pitch, yaw = da[:, 0] - eR[:, 0], da[:, 1] - eR[:, 1], da[:, 2]
    w = np.asarray(d["angular_velocity"], dtype=float)
    cut = None
    if "failure_time" in d:
        v = float(np.asarray(d["failure_time"]))
        cut = None if not np.isfinite(v) else v
    rotor = int(np.asarray(d["failed_rotor"])) if "failed_rotor" in d else -1
    return t, roll, pitch, yaw, w, cut, rotor


def infer_cut(t, w):
    """Where the roll/pitch rate leaves zero and stays there.

    Only used when the run did not record a failure time. Deliberately ignores
    the first 4 s, which are the takeoff transient — the vehicle is not level
    then and any threshold trips on the climb rather than on the failure.
    """
    pq = np.linalg.norm(w[:, :2], axis=1)
    after = t > 4.0
    idx = np.where(after & (pq > 0.5))[0]
    return float(t[idx[0]]) if len(idx) else None


def main() -> int:
    a = parse_args()
    path = Path(a.npz)
    t, roll, pitch, yaw, w, cut, rotor = load(path)
    if a.cut is not None:
        cut = a.cut
    if cut is None:
        cut = infer_cut(t, w)

    zw = body_z_in_world(roll, pitch, yaw)
    cone = np.degrees(np.arctan2(np.hypot(zw[:, 0], zw[:, 1]), zw[:, 2]))

    ww = omega_in_world(w, roll, pitch, yaw)
    wn = np.linalg.norm(ww, axis=1)
    # The direction of omega is undefined when the vehicle is barely rotating,
    # and dividing by a near-zero norm manufactures a spurious 80-plus degree
    # excursion on the pad that is pure numerical noise. Below this rate the
    # spin axis is simply not a meaningful quantity, so it is left blank.
    SPIN_FLOOR = 1.0                     # rad/s
    safe = wn > SPIN_FLOOR
    spin_tilt = np.full_like(cone, np.nan)
    spin_tilt[safe] = np.degrees(
        np.arccos(np.clip(np.abs(ww[safe, 2]) / wn[safe], -1.0, 1.0)))

    tail = t > t[-1] - a.tail
    pre = (t < cut) & (t > 4.0) if cut is not None else np.zeros_like(t, bool)
    post = (t >= cut) if cut is not None else np.ones_like(t, bool)

    fig, axes = plt.subplots(2, 2, figsize=(12.5, 8.2))
    fig.patch.set_facecolor("#fcfcfb")
    for ax in axes.ravel():
        ax.set_facecolor("#fcfcfb")
        ax.grid(True, color=GRID, lw=0.6, alpha=0.9)
        ax.set_axisbelow(True)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
        for s in ("left", "bottom"):
            ax.spines[s].set_color(GRID)
        ax.tick_params(colors=INK_2, labelsize=9)

    def mark_cut(ax, label=True):
        """Vertical rule at the failure. The caption is anchored in axes
        fraction near the floor, which is empty on every panel here — anchoring
        it to the data maximum put it straight through the traces."""
        if cut is None:
            return
        ax.axvline(cut, color=INK_2, lw=1.2, ls="--", alpha=0.8)
        if label:
            ax.annotate("rotor stopped", xy=(cut, 0.03), xycoords=("data", "axes fraction"),
                        xytext=(5, 0), textcoords="offset points",
                        color=INK_2, fontsize=8.5, va="bottom")

    # A — cone half-angle over time. One series, so no legend: the title names it.
    ax = axes[0, 0]
    ax.plot(t, cone, color=C_BLUE, lw=2.0)
    ax.set_title("A   Cone half-angle — body z-axis lean off vertical",
                 color=INK, fontsize=11, loc="left", pad=10)
    ax.set_xlabel("time [s]", color=INK_2, fontsize=9)
    ax.set_ylabel("half-angle [deg]", color=INK_2, fontsize=9)
    mark_cut(ax)
    if tail.any():
        v = cone[tail].mean()
        ax.axhline(v, color=C_BLUE, lw=1.0, ls=":", alpha=0.7)
        ax.annotate(f"settles at {v:.2f}°", xy=(t[-1], v), xytext=(-6, 6),
                    textcoords="offset points", ha="right",
                    color=INK, fontsize=9, fontweight="bold")

    # B — the tip locus. A point is level flight; a circle is coning.
    ax = axes[0, 1]
    if pre.any():
        ax.plot(zw[pre, 0], zw[pre, 1], color=C_ORANGE, lw=1.4, alpha=0.9,
                label=f"before  ({cone[pre].mean():.2f}°)")
    ax.plot(zw[post, 0], zw[post, 1], color=C_BLUE, lw=0.7, alpha=0.55,
            label=f"after  ({cone[post].mean():.2f}°)")
    if tail.any():
        ax.plot(zw[tail, 0], zw[tail, 1], color=C_AQUA, lw=2.2,
                label=f"settled  ({cone[tail].mean():.2f}°)")
    ax.plot(0, 0, marker="+", ms=10, color=INK_2, lw=1)
    ax.set_aspect("equal")
    ax.set_title("B   Body z-axis tip in world XY — a circle means coning",
                 color=INK, fontsize=11, loc="left", pad=10)
    ax.set_xlabel("world x component", color=INK_2, fontsize=9)
    ax.set_ylabel("world y component", color=INK_2, fontsize=9)
    # Below the axes: the locus is centred and fills the panel, so every
    # in-axes corner overlaps data.
    ax.legend(frameon=False, fontsize=9, labelcolor=INK_2, ncol=3,
              loc="upper center", bbox_to_anchor=(0.5, -0.14))

    # C — body rates. Three series: legend plus direct labels at the right edge.
    ax = axes[1, 0]
    # p and q settle within ~0.5 rad/s of each other, so their end-of-trace
    # labels are nudged apart vertically rather than sitting on top of one
    # another.
    for series, color, lab, dy in ((w[:, 0], C_BLUE, "p  roll rate", 7),
                                   (w[:, 1], C_ORANGE, "q  pitch rate", -7),
                                   (w[:, 2], C_AQUA, "r  yaw rate", 0)):
        ax.plot(t, series, color=color, lw=1.6, label=lab)
        ax.annotate(lab.split()[0], xy=(t[-1], series[-1]), xytext=(6, dy),
                    textcoords="offset points", color=color, fontsize=9,
                    fontweight="bold", va="center")
    ax.set_title("C   Body rates — r is the spin, p and q are the coning",
                 color=INK, fontsize=11, loc="left", pad=10)
    ax.set_xlabel("time [s]", color=INK_2, fontsize=9)
    ax.set_ylabel("rate [rad/s]", color=INK_2, fontsize=9)
    ax.legend(frameon=False, fontsize=9, labelcolor=INK_2, loc="center left")
    mark_cut(ax)

    # D — the distinction that matters: the body cones, the spin axis does not.
    ax = axes[1, 1]
    ax.plot(t, cone, color=C_BLUE, lw=2.0, label="body z-axis off vertical")
    ax.plot(t, spin_tilt, color=C_ORANGE, lw=2.0, label="spin axis off vertical")
    ax.set_title("D   The body cones; the spin axis stays put",
                 color=INK, fontsize=11, loc="left", pad=10)
    ax.set_xlabel("time [s]", color=INK_2, fontsize=9)
    ax.set_ylabel("angle from world vertical [deg]", color=INK_2, fontsize=9)
    ax.legend(frameon=False, fontsize=9, labelcolor=INK_2, loc="upper right")
    mark_cut(ax)
    if tail.any():
        ax.annotate(f"cone {cone[tail].mean():.2f}°   vs   "
                    f"spin axis {np.nanmean(spin_tilt[tail]):.2f}°",
                    xy=(0.98, 0.60), xycoords="axes fraction", ha="right",
                    color=INK, fontsize=9.5, fontweight="bold")

    sub = f"{path.name}"
    if rotor >= 0:
        sub += f"   •   rotor {rotor} stopped"
    elif cut is not None:
        sub += f"   •   failure inferred at t = {cut:.2f} s (not recorded in file)"
    fig.suptitle("Spin axis and coning geometry", color=INK, fontsize=14,
                 fontweight="bold", x=0.007, ha="left", y=0.99)
    fig.text(0.007, 0.955, sub, color=INK_2, fontsize=9.5, ha="left")
    fig.tight_layout(rect=[0, 0, 1, 0.94])

    out = Path(a.out) if a.out else path.with_name(path.stem + "_coning.png")
    fig.savefig(str(out), dpi=150, bbox_inches="tight", facecolor=fig.get_facecolor())
    print(f"wrote {out}")

    # The table view, so nothing here is available only as colour.
    print(f"\n{'window':<22}{'cone [deg]':>12}{'spin tilt [deg]':>17}{'|w| [rad/s]':>13}")
    for lab, m in (("before the cut", pre), ("after the cut", post),
                   (f"settled (last {a.tail:g} s)", tail)):
        if m.any():
            print(f"{lab:<22}{cone[m].mean():12.3f}{np.nanmean(spin_tilt[m]):17.3f}"
                  f"{np.linalg.norm(w[m], axis=1).mean():13.3f}")
    if tail.any():
        print(f"\nsettled cone radius on the unit sphere: "
              f"{np.sin(np.radians(cone[tail].mean())):.5f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
