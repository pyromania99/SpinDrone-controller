#!/usr/bin/env python3
"""
plot_run.py - the standard figure set for a PX4 run (level 2 or level 3), from its ulog.

Produces the same four views for every arm/level so they can be laid side by side:

    <prefix>_compare.png      8 panels: altitude, |p,q|, tilt error, xy error, torque, r + theta_s
    <prefix>_attitude.png     roll / pitch / yaw / roll-pitch phase portrait
    <prefix>_seat.png         theta_s on its own axes, with alpha and the model angle
    <prefix>_trajectory.png   flown vs commanded path, following distance, speed

Mirrors the level-1 figures in ../hover_level1_km6e7_seat_off_fixed_adaptive/ panel for
panel, so a level-1 / level-2 / level-3 comparison is a matter of putting three PNGs next
to each other.

WHY GROUND TRUTH EVERYWHERE
    Attitude and position are read from *_groundtruth. Level 2 flies on ground truth by
    definition (MC_CTRL_GT=1), and for level 3 the estimator is what is under test, so
    scoring it against its own estimate would be circular. See
    notes/EKF2_OUTPUT_PREDICTOR_SPIN_FIX.md for why vehicle_attitude was unusable at spin
    before 2026-09-13 - that failure is exactly what using GT here avoids.

THE TIMESTAMP TRAP
    mc_controller_status and trajectory_setpoint carry occasional UINT64_MAX timestamps.
    np.interp requires sorted x and returns silent garbage (exact zeros, in one observed
    case) if it is not. _series() filters and re-sorts every topic on the way out. Do not
    remove that.

Usage:
    python3 plot_run.py --out DIR --prefix NAME \
        --arm "seat OFF:px4_runs/L2_eigen_20260913_201119" \
        --arm "seat ON:px4_runs/L2_eigen_20260913_201321" \
        [--traj examples/results/fast_circle_l2.csv] [--zoom 0 20] [--title "..."]
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pyulog

TMAX_US = 1 << 60          # anything above this is the uninitialised-timestamp garbage
COLORS = ["tab:blue", "tab:orange", "tab:red", "tab:green", "tab:purple"]


# ---------------------------------------------------------------- loading

def _series(ulog, topic, fields):
    """(t_seconds_from_first_sample, {field: array}), garbage timestamps removed."""
    ds = next((d for d in ulog.data_list if d.name == topic), None)
    if ds is None:
        return None, {}
    t = ds.data["timestamp"].astype(np.float64)
    ok = (t > 0) & (t < TMAX_US)
    order = np.argsort(t[ok])
    t = t[ok][order]
    out = {}
    for f in fields:
        if f in ds.data:
            out[f] = np.asarray(ds.data[f], dtype=np.float64)[ok][order]
    return t, out


def _euler(qw, qx, qy, qz):
    """roll, pitch, yaw [rad] from a PX4 (w, x, y, z) quaternion."""
    roll = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = np.arcsin(np.clip(2 * (qw * qy - qz * qx), -1, 1))
    yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return roll, pitch, yaw


def load(run_dir):
    """Everything the figures need, on a common time origin (first GT position sample)."""
    ulg = glob.glob(os.path.join(run_dir, "log", "*", "*.ulg"))
    if not ulg:
        raise SystemExit(f"no ulog under {run_dir}")
    u = pyulog.ULog(ulg[0])

    params = dict(u.initial_parameters)
    for _ts, name, val in u.changed_parameters:
        params[name] = val
    try:
        cfg = json.load(open(os.path.join(run_dir, "run_config.json")))
    except Exception:
        cfg = {}

    tp, pos = _series(u, "vehicle_local_position_groundtruth", ["x", "y", "z", "vx", "vy"])
    if tp is None:
        raise SystemExit(f"{run_dir}: no vehicle_local_position_groundtruth")
    t0 = tp[0]

    ta, att = _series(u, "vehicle_attitude_groundtruth", ["q[0]", "q[1]", "q[2]", "q[3]"])
    tw, w = _series(u, "vehicle_angular_velocity_groundtruth", ["xyz[0]", "xyz[1]", "xyz[2]"])
    tc, cs = _series(u, "mc_controller_status",
                     ["seat_theta", "seat_alpha", "pole_theta_b",
                      "torque_sp[0]", "torque_sp[1]"])
    ts_, sp = _series(u, "trajectory_setpoint", ["position[0]", "position[1]", "position[2]"])

    d = {"name": os.path.basename(run_dir), "params": params, "cfg": cfg,
         "t_pos": (tp - t0) / 1e6,
         "x": pos["x"], "y": pos["y"], "alt": -pos["z"],
         "speed": np.hypot(pos.get("vx", np.zeros_like(tp)), pos.get("vy", np.zeros_like(tp)))}

    if att:
        roll, pitch, yaw = _euler(att["q[0]"], att["q[1]"], att["q[2]"], att["q[3]"])
        # tilt = angle between body z and world z; the frame-independent attitude error
        # for a vehicle whose yaw is deliberately spinning.
        tilt = np.arccos(np.clip(1 - 2 * (att["q[1]"] ** 2 + att["q[2]"] ** 2), -1, 1))
        d.update(t_att=(ta - t0) / 1e6, roll=roll, pitch=pitch, yaw=yaw, tilt=tilt)
    if w:
        d.update(t_w=(tw - t0) / 1e6,
                 pq=np.hypot(w["xyz[0]"], w["xyz[1]"]), r=w["xyz[2]"])
    if cs:
        d.update(t_cs=(tc - t0) / 1e6,
                 seat_theta=cs.get("seat_theta"), seat_alpha=cs.get("seat_alpha"),
                 pole_theta=cs.get("pole_theta_b"),
                 tau_xy=np.hypot(cs.get("torque_sp[0]", np.zeros_like(tc)),
                                 cs.get("torque_sp[1]", np.zeros_like(tc))))
    if sp:
        d.update(t_sp=(ts_ - t0) / 1e6,
                 sp_x=sp["position[0]"], sp_y=sp["position[1]"], sp_z=sp["position[2]"])
        # position error, setpoint interpolated onto the GT clock
        ep = np.hypot(np.interp(d["t_pos"], d["t_sp"], d["sp_x"]) - d["x"],
                      np.interp(d["t_pos"], d["t_sp"], d["sp_y"]) - d["y"])
        # Blank it while the commanded altitude is still on the ground: before the
        # trajectory streams, the setpoint is (0,0,0) and the "error" is just the
        # vehicle's distance from the origin, which is not a tracking measurement.
        sp_alt = -np.interp(d["t_pos"], d["t_sp"], d["sp_z"])
        d["ep"] = np.where(sp_alt > 1.0, ep, np.nan)
    return d


# ---------------------------------------------------------------- helpers

def airborne(d, thresh=4.0):
    """(first, last, n_crossings) above `thresh` m. Crossings distinguish a flight from
    a bounce sequence - a first/last duration alone cannot, and that mistake has already
    produced one wrong result in this project."""
    a = d["alt"] > thresh
    if not a.any():
        return None, None, 0
    t = d["t_pos"]
    return t[a][0], t[a][-1], int((np.diff(a.astype(int)) != 0).sum())


def crash_time(d, climb_to=None, floor=None):
    """When the vehicle stopped flying, or None if it never did.

    Defined as the first sample AFTER it has climbed above `climb_to` at which altitude
    falls below `floor`. Defaults scale off the run's own hover altitude, so the same rule
    works for a 1.5 m level-1 hover and a 6 m level-2 circle.

    Everything after this instant is ground contact: tumbling attitude, saturated torque,
    and a position "error" that is really the reference walking away from a wreck. Those
    values are an order of magnitude larger than anything in the flight and set the axis
    limits of every panel if they are left in."""
    alt = d["alt"]
    hover = float(np.percentile(alt, 95))
    climb_to = 0.6 * hover if climb_to is None else climb_to
    floor = max(0.25 * hover, 0.3) if floor is None else floor
    up = np.flatnonzero(alt > climb_to)
    if up.size == 0:
        return None
    # The first loss of altitude that is not recovered as CONTROLLED flight.
    #
    # Two cases must be told apart and altitude alone cannot do it:
    #   * level 2, rotor_tau=0, seat off: dips to 0.11 m at t~16 s, then returns to
    #     6.09 m and flies stably for 40 more seconds. A recovery - do not cut.
    #   * level 3, rotor_tau=0.02, seat off: loses it at t~16 s and then BOUNCES, with
    #     peaks up to 8.4 m. Altitude comes back, control does not. Cut at the first loss.
    # So recovery is judged on body rates, not height: the vehicle must get back above
    # climb_to AND hold |p,q| below `rate_ok` for at least `hold_s` before it counts.
    rate_ok, hold_s = 1.0, 3.0
    t = d["t_pos"]
    tw, pq = d.get("t_w"), d.get("pq")
    down = np.flatnonzero((alt < floor) & (np.arange(len(alt)) > up[0]))
    for i in down:
        after = slice(i, None)
        if alt[after].max() < climb_to:
            return float(t[i])                       # never gets back up at all
        if tw is None or pq is None:
            continue                                 # no rates logged: fall back to altitude
        # Is there a sustained, calm, airborne stretch after this loss?
        calm = (tw > t[i]) & (pq < rate_ok) & (np.interp(tw, t, alt) > climb_to)
        if not calm.any():
            return float(t[i])
        idx = np.flatnonzero(calm)
        splits = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
        if not any(tw[g[-1]] - tw[g[0]] >= hold_s for g in splits if g.size):
            return float(t[i])                       # bounced, never flew again
    return None


def takeoff_time(d, frac=0.05):
    """When the vehicle left the ground, or None. Before this the vehicle is disarmed or
    spooling: |p,q| sits at a ~1e-6 rad/s numerical floor which, on a log axis, costs six
    decades of range and squashes the whole flight into the top of the panel."""
    alt = d["alt"]
    hover = float(np.percentile(alt, 95))
    up = np.flatnonzero(alt > max(frac * hover, 0.15))
    return float(d["t_pos"][up[0]]) if up.size else None


def apply_crash_cut(d, t_cut, alt_tail=2.0, t_start=None):
    """Blank every series outside the flight. The altitude trace is kept `alt_tail`
    seconds past `t_cut` so the fall itself stays visible - the point is to drop the
    post-impact data, not to hide that a crash happened."""
    if t_cut is None and t_start is None:
        return d
    pairs = [("t_pos", ("x", "y", "speed", "ep")), ("t_att", ("roll", "pitch", "yaw", "tilt")),
             ("t_w", ("pq", "r")), ("t_cs", ("seat_theta", "seat_alpha", "pole_theta", "tau_xy"))]
    for tkey, fields in pairs:
        if tkey not in d:
            continue
        m = np.zeros_like(d[tkey], dtype=bool)
        if t_cut is not None:
            m |= d[tkey] > t_cut
        if t_start is not None:
            m |= d[tkey] < t_start
        for f in fields:
            v = d.get(f)
            if v is not None and np.shape(v) == np.shape(d[tkey]):
                d[f] = np.where(m, np.nan, np.asarray(v, dtype=float))
    if t_cut is not None:
        d["alt"] = np.where(d["t_pos"] > t_cut + alt_tail, np.nan, d["alt"])
    d["t_crash"] = t_cut
    d["t_takeoff"] = t_start
    return d


def _seat_is_active(theta):
    """Was the seat actually running on this arm?

    NOT `np.any(theta != 0)`: the flight-window trim writes NaNs into the series, and
    `NaN != 0` is True, so that test picks the seat-OFF arm. Compare only finite samples."""
    if theta is None:
        return False
    t = np.asarray(theta, dtype=float)
    ok = np.isfinite(t)
    return bool(ok.any() and np.abs(t[ok]).max() > 1e-9)


def _crash_t(d):
    """This arm's crash time, or None if it never crashed. Uses the cut that was already
    applied, and recomputes it from the raw altitude when it was not (`--no-crash-cut`)."""
    return d["t_crash"] if "t_crash" in d else crash_time(d)


def pick_adaptive_arm(arms):
    """The arm to draw on its own axes: the one whose seat is running AND flying.

    NOT simply the first arm with a live seat. A FIXED arm holds a non-zero theta_s too,
    and in an off/fixed/adaptive trio it comes first - so that rule drew the arm that
    crashed at 4 s, labelled the figure with its own name, and published it as the
    adaptive one (km6e7_refly_attitude_adaptive.png, caught 2026-09-17).

    Prefer a live-seat arm that never crashed, first one given (so a run with two
    survivors - e.g. level 3's OFF / ON / ON+b trio - keeps picking the same one it
    always did). If every live-seat arm crashed, take the one that flew longest.
    """
    live = [(lb, d) for lb, d in arms if _seat_is_active(d.get("seat_theta"))]
    if not live:
        return arms[-1]
    flew = [kv for kv in live if _crash_t(kv[1]) is None]
    return flew[0] if flew else max(live, key=lambda kv: float(_crash_t(kv[1])))


def unwrap_nan(y):
    """np.unwrap, but NaN-safe.

    np.unwrap works on successive differences, so a single NaN poisons everything after
    it - and the flight-window trim puts NaNs at the START of every trimmed series, which
    silently blanks the entire yaw panel. Unwrap the finite samples only and put them back
    in place."""
    y = np.asarray(y, dtype=float)
    out = np.full_like(y, np.nan)
    ok = np.isfinite(y)
    if ok.any():
        out[ok] = np.unwrap(y[ok])
    return out


def _semilog(ax, t, y, **kw):
    y = np.asarray(y, dtype=float)
    ax.semilogy(t, np.maximum(np.abs(y), 1e-6), **kw)


def _roll_nanmean(y, t, win_s):
    """NaN-aware rolling mean over a `win_s` window. The NaNs are the frozen-gate ticks,
    and dropping them is correct: a frozen tick is a missing measurement, not a zero."""
    y = np.asarray(y, dtype=float)
    ok = np.isfinite(y).astype(float)
    val = np.where(np.isfinite(y), y, 0.0)
    dt = np.median(np.diff(t)) if len(t) > 2 else 1.0
    n = max(int(win_s / max(dt, 1e-9)), 1)
    k = np.ones(n)
    num = np.convolve(val, k, mode="same")
    den = np.convolve(ok, k, mode="same")
    return np.where(den > 0, num / np.maximum(den, 1e-9), np.nan)


def psi_model(r, tau_a, T):
    return abs(r) * tau_a + math.atan(abs(r) * T)


def cruise_spin(arms):
    """Median |r| over the airborne window of the LONGEST-FLYING arm.

    Taking it over a whole record is wrong: a crashed arm sits at r = 0 for most of its
    log, which drags the median to zero and makes psi_model read 0.000."""
    best, best_span = None, -1.0
    for _label, d in arms:
        if "r" not in d:
            continue
        f, l, _n = airborne(d)
        if f is None or (l - f) <= best_span:
            continue
        m = (d["t_w"] >= f) & (d["t_w"] <= l)
        if m.any():
            best, best_span = float(np.median(np.abs(d["r"][m]))), l - f
    return best


def flight_window(arms, pad=2.0):
    """(t_first_climb - pad, t_last_sample): trims the pre-arm idle period, whose 1e-6
    rad/s noise floor otherwise sets the y-range of every log panel."""
    starts, ends = [], []
    for _label, d in arms:
        a = d["alt"] > 1.0
        if a.any():
            starts.append(d["t_pos"][a][0])
        ends.append(d["t_crash"] + 3.0 if d.get("t_crash") is not None else d["t_pos"][-1])
    return (min(starts) - pad if starts else 0.0), max(ends)


# ---------------------------------------------------------------- figures

def fig_compare(arms, out, title, zoom):
    fig, ax = plt.subplots(4, 2, figsize=(15, 15))
    fig.suptitle(title, fontsize=13)
    for i, (label, d) in enumerate(arms):
        c = COLORS[i % len(COLORS)]
        ax[0, 0].plot(d["t_pos"], d["alt"], c, lw=0.9, label=label)
        ax[0, 1].plot(d["t_pos"], d["alt"], c, lw=0.9, label=label)
        if "pq" in d:
            _semilog(ax[1, 0], d["t_w"], d["pq"], color=c, lw=0.5, label=label)
            _semilog(ax[1, 1], d["t_w"], d["pq"], color=c, lw=0.5, label=label)
        if "tilt" in d:
            _semilog(ax[2, 0], d["t_att"], d["tilt"], color=c, lw=0.5, label=label)
        if "ep" in d:
            _semilog(ax[2, 1], d["t_pos"], d["ep"], color=c, lw=0.7, label=label)
        if "tau_xy" in d:
            _semilog(ax[3, 0], d["t_cs"], d["tau_xy"], color=c, lw=0.4, label=label)
        if "r" in d:
            ax[3, 1].plot(d["t_w"], d["r"], c, lw=0.9, label=f"r, {label}")
    for i, (label, d) in enumerate(arms):
        if d.get("t_crash") is not None:
            for a in (ax[0, 0], ax[0, 1]):
                a.axvline(d["t_crash"], color=COLORS[i % len(COLORS)], ls=":", lw=1.2)
    ax[0, 0].plot([], [], "k:", lw=1.2, label="crash — later data not plotted")
    axr = ax[3, 1].twinx()
    for i, (label, d) in enumerate(arms):
        if d.get("seat_theta") is not None:
            axr.plot(d["t_cs"], d["seat_theta"], COLORS[i % len(COLORS)],
                     ls="--", lw=1.2, label=f"$\\theta_s$, {label}")
    axr.set_ylabel(r"$\theta_s$ [rad] (dashed)")
    rss = cruise_spin(arms)
    if rss is not None:
        cfg = arms[0][1]["cfg"]
        pm = psi_model(rss, 0.0, float(cfg.get("rotor_tau") or 0.0))
        if pm > 0:
            axr.axhline(pm, color="k", ls=":", lw=1)
            axr.text(0.5, pm, f"  model $\\psi$ = {pm:.3f} (rotor pole only)",
                     transform=axr.get_yaxis_transform(), fontsize=8, va="bottom", color="0.3")

    t_lo, t_hi = flight_window(arms)
    for a in (ax[0, 0], ax[1, 0], ax[2, 0], ax[2, 1], ax[3, 0], ax[3, 1]):
        a.set_xlim(t_lo, t_hi)
    for a, t in ((ax[0, 0], "altitude — full flight"), (ax[0, 1], f"altitude — first {zoom[1]:g} s"),
                 (ax[1, 0], "roll/pitch rate |p,q|"), (ax[1, 1], f"|p,q| — first {zoom[1]:g} s"),
                 (ax[2, 0], "tilt from vertical (attitude error)"), (ax[2, 1], "horizontal position error"),
                 (ax[3, 0], "roll/pitch torque"), (ax[3, 1], "spin rate (solid) and $\\theta_s$ (dashed)")):
        a.set_title(t, fontsize=10); a.grid(alpha=.3, which="both"); a.set_xlabel("t [s]")
    for a in (ax[0, 1], ax[1, 1]):
        a.set_xlim(*zoom)
    ax[0, 0].set_ylabel("altitude [m]"); ax[0, 1].set_ylabel("altitude [m]")
    ax[1, 0].set_ylabel("|p,q| [rad/s]"); ax[1, 1].set_ylabel("|p,q| [rad/s]")
    ax[2, 0].set_ylabel("tilt [rad]"); ax[2, 1].set_ylabel("xy error [m]")
    ax[3, 0].set_ylabel(r"$|\tau_{xy}|$ [N m]"); ax[3, 1].set_ylabel("yaw rate r [rad/s]")
    for a in ax.ravel():
        if a.get_legend_handles_labels()[0]:
            a.legend(fontsize=7)
    fig.tight_layout(); fig.savefig(out, dpi=130); plt.close(fig)
    return out


def fig_attitude(arms, out, title, zoom, settle_from):
    fig, ax = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle(title, fontsize=13)
    for i, (label, d) in enumerate(arms):
        if "roll" not in d:
            continue
        c, t = COLORS[i % len(COLORS)], d["t_att"]
        ax[0, 0].plot(t, np.degrees(d["roll"]), c, lw=0.5, label=label)
        ax[0, 1].plot(t, np.degrees(d["roll"]), c, lw=0.6, label=label)
        ax[1, 0].plot(t, np.degrees(d["pitch"]), c, lw=0.5, label=label)
        ax[1, 1].plot(t, np.degrees(d["pitch"]), c, lw=0.6, label=label)
        ax[2, 0].plot(t, np.degrees(unwrap_nan(d["yaw"])), c, lw=0.9, label=label)
        m = t >= settle_from
        if m.any():
            ax[2, 1].plot(np.degrees(d["roll"][m]), np.degrees(d["pitch"][m]),
                          c, lw=0.3, alpha=.8, label=label)
    for a, t in ((ax[0, 0], "roll — full flight"), (ax[0, 1], f"roll — first {zoom[1]:g} s"),
                 (ax[1, 0], "pitch — full flight"), (ax[1, 1], f"pitch — first {zoom[1]:g} s"),
                 (ax[2, 0], "yaw / heading — full flight (unwrapped)"),
                 (ax[2, 1], f"roll–pitch phase portrait (t $\\geq$ {settle_from:g} s)")):
        a.set_title(t, fontsize=10); a.grid(alpha=.3)
    t_lo, t_hi = flight_window(arms)
    for a in (ax[0, 0], ax[1, 0], ax[2, 0]):
        a.set_xlim(t_lo, t_hi)
    for a in (ax[0, 1], ax[1, 1]):
        a.set_xlim(*zoom)
    for a in (ax[0, 0], ax[0, 1]): a.set_ylabel("roll [deg]")
    for a in (ax[1, 0], ax[1, 1]): a.set_ylabel("pitch [deg]")
    ax[2, 0].set_ylabel("yaw [deg]")
    ax[2, 1].set_xlabel("roll [deg]"); ax[2, 1].set_ylabel("pitch [deg]")
    for a in ax.ravel()[:5]: a.set_xlabel("t [s]")
    for a in ax.ravel():
        if a.get_legend_handles_labels()[0]:
            a.legend(fontsize=7)
    fig.tight_layout(); fig.savefig(out, dpi=130); plt.close(fig)
    return out


def fig_attitude_adaptive(label, d, out, title, settle_from=None):
    """One arm's attitude on its OWN axes - the level-1 `km6e7_attitude_adaptive.png` view.

    Exists because a shared-axis multi-arm attitude figure is useless for the arm that
    survives: the diverging arms reach +/-180 deg and squash the survivor to a flat line.
    Rescaled to a single arm, the same data resolves to a fraction of a degree.

    2x2: roll, pitch, unwrapped yaw, and the roll-pitch phase portrait over the settled
    window."""
    t, roll, pitch = d["t_att"], np.degrees(d["roll"]), np.degrees(d["pitch"])
    ok = np.isfinite(roll)
    if not ok.any():
        return None
    t_lo, t_hi = t[ok][0], t[ok][-1]
    # Default the portrait window to the last 40% of the flight, and never let a caller's
    # value fall outside the data - a crashed arm may have only a few seconds.
    sf = settle_from if settle_from is not None else t_lo + 0.6 * (t_hi - t_lo)
    if not (t_lo <= sf < t_hi):
        sf = t_lo + 0.6 * (t_hi - t_lo)

    fig, ax = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle(title, fontsize=13)
    c = "tab:red"
    ax[0, 0].plot(t, roll, c, lw=0.5)
    ax[0, 0].set_title("roll — full flight", fontsize=10); ax[0, 0].set_ylabel("roll [deg]")
    ax[0, 1].plot(t, pitch, c, lw=0.5)
    ax[0, 1].set_title("pitch — full flight", fontsize=10); ax[0, 1].set_ylabel("pitch [deg]")
    yaw = np.degrees(unwrap_nan(d["yaw"]))
    ax[1, 0].plot(t, yaw, c, lw=0.9)
    ax[1, 0].set_title("yaw / heading — full flight (unwrapped)", fontsize=10)
    ax[1, 0].set_ylabel("yaw [deg]")
    m = ok & (t >= sf)
    ax[1, 1].plot(roll[m], pitch[m], c, lw=0.3, alpha=.8)
    ax[1, 1].set_title(f"roll–pitch phase portrait (t $\\geq$ {sf:.0f} s)", fontsize=10)
    ax[1, 1].set_xlabel("roll [deg]"); ax[1, 1].set_ylabel("pitch [deg]")
    ax[1, 1].set_aspect("equal", "datalim")
    for a in (ax[0, 0], ax[0, 1], ax[1, 0]):
        a.set_xlabel("t [s]"); a.set_xlim(t_lo, t_hi)
    for a in ax.ravel():
        a.grid(alpha=.3)
    if m.any():
        rng = max(np.ptp(roll[m]), np.ptp(pitch[m]))
        fig.text(0.5, 0.005,
                 f"{label}   |   settled window t≥{sf:.0f}s: roll {roll[m].mean():+.2f}±{roll[m].std():.2f}°, "
                 f"pitch {pitch[m].mean():+.2f}±{pitch[m].std():.2f}°, full range {rng:.2f}°",
                 ha="center", fontsize=9, color="0.35")
    fig.tight_layout(rect=(0, 0.02, 1, 1)); fig.savefig(out, dpi=130); plt.close(fig)
    return out


def fig_seat(arms, out, title):
    """theta_s on its own axes - the panel the combined figures cannot show legibly."""
    fig, ax = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle(title, fontsize=13)
    for i, (label, d) in enumerate(arms):
        c = COLORS[i % len(COLORS)]
        if d.get("seat_theta") is not None:
            ax[0].plot(d["t_cs"], d["seat_theta"], c, lw=1.1, label=label)
            a = np.asarray(d["seat_alpha"], dtype=float)
            ax[1].plot(d["t_cs"], a, c, lw=0.3, alpha=.18)
            # Per-tick alpha is a dense band at this spin rate; the trend is what says
            # whether the seat is converging, so overlay a ~1 s NaN-aware rolling mean.
            ax[1].plot(d["t_cs"], _roll_nanmean(a, d["t_cs"], 1.0), c, lw=1.4,
                       label=f"{label} (1 s mean)")
            frozen = 100.0 * np.mean(~np.isfinite(a))
            ax[1].plot([], [], " ", label=f"{label}: gate froze {frozen:.0f}% of ticks")
        if d.get("pole_theta") is not None and np.any(d["pole_theta"] != 0):
            ax[2].plot(d["t_cs"], d["pole_theta"], c, lw=1.0, label=f"$\\theta_b$, {label}")
    rss = cruise_spin(arms)
    if rss is not None:
        T = float(arms[0][1]["cfg"].get("rotor_tau") or 0.0)
        if T > 0:
            pm = psi_model(rss, 0.0, T)
            ax[0].axhline(pm, color="k", ls=":", lw=1,
                          label=f"$\\psi$ = atan(rT) = {pm:.3f} (rotor pole only)")
            ax[0].axhline(-pm, color="k", ls=":", lw=1)
    ax[0].set_ylabel(r"$\theta_s$ [rad]")
    ax[0].set_title("seat angle — what the adaptation converged to", fontsize=10)
    ax[1].set_ylabel(r"$\alpha$ [rad]")
    ax[1].set_title(r"measured misalignment $\alpha$ (gaps = allocator gate froze the update, NOT unmeasurable)", fontsize=10)
    ax[2].set_ylabel(r"$\theta_b$ [rad]")
    ax[2].set_title("adaptive pole angle", fontsize=10)
    if not ax[2].get_legend_handles_labels()[0]:
        modes = {str(int(d["params"].get("MC_POLE_EN", 0))) for _l, d in arms}
        ax[2].text(0.5, 0.5, f"adaptive pole disabled in these runs (MC_POLE_EN={'/'.join(sorted(modes))})",
                   transform=ax[2].transAxes, ha="center", va="center", fontsize=11, color="0.45")
        ax[2].set_yticks([])
    ax[2].set_xlabel("t [s]")
    ax[0].set_xlim(*flight_window(arms))
    for a in ax:
        a.grid(alpha=.3)
        if a.get_legend_handles_labels()[0]:
            a.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(out, dpi=130); plt.close(fig)
    return out


def fig_trajectory(arms, out, title, settle_from):
    fig = plt.figure(figsize=(15, 10))
    fig.suptitle(title, fontsize=13)
    axp = fig.add_subplot(2, 2, 1)
    axe = fig.add_subplot(2, 2, 2)
    axs = fig.add_subplot(2, 2, 3)
    axa = fig.add_subplot(2, 2, 4)
    for i, (label, d) in enumerate(arms):
        c = COLORS[i % len(COLORS)]
        if i == 0 and "sp_x" in d:
            axp.plot(d["sp_x"], d["sp_y"], "0.6", lw=1.2, label="commanded")
        axp.plot(d["x"], d["y"], c, lw=0.8, label=label)
        if "ep" in d:
            axe.plot(d["t_pos"], d["ep"], c, lw=0.8, label=label)
        axs.plot(d["t_pos"], d["speed"], c, lw=0.8, label=label)
        axa.plot(d["t_pos"], d["alt"], c, lw=0.9, label=label)
    axp.set_title("flown vs commanded path", fontsize=10)
    axp.set_xlabel("x [m]"); axp.set_ylabel("y [m]"); axp.set_aspect("equal", "datalim")
    axe.set_title("distance from the commanded point (following lag + error)", fontsize=10)
    axe.set_xlabel("t [s]"); axe.set_ylabel("|error| [m]")
    axs.set_title("ground speed", fontsize=10)
    axs.set_xlabel("t [s]"); axs.set_ylabel("speed [m/s]")
    axa.set_title("altitude", fontsize=10)
    axa.set_xlabel("t [s]"); axa.set_ylabel("altitude [m]")
    for a in (axp, axe, axs, axa):
        a.grid(alpha=.3); a.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(out, dpi=130); plt.close(fig)
    return out


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", action="append", required=True, metavar="LABEL:RUN_DIR")
    ap.add_argument("--out", required=True)
    ap.add_argument("--prefix", required=True)
    ap.add_argument("--title", default="")
    ap.add_argument("--zoom", nargs=2, type=float, default=[0, 25])
    ap.add_argument("--settle-from", type=float, default=30.0)
    ap.add_argument("--no-crash-cut", dest="crash_cut", action="store_false", default=True,
                    help="Keep the pre-takeoff and post-crash data. Needed when the "
                         "ground-contact behaviour IS the evidence, e.g. counting bounces.")
    ap.add_argument("--only", nargs="*", default=None,
                    choices=["compare", "attitude", "seat", "trajectory", "adaptive"])
    a = ap.parse_args()

    arms = []
    for spec in a.arm:
        label, _, run = spec.partition(":")
        d = load(run)
        d["airborne_raw"] = airborne(d)          # before any cut: crossings stay truthful
        if a.crash_cut:
            apply_crash_cut(d, crash_time(d), t_start=takeoff_time(d))
        arms.append((label, d))

    os.makedirs(a.out, exist_ok=True)
    base = a.title or a.prefix
    want = a.only or ["compare", "attitude", "seat", "trajectory", "adaptive"]
    made = []
    if "compare" in want:
        made.append(fig_compare(arms, f"{a.out}/{a.prefix}_compare.png",
                                f"{base} — flight comparison", a.zoom))
    if "attitude" in want:
        made.append(fig_attitude(arms, f"{a.out}/{a.prefix}_attitude.png",
                                 f"{base} — attitude vs time", a.zoom, a.settle_from))
    if "seat" in want:
        made.append(fig_seat(arms, f"{a.out}/{a.prefix}_seat.png",
                             f"{base} — seat angle and observable"))
    if "trajectory" in want and any("sp_x" in d for _, d in arms):
        made.append(fig_trajectory(arms, f"{a.out}/{a.prefix}_trajectory.png",
                                   f"{base} — trajectory tracking", a.settle_from))
    if "adaptive" in want:
        pick = pick_adaptive_arm(arms)
        got = fig_attitude_adaptive(pick[0], pick[1],
                                    f"{a.out}/{a.prefix}_attitude_adaptive.png",
                                    f"{base} — {pick[0]}, attitude on its own axes",
                                    a.settle_from)
        if got:
            made.append(got)

    for label, d in arms:
        f, l, n = d.get("airborne_raw") or airborne(d)
        span = f"{l - f:.1f}s above 4 m, {n} crossings" if f is not None else "never above 4 m"
        cut = f"  [cut at t={d['t_crash']:.1f}s]" if d.get("t_crash") is not None else ""
        print(f"  {label:22s} {d['name']:34s} {span}{cut}")
    for m in made:
        print("wrote", m)


if __name__ == "__main__":
    main()
