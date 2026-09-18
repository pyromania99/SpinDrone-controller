#!/usr/bin/env python3
"""
plot_level1.py - the standard figure set for a LEVEL 1 run, from its .npz.

Deliberately thin: it converts the level-1 npz into the same dict layout plot_run.py
builds from a ulog, then calls plot_run.py's own figure functions. So the level-1,
level-2 and level-3 panels are drawn by identical code and are directly comparable -
which is the whole point of having a standard set.

THE ATTITUDE RECONSTRUCTION
    The npz holds NO raw attitude. eigen_controller.py logs
        desired_angles = [desired_roll, desired_pitch, yaw]   <- slot 2 is MEASURED yaw
        eR             = [roll_error, pitch_error, 0]         <- error = desired - actual
    so actual roll/pitch = desired_angles[:, :2] - eR[:, :2], and yaw is taken straight
    from slot 2. Getting this backwards silently flips the sign of every attitude panel.

Usage:
    python3 plot_level1.py --out DIR --prefix NAME --rotor-tau 0.02 \
        --arm "seat OFF:path/to/eigen_off.npz" \
        --arm "seat ADAPTIVE:path/to/eigen_adaptive.npz"
"""
from __future__ import annotations

import argparse
import os

import numpy as np

import plot_run as P


def load_npz(path, rotor_tau):
    d = np.load(path, allow_pickle=True)
    t = np.asarray(d["time"], dtype=float)
    p = np.asarray(d["p"], dtype=float)
    ep = np.asarray(d["ep"], dtype=float)
    da = np.asarray(d["desired_angles"], dtype=float)
    eR = np.asarray(d["eR"], dtype=float)
    w = np.asarray(d["angular_velocity"], dtype=float)
    tau = np.asarray(d["torque"], dtype=float)

    roll = da[:, 0] - eR[:, 0]
    pitch = da[:, 1] - eR[:, 1]
    yaw = da[:, 2]

    def _zb(phi, th, psi):
        """Body z in world, ZYX yaw-pitch-roll. Commanded and actual share the MEASURED
        yaw, so the angle between them is a roll/pitch error and not a yaw difference -
        which matters here, because yaw is deliberately spinning."""
        return np.stack([np.cos(phi) * np.sin(th) * np.cos(psi) + np.sin(phi) * np.sin(psi),
                         np.cos(phi) * np.sin(th) * np.sin(psi) - np.sin(phi) * np.cos(psi),
                         np.cos(phi) * np.cos(th)], axis=-1)

    # tilt is what the vehicle did; att_err is what it was asked for minus that. In this
    # hover they are the same to a hundredth of a degree, but the trajectory figures at
    # levels 2 and 3 showed a ~19 deg COMMANDED lean labelled as error, so both are
    # carried everywhere now. See plot_run.load.
    za = _zb(roll, pitch, yaw)
    zc = _zb(da[:, 0], da[:, 1], yaw)
    tilt_cmd = np.arccos(np.clip(zc[:, 2], -1, 1))
    att_err = np.arccos(np.clip(np.sum(za * zc, axis=1), -1, 1))
    # tilt from vertical, the same frame-independent quantity plot_run.py takes from the
    # quaternion. For small-to-moderate angles cos(tilt) = cos(roll)cos(pitch).
    tilt = np.arccos(np.clip(np.cos(roll) * np.cos(pitch), -1, 1))

    v = np.gradient(p[:, :2], t, axis=0) if len(t) > 2 else np.zeros_like(p[:, :2])

    out = {
        "name": os.path.basename(path),
        "params": {"MC_POLE_EN": 0},
        "cfg": {"rotor_tau": rotor_tau},
        "t_pos": t, "x": p[:, 0], "y": p[:, 1], "alt": p[:, 2],
        "speed": np.hypot(v[:, 0], v[:, 1]),
        "ep": np.hypot(ep[:, 0], ep[:, 1]),
        "t_att": t, "roll": roll, "pitch": pitch, "yaw": yaw, "tilt": tilt,
        "tilt_cmd": tilt_cmd, "att_err": att_err,
        "t_w": t, "pq": np.hypot(w[:, 0], w[:, 1]), "r": np.asarray(d["yaw_rate"], float),
        "t_cs": t, "tau_xy": np.hypot(tau[:, 0], tau[:, 1]),
        "seat_theta": np.asarray(d["seat_theta"], float) if "seat_theta" in d else None,
        "seat_alpha": np.asarray(d["seat_alpha"], float) if "seat_alpha" in d else None,
        "seat_stepped": np.asarray(d["seat_stepped"], float) if "seat_stepped" in d else None,
        "pole_theta": np.asarray(d["pole_theta"], float) if "pole_theta" in d else None,
    }
    dp = np.asarray(d["desired_p"], dtype=float)
    if dp.size:
        out.update(t_sp=t, sp_x=dp[:, 0], sp_y=dp[:, 1], sp_z=-dp[:, 2])
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", action="append", required=True, metavar="LABEL:NPZ")
    ap.add_argument("--out", required=True)
    ap.add_argument("--prefix", required=True)
    ap.add_argument("--title", default="")
    ap.add_argument("--rotor-tau", type=float, default=0.020)
    ap.add_argument("--zoom", nargs=2, type=float, default=[0, 15])
    ap.add_argument("--settle-from", type=float, default=60.0)
    ap.add_argument("--no-crash-cut", dest="crash_cut", action="store_false", default=True,
                    help="Keep the post-crash data (see plot_run.apply_crash_cut).")
    ap.add_argument("--alt-threshold", type=float, default=1.0,
                    help="'airborne' threshold [m]. Level 1 hovers at 1.5 m, so the "
                         "level-2/3 default of 4 m would report every arm as grounded.")
    a = ap.parse_args()

    arms = []
    for spec in a.arm:
        label, _, path = spec.partition(":")
        d = load_npz(path, a.rotor_tau)
        if a.crash_cut:
            P.apply_crash_cut(d, P.crash_time(d), t_start=P.takeoff_time(d))
        arms.append((label, d))

    os.makedirs(a.out, exist_ok=True)
    base = a.title or a.prefix
    made = [
        P.fig_compare(arms, f"{a.out}/{a.prefix}_compare.png", f"{base} — flight comparison", a.zoom),
        P.fig_attitude(arms, f"{a.out}/{a.prefix}_attitude.png", f"{base} — attitude vs time", a.zoom, a.settle_from),
        P.fig_seat(arms, f"{a.out}/{a.prefix}_seat.png", f"{base} — seat angle and observable"),
    ]
    # Station keeping. The npz carries desired_p, so level 1 can answer the same "did it
    # track?" question as levels 2 and 3 - on a hover the commanded path is a point and the
    # panel reads as drift about it. This was missing until 2026-09-17, which is why the
    # level-1 column was absent from the trajectory comparison sheet.
    if any("sp_x" in d for _l, d in arms):
        made.append(P.fig_trajectory(arms, f"{a.out}/{a.prefix}_trajectory.png",
                                     f"{base} — trajectory tracking", a.settle_from))
    # The adaptive arm on its own axes. On a shared-axis attitude figure the arms that
    # diverge reach +/-180 deg and squash the survivor to a flat line; this is the whole
    # reason the archived km6e7_attitude_adaptive.png exists.
    pick = P.pick_adaptive_arm(arms)
    got = P.fig_attitude_adaptive(pick[0], pick[1],
                                  f"{a.out}/{a.prefix}_attitude_adaptive.png",
                                  f"{base} — {pick[0]}, attitude on its own axes",
                                  a.settle_from)
    if got:
        made.append(got)
    for label, d in arms:
        f, l, n = P.airborne(d, thresh=a.alt_threshold)
        span = f"{l - f:.1f}s above {a.alt_threshold:g} m, {n} crossings" if f is not None else "never airborne"
        cut = f"  [cut at t={d['t_crash']:.1f}s]" if d.get("t_crash") is not None else ""
        print(f"  {label:22s} {d['name']:26s} {span}{cut}")
    for m in made:
        print("wrote", m)


if __name__ == "__main__":
    main()
