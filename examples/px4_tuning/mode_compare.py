#!/usr/bin/env python3
"""
mode_compare.py — measure the spin-proportional roll/pitch mode identically in a PX4 ulog
and in a Pegasus-only .npz, so the two stacks can be compared like for like.

Why this exists
───────────────
Six PX4 flights, two different control laws (MC_CTRL_ALG 1 and 3), MC_EIG_ATT_P from 8 to
2 and MC_EIG_B from +19.3 to -5.0, all show the same roll/pitch mode, and its frequency
tracks the spin rate:

    r = 7.60  ->  10.70 rad/s        r = 9.47  ->  13.30 rad/s
    mode = 1.390 * r + 0.13,  the two ratios agreeing to 0.3%

No controller parameter moves it, so it is a property of the vehicle, not of the loop. It
is NOT free rigid-body nutation either: from the real asymmetric inertias that is 0.895*r,
and the measurement is 1.57x higher. The open question is whether the Pegasus-only stack -
which flies the same USD vehicle with no HIL transport and no 250 Hz discretisation - shows
the same mode. If it does, the mode is the vehicle and PX4 is merely less tolerant of it.
If it does not, the mode belongs to the PX4 path.

THE FRAME TRAP, which is the whole reason this is one tool and not two
─────────────────────────────────────────────────────────────────────
Pegasus logs body rates in ENU/FLU; PX4 logs them in NED/FRD. The conversion negates y
and z:

    p_frd = p_flu        q_frd = -q_flu        r_frd = -r_flu

so the complex rate Omega = p + i*q maps to its own CONJUGATE:

    Omega_frd = conj(Omega_flu)     and      r_frd = -r_flu

Both the mode's signed frequency AND the spin rate flip sign. A mode that is retrograde in
PX4 therefore reads as PROGRADE in a raw Pegasus log, and comparing the two signs directly
would "prove" the mode is absent when it is present.

The frame-INVARIANT quantity is the ratio mode/r, because numerator and denominator negate
together. That is what this reports, and it is what should be compared across stacks.

Usage
─────
  mode_compare.py flight.ulg                       # PX4 (FRD assumed)
  mode_compare.py run.npz                          # Pegasus (FLU assumed, converted)
  mode_compare.py a.ulg b.npz --window 20 60
"""
from __future__ import annotations

import argparse
import sys

import numpy as np


def _spectrum(p: np.ndarray, q: np.ndarray, dt: float) -> float:
    """Signed peak frequency of the complex signal p + i*q, in rad/s.

    Complex, not |p,q|: a quadrature mode's magnitude envelope is rectified and peaks at
    twice the true frequency. Linearly detrended so a growing amplitude does not put all
    its energy in the lowest bins and win the argmax.
    """
    n = len(p)
    if n < 64 or not np.isfinite(dt) or dt <= 0:
        return float("nan")
    t = np.arange(n) * dt
    sig = ((p - np.polyval(np.polyfit(t, p, 1), t))
           + 1j * (q - np.polyval(np.polyfit(t, q, 1), t)))
    if np.abs(sig).std() < 1e-9:
        return float("nan")
    freqs = np.fft.fftfreq(n, d=dt)
    amp = np.abs(np.fft.fft(sig))
    amp[np.abs(freqs) <= 3.0 / (n * dt)] = 0.0     # drift and window edges are not modes
    return float(2.0 * np.pi * freqs[np.argmax(amp)])


def _report(label: str, t: np.ndarray, pqr: np.ndarray, frame: str,
            window: tuple[float, float] | None) -> None:
    """pqr is (N,3) in `frame` ('frd' or 'flu'); converted to FRD before measuring."""
    if frame == "flu":
        pqr = pqr * np.array([1.0, -1.0, -1.0])    # FLU -> FRD: negate y and z

    t = t - t[0]
    lo, hi = window if window else (t[0], t[-1])
    m = (t >= lo) & (t <= hi)
    if m.sum() < 128:
        print(f"{label}: only {m.sum()} samples in {lo:.1f}-{hi:.1f}s - too few")
        return

    p, q, r = pqr[m, 0], pqr[m, 1], pqr[m, 2]
    tt = t[m]
    dt = float(np.median(np.diff(tt)))
    env = np.hypot(p, q)

    # Below this there is no mode, only the spectrum of noise - which still has a peak,
    # and reporting it as "the dominant mode" invites chasing it. The Pegasus-only run
    # sits at 0.006 rad/s rms and produced a confident-looking 92 rad/s that means nothing.
    if env.std() < 0.02:
        print(f"{label}")
        print(f"   window        {lo:.1f}-{hi:.1f}s   spin r {np.mean(r):+.3f} rad/s")
        print(f"   coning rms    {env.std():.4f} rad/s  -- BELOW the 0.02 noise floor")
        print(f"   mode          none. No roll/pitch mode is present at this spin.")
        print(f"   PX4 reference at the same spin: coning 0.245 rms, mode/r = -1.40")
        return

    freq = _spectrum(p, q, dt)
    r_mean = float(np.mean(r))
    growth = float(np.polyfit(tt, np.log(np.maximum(env, 1e-9)), 1)[0])

    # Sign convention: report the ratio against |r| with the mode's sign relative to the
    # spin direction, so "retrograde" means the same thing whatever the spin sign is.
    rel = freq / r_mean if abs(r_mean) > 1e-6 else float("nan")
    sense = "retrograde" if rel < 0 else "prograde"

    print(f"{label}")
    print(f"   window        {lo:.1f}-{hi:.1f}s  ({m.sum()} samples, dt {dt*1e3:.2f} ms)")
    print(f"   spin r        {r_mean:+.3f} rad/s")
    print(f"   mode          {freq:+.2f} rad/s   ({abs(freq)/(2*np.pi):.2f} Hz, {sense})")
    print(f"   mode / r      {rel:+.3f}      <-- FRAME-INVARIANT, compare THIS across stacks")
    print(f"   coning rms    {env.std():.4f} rad/s   growth {growth:+.3f}/s"
          + (f"  (doubling {np.log(2)/growth:.1f}s)" if growth > 0.01 else ""))
    print(f"   PX4 reference: mode/r = -1.40 +/- 0.05 over r = 7.5-9.5, both control laws")


def _load(path: str):
    if path.endswith(".npz"):
        d = np.load(path)
        if "angular_velocity" not in d or d["angular_velocity"].shape[0] < 2:
            sys.exit(f"{path}: no 'angular_velocity' - the run needs a results_file and "
                     f"EigenController.stop() must have been reached")
        return np.asarray(d["time"], float), np.asarray(d["angular_velocity"], float), "flu"

    from pyulog import ULog
    u = ULog(path, ["vehicle_angular_velocity"])
    if not u.data_list:
        sys.exit(f"{path}: no vehicle_angular_velocity")
    av = u.data_list[0]
    t = av.data["timestamp"] * 1e-6
    pqr = np.column_stack([av.data[f"xyz[{i}]"] for i in range(3)])
    return t, pqr, "frd"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="+", help=".ulg (PX4, FRD) or .npz (Pegasus, FLU)")
    ap.add_argument("--window", nargs=2, type=float, metavar=("START", "END"),
                    help="seconds from the start of the record")
    args = ap.parse_args()
    win = tuple(args.window) if args.window else None

    for path in args.files:
        try:
            t, pqr, frame = _load(path)
            _report(f"=== {path}  [{frame.upper()}]", t, pqr, frame, win)
        except SystemExit:
            raise
        except Exception as exc:
            print(f"=== {path}\n   FAILED: {exc}")
        print()


if __name__ == "__main__":
    main()
