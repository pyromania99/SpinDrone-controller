#!/usr/bin/env python3
"""
analyse_envelope.py - measure the slow attitude envelope and the theta_s ringdown from a
level-1 npz, replacing the figures in KM6E7_LEVEL1.md section 4a that were read off a PNG.

Reports, for the adaptive arm:
  * lobe peak times and amplitudes of the roll envelope
  * the lobe period, and the log decrement / damping ratio after removing the floor
  * theta_s: overshoot peak, converged value, and its own ringdown period
  * the ratio of converged theta_s to psi_model = atan(r*T)

Usage: python3 analyse_envelope.py ARM.npz [--rotor-tau 0.02] [--floor auto]
"""
from __future__ import annotations

import argparse
import math

import numpy as np


def envelope_peaks(t, y, win_s=0.6, min_sep_s=3.0):
    """Peaks of the |y| envelope. The signal is a fast spin-synchronous carrier under a
    slow envelope, so take a running max over ~one carrier period first, then find the
    local maxima of that."""
    dt = float(np.median(np.diff(t)))
    n = max(int(win_s / dt), 1)
    pad = np.pad(np.abs(y), (n // 2, n // 2), mode="edge")
    env = np.array([pad[i:i + n].max() for i in range(len(y))])
    sep = max(int(min_sep_s / dt), 1)
    idx = [i for i in range(sep, len(env) - sep)
           if env[i] == env[max(0, i - sep):i + sep].max()]
    # collapse plateaus
    keep = []
    for i in idx:
        if not keep or t[i] - t[keep[-1]] > min_sep_s:
            keep.append(i)
    return np.array(keep, dtype=int), env


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("npz")
    ap.add_argument("--rotor-tau", type=float, default=0.020)
    ap.add_argument("--floor", default="auto")
    a = ap.parse_args()

    d = np.load(a.npz, allow_pickle=True)
    t = np.asarray(d["time"], float)
    roll = np.asarray(d["desired_angles"], float)[:, 0] - np.asarray(d["eR"], float)[:, 0]
    roll_deg = np.degrees(roll)
    r = np.asarray(d["yaw_rate"], float)
    th = np.asarray(d["seat_theta"], float) if "seat_theta" in d else None

    rss = float(np.median(r[len(r) // 3:]))
    psi = math.atan(abs(rss) * a.rotor_tau)
    print(f"spin r = {rss:.2f} rad/s   psi_model = atan(rT) = {psi:.4f} rad "
          f"(tau_a term excluded: level 1 has no transport delay)")

    idx, env = envelope_peaks(t, roll_deg)
    idx = idx[t[idx] > 2.0]
    print(f"\nroll envelope: {len(idx)} lobes")
    print("  t [s]   :", " ".join(f"{t[i]:6.1f}" for i in idx[:12]))
    print("  peak[d] :", " ".join(f"{env[i]:6.2f}" for i in idx[:12]))
    if len(idx) > 2:
        gaps = np.diff(t[idx])
        print(f"  spacing: {np.mean(gaps):.2f} +/- {np.std(gaps):.2f} s "
              f"-> {1/np.mean(gaps):.4f} Hz")
        floor = (float(np.percentile(env[t > t[-1] * 0.6], 50))
                 if a.floor == "auto" else float(a.floor))
        amp = env[idx] - floor
        ok = amp > 0
        print(f"  floor (median envelope, last 40%) = {floor:.3f} deg")
        if ok.sum() > 2:
            ld = np.diff(np.log(amp[ok]))
            delta = float(-np.mean(ld))
            zeta = delta / math.sqrt(4 * math.pi ** 2 + delta ** 2)
            print(f"  log decrement delta = {delta:.3f} -> zeta = {zeta:.3f}")
    fin = env[t > t[-1] * 0.9]
    print(f"  envelope over last 10%: {fin.mean():.3f} deg (still decaying?"
          f" first-half {env[(t>t[-1]*0.5)&(t<t[-1]*0.7)].mean():.3f})")

    if th is not None and np.any(th != 0):
        settled = th[t > t[-1] * 0.8]
        pk = int(np.argmax(np.abs(th)))
        print(f"\ntheta_s: peak {th[pk]:+.4f} rad at t = {t[pk]:.1f} s")
        print(f"         converged {settled.mean():+.4f} +/- {settled.std():.4f} rad")
        over = abs(th[pk] - settled.mean()) / max(abs(settled.mean()), 1e-9)
        print(f"         overshoot {100*over:.0f}% ; peak is {100*abs(th[pk])/psi:.0f}% of psi_model")
        print(f"         converged / psi_model = {abs(settled.mean())/psi:.3f}")
        ti, _e = envelope_peaks(t, th - settled.mean(), win_s=0.2, min_sep_s=3.0)
        ti = ti[t[ti] > 2.0]
        if len(ti) > 2:
            g = np.diff(t[ti])
            print(f"         ringdown period {np.mean(g):.2f} +/- {np.std(g):.2f} s")


if __name__ == "__main__":
    main()
