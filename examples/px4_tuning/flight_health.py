#!/usr/bin/env python3
"""
flight_health.py — is this PX4 flight healthy, and if not, which mode is eating it?

Reduces a ulog to the handful of numbers that separate "flying" from "limit-cycling
while holding attitude error small enough to look fine". That distinction is what cost
the most time on this port: a departure and a healthy hover have similar attitude error;
they differ in torque, motor saturation, and the P/D split of the rate setpoint.

Usage
─────
  python3 flight_health.py <ulog> [<ulog> ...]
  python3 flight_health.py --window 60 run.ulg      # last 60 s of the airborne segment

Reads (SDLOG_PROFILE 25 recommended, but it degrades gracefully):
  mc_controller_status      dt / dt_attitude / dt_position, update_count, debug[0..7]
  vehicle_angular_velocity  coning |p,q| and spin r
  vehicle_torque_setpoint   authority actually used
  actuator_motors           saturation against both rails
  vehicle_local_position    position hold

WHAT THE NUMBERS MEAN

  loop rates      inner is the update_count slope, not a logged-message rate - the
                  logger throttles, so counting messages understates it by 5x.
                  dt_position is the position stage: it is gated on a fresh
                  vehicle_local_position, which EKF2 publishes at 1/EKF2_PREDICT_US
                  rounded to a multiple of the IMU interval (125 Hz at the defaults).

  D/P ratio       the angle stage commands rate_sp = ATT_P*err + ATT_D*d(err)/dt.
                  Above ~1 the derivative path is driving the vehicle, which is a
                  self-excitation: the finite difference's gain rises with frequency,
                  so it feeds whatever it is already ringing at. FINDINGS section 8
                  measured 8.4x/9.4x at MC_EIG_ATT_D=1.0.

  freq vs spin    the one test that separates a plant mode from a loop mode. A
                  gyroscopic mode's frequency TRACKS the spin rate (slope ~0.83 for
                  this airframe); a control-loop resonance is flat. Reasoning from the
                  frequency's value alone is numerology - this reports the slope.

  airframe        SYS_AUTOSTART/CT/GYRO_CUTOFF, because a log from the wrong airframe
                  is not evidence about this vehicle and every retained log so far was.
"""
from __future__ import annotations

import argparse
import sys

import numpy as np

try:
    from pyulog import ULog
except ImportError:
    sys.exit("pyulog not found: pip install pyulog")


TOPICS = ["mc_controller_status", "vehicle_angular_velocity", "vehicle_torque_setpoint",
          "actuator_motors", "vehicle_local_position", "vehicle_local_position_setpoint"]

# What 10016_none_iris sets. A mismatch is reported, never corrected for.
AIRFRAME_EXPECT = {"SYS_AUTOSTART": 10016, "CA_ROTOR0_CT": 10.344, "IMU_GYRO_CUTOFF": 100.0}

# Noise floors. Every metric below is a ratio or a spectrum, and all three of them are
# meaningless once the signal reaches the estimator's noise floor - a healthy hover then
# trips the same warnings a departure does, which makes the whole report untrustworthy
# exactly where it needs to be trusted. These are the thresholds below which a quantity
# is reported but not flagged.
D_TERM_FLOOR = 0.05      # [rad/s] rate setpoint from the derivative path, ~1% of authority
CONING_FLOOR = 0.02      # [rad/s] rms |p,q| below which the spectrum is estimator noise
TREND_FRACTION = 0.20    # coning trend must project >20% of current rms over 60 s to matter


def _find_run_config(ulog_path: str) -> dict | None:
    """Locate run_config.json for a ulog. It sits at the rootfs root, the ulog under
    <rootfs>/log/<date>/<time>.ulg, so walk up a few levels rather than assume a depth."""
    import json
    import os

    directory = os.path.dirname(os.path.abspath(ulog_path))
    for _ in range(4):
        candidate = os.path.join(directory, "run_config.json")
        if os.path.isfile(candidate):
            try:
                with open(candidate) as handle:
                    return json.load(handle)
            except (OSError, ValueError):
                return None
        parent = os.path.dirname(directory)
        if parent == directory:
            break
        directory = parent
    return None


def _dominant_freq(p: np.ndarray, q: np.ndarray, dt: float) -> float:
    """Signed frequency of the strongest roll/pitch mode, in rad/s.

    Spectrum of the COMPLEX signal p + jq, not of |p,q|. A coning mode is a quadrature
    pair, so its magnitude envelope is rectified and peaks at 2*omega - taking the
    spectrum of hypot(p, q) reports double the real frequency. The complex form also
    makes the sign meaningful: positive is prograde (rotating with +r), negative
    retrograde, which is the first thing to check against the gyroscopic prediction.

    Detrended linearly, not just de-meaned: a coning amplitude that is growing or
    decaying puts most of its energy in the lowest bins and would otherwise win the
    argmax outright.
    """
    n = len(p)
    if n < 64 or not np.isfinite(dt) or dt <= 0:
        return float("nan")

    t = np.arange(n) * dt
    sig = (p - np.polyval(np.polyfit(t, p, 1), t)
           + 1j * (q - np.polyval(np.polyfit(t, q, 1), t)))
    if np.abs(sig).std() < 1e-9:
        return float("nan")

    freqs = np.fft.fftfreq(n, d=dt)
    amp = np.abs(np.fft.fft(sig))

    # Ignore the lowest few bins outright: residual drift and the window edges live
    # there and are not modes.
    floor = 3.0 / (n * dt)          # Hz
    usable = np.abs(freqs) > floor
    if not usable.any():
        return float("nan")

    amp = np.where(usable, amp, 0.0)
    return float(2.0 * np.pi * freqs[np.argmax(amp)])


def _freq_tracks_spin(p: np.ndarray, q: np.ndarray, r: np.ndarray, t: np.ndarray,
                      segments: int = 8) -> tuple[float, float]:
    """Regress the coning frequency against the spin rate over time slices.

    Returns (slope [rad/s per rad/s], correlation). Slope ~0.83 means the mode is the
    rigid-body gyroscopic coupling; slope ~0 with a strong limit cycle means it is the
    control loop. Needs the spin to actually vary across the window to say anything.
    """
    if len(t) < segments * 64:
        return float("nan"), float("nan"), float("nan")
    dt = float(np.median(np.diff(t)))
    edges = np.linspace(t[0], t[-1], segments + 1)
    freqs, spins = [], []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (t >= lo) & (t < hi)
        if m.sum() < 64:
            continue
        f = _dominant_freq(p[m], q[m], dt)
        if np.isfinite(f):
            freqs.append(abs(f))
            spins.append(float(np.abs(r[m]).mean()))
    if len(freqs) < 4 or np.std(spins) < 0.05:
        return float("nan"), float("nan"), float("nan")
    slope = float(np.polyfit(spins, freqs, 1)[0])
    corr = float(np.corrcoef(spins, freqs)[0, 1])
    # How much the frequency moves at all. A near-zero slope with a near-zero
    # correlation is the signature of a FIXED mode - but it is also what pure noise
    # looks like, so report the scatter and let the caller separate the two.
    cv = float(np.std(freqs) / max(np.mean(freqs), 1e-9))
    return slope, corr, cv


def analyse(path: str, window: float | None) -> None:
    ulog = ULog(path, TOPICS)
    data = {d.name: d for d in ulog.data_list}
    params = ulog.initial_parameters

    print(f"\n=== {path}")

    # --- airframe: is this log even about the right vehicle? ---------------
    problems = [f"{k}={params.get(k)} (want {v})"
                for k, v in AIRFRAME_EXPECT.items()
                if params.get(k) is None or abs(params[k] - v) > 1e-3]
    if problems:
        print("  AIRFRAME MISMATCH -- not the Pegasus Iris: " + ", ".join(problems))
        print("  Nothing below describes the intended vehicle.")
    else:
        print(f"  airframe OK (SYS_AUTOSTART={params['SYS_AUTOSTART']:.0f})")
    # The PARAMETER is not what flew. MulticopterController defers MC_CTRL_ALG to disarm,
    # so a mid-flight change leaves initial_parameters claiming a controller that never
    # ran; and a latched fallback hands the vehicle to the reference cascade while the
    # parameter still reads 3. mc_controller_status.algorithm is _active_alg - the
    # instance actually built - which is the only trustworthy answer.
    ran = fallback = None
    if "mc_controller_status" in data and "vehicle_local_position" in data:
        st = data["mc_controller_status"]
        # Only while AIRBORNE. Over the whole log this picks up every value the
        # parameter passed through on the ground, which is not what flew.
        lp0 = data["vehicle_local_position"]
        base = lp0.data["timestamp"][0]
        up = lp0.data["timestamp"][lp0.data["z"] < -1.0]
        ts_ms = st.data["timestamp"]
        m0 = ((ts_ms >= up[0]) & (ts_ms <= up[-1])) if len(up) > 10 else np.ones(len(ts_ms), bool)
        del base
        if m0.sum():
            ran = sorted({int(a) for a in st.data["algorithm"][m0]})
            fallback = int(max(st.data["fallback_active"][m0]))
    claimed = params.get("MC_CTRL_ALG")
    note = ""
    if ran is not None and (len(ran) > 1 or (claimed is not None and ran != [int(claimed)])):
        note = f"  <-- PARAM SAYS {claimed}, THIS IS WHAT RAN"
    if fallback:
        note += "  <-- FALLBACK LATCHED: the reference cascade flew, not this law"
    print(f"  algorithm(ran)={ran if ran is not None else '?'} "
          f"(param MC_CTRL_ALG={claimed}){note}")
    print(f"  MC_EIG_ATT_P={params.get('MC_EIG_ATT_P')} ATT_D={params.get('MC_EIG_ATT_D')} "
          f"WN={params.get('MC_EIG_WN')} B={params.get('MC_EIG_B')}")

    # The COMMANDED spin, from the simulator side. motor_moment is a Pegasus setting and
    # appears nowhere in the ulog, so without this the only way to label a spin-ladder run
    # is to infer km from the measured yaw rate - circular, and wrong whenever the
    # controller arrests some of the spin. Written by single_vehicle.py --keep-log.
    run_config = _find_run_config(path)
    if run_config:
        print(f"  commanded spin  motor_moment {run_config['motor_moment']:.2e} "
              f"-> expected r_ss {run_config['expected_spin_rad_s']:.1f} rad/s")
    else:
        print("  commanded spin  UNKNOWN - no run_config.json; motor_moment can only be "
              "inferred from the measured r below")

    if "vehicle_local_position" not in data:
        print("  no vehicle_local_position - cannot find the airborne segment")
        return

    lpos = data["vehicle_local_position"]
    t0 = lpos.data["timestamp"][0] * 1e-6

    def times(name):
        return data[name].data["timestamp"] * 1e-6 - t0

    tl = times("vehicle_local_position")
    z = lpos.data["z"]

    # Airborne = below -1 m. Cruise trims the first 2 s of climb, then optionally the
    # trailing `window` seconds, so a departure at the end is not averaged away by a
    # healthy start.
    airborne = z < -1.0
    if airborne.sum() < 10:
        print("  never got airborne (z never below -1 m)")
        return
    t_start, t_end = tl[airborne][0] + 2.0, tl[airborne][-1]
    if window is not None:
        t_start = max(t_start, t_end - window)
    if t_end - t_start < 1.0:
        print(f"  airborne only {tl[airborne][-1] - tl[airborne][0]:.1f} s - too short")
        return
    print(f"  airborne {tl[airborne][0]:.1f}..{t_end:.1f} s; "
          f"analysing {t_start:.1f}..{t_end:.1f} s")

    def cruise(name):
        t = times(name)
        return t, (t >= t_start) & (t <= t_end)

    # --- loop rates --------------------------------------------------------
    if "mc_controller_status" in data:
        st = data["mc_controller_status"]
        ts, cs = cruise("mc_controller_status")
        count = st.data["update_count"].astype(np.float64)
        valid = cs & (count > 0)
        rate = np.polyfit(ts[valid], count[valid], 1)[0] if valid.sum() > 10 else float("nan")
        med = lambda k: np.median(st.data[k][cs][st.data[k][cs] > 0]) * 1e3
        print(f"  loop rates      inner {rate:6.1f} Hz (dt {med('dt'):.3f} ms)  "
              f"attitude {med('dt_attitude'):.3f} ms  position {med('dt_position'):.3f} ms")
        if st.data["fallback_active"][cs].max():
            print("  FALLBACK LATCHED - the reference cascade flew, not the selected law")

        # D/P split of the angle stage. debug[0,1] = roll/pitch error,
        # debug[2,3] = the rate setpoint those errors produced.
        att_p = params.get("MC_EIG_ATT_P")
        if att_p and params.get("MC_CTRL_ALG") == 3:
            for axis, i in (("roll", 0), ("pitch", 1)):
                err = st.data[f"debug[{i}]"][cs]
                rsp = st.data[f"debug[{i + 2}]"][cs]
                p_term = att_p * err
                d_term = rsp - p_term
                ratio = d_term.std() / max(p_term.std(), 1e-9)
                # A ratio on its own cries wolf on a healthy flight. When the attitude
                # error sits at the estimator noise floor (5e-4 rad), the proportional
                # term is ~zero and differentiating that noise over 4 ms gives D/P > 1
                # from a derivative that commands nothing. Only flag when the D path is
                # both dominant AND large enough to move the vehicle: at wn=24 and
                # I/TRQ_MAX ~ 0.009, 0.05 rad/s of rate setpoint is ~1% of authority.
                driving = ratio > 1.0 and d_term.std() > D_TERM_FLOOR
                flag = ("  <-- derivative is driving" if driving
                        else "  (both terms negligible)" if d_term.std() <= D_TERM_FLOOR
                        else "")
                print(f"  angle {axis:5s}    err {err.std():.4f} rad rms   "
                      f"D {d_term.std():.3f} rad/s   D/P {ratio:5.2f}{flag}")

    # --- rates, coning, and which mode it is -------------------------------
    if "vehicle_angular_velocity" in data:
        av = data["vehicle_angular_velocity"]
        ta, ca = cruise("vehicle_angular_velocity")
        p, q = av.data["xyz[0]"][ca], av.data["xyz[1]"][ca]
        r = av.data["xyz[2]"][ca]
        pq = np.hypot(p, q)
        t = ta[ca]
        # Trend verdict needs two things a bare slope sign does not give: that the slope
        # is distinguishable from zero, and that it is material over the window ACTUALLY
        # OBSERVED. Judging a 28 s sample by its extrapolation to a full minute inflates
        # it by 2x and re-fires the same false alarm the noise floors were added to stop.
        trend, intercept = np.polyfit(t, pq, 1)
        residual = pq - (trend * t + intercept)
        span = t[-1] - t[0]
        n = len(t)
        denom = np.sum((t - t.mean()) ** 2)
        stderr = (np.sqrt(np.sum(residual ** 2) / (n - 2) / denom)
                  if n > 2 and denom > 0 else np.inf)
        significant = abs(trend) > 3.0 * stderr
        material = abs(trend) * span > TREND_FRACTION * max(pq.std(), 1e-9)

        if span < 15.0:
            # "flat" is a claim; on a short sample the honest answer is that no claim
            # can be made. A sustained limit cycle and a slow divergence look identical
            # over a few seconds.
            verdict = f"window only {span:.0f}s - trend not judgeable"
        elif not (significant and material):
            verdict = f"flat over {span:.0f}s"
        else:
            verdict = "decaying" if trend < 0 else "GROWING"
        if pq.std() < CONING_FLOOR:
            verdict += ", sub-noise-floor"
        print(f"  spin r          {r.mean():6.2f} rad/s mean, {np.abs(r).max():.2f} peak")
        print(f"  coning |p,q|    {pq.std():.3f} rad/s rms, {pq.max():.3f} peak, "
              f"trend {trend * 60:+.3f} per min ({verdict})")

        # Below the noise floor there is no mode to identify: the spectrum of estimator
        # noise has a peak too, and reporting it as "the dominant mode" invites chasing it.
        if pq.std() < CONING_FLOOR:
            print(f"  dominant mode   none - coning below the {CONING_FLOOR} rad/s "
                  f"noise floor, nothing to identify")
            freq = float("nan")
            slope = corr = cv = float("nan")
        else:
            freq = _dominant_freq(p, q, float(np.median(np.diff(t))))
            slope, corr, cv = _freq_tracks_spin(p, q, r, t)

        if np.isfinite(freq):
            sense = "prograde" if freq > 0 else "retrograde"
            line = (f"  dominant mode   {abs(freq):.1f} rad/s "
                    f"({abs(freq) / (2 * np.pi):.2f} Hz, {sense})")
            if np.isfinite(slope):
                # Tracking evidence is checked FIRST. A mode can move with spin and
                # still have modest scatter, so letting the fixed-frequency test run
                # first would overrule the stronger signal with the weaker one.
                if slope > 0.4 and corr > 0.5:
                    kind = "plant/gyroscopic (tracks spin)"
                elif cv < 0.15 and abs(corr) < 0.5:
                    kind = "loop resonance (frequency fixed)"
                else:
                    kind = "ambiguous"
                line += (f"; vs spin slope {slope:+.2f} corr {corr:+.2f} "
                         f"spread {cv:.2f} -> {kind}")
                # The label is a hint; the slope against the rigid-body prediction is
                # the actual evidence. Coning couples as ((Izz - Ixx) / Ixx) * r, so
                # that ratio is the slope a purely gyroscopic mode would show.
                ixx, izz = params.get("MC_EIG_IXX"), params.get("MC_EIG_IZZ")
                if ixx and izz and ixx > 0:
                    line += f"  [rigid-body predicts {(izz - ixx) / ixx:+.2f}]"
            else:
                line += "; spin too constant to classify (fly a spin ladder to resolve)"
            print(line)

    # --- authority ---------------------------------------------------------
    if "vehicle_torque_setpoint" in data:
        tq = data["vehicle_torque_setpoint"]
        _, ct = cruise("vehicle_torque_setpoint")
        for i, axis in enumerate("xyz"):
            v = tq.data[f"xyz[{i}]"][ct]
            print(f"  torque {axis}        {np.abs(v).std():.4f} rms, "
                  f"{np.abs(v).max():.4f} peak of 1.0")

    if "actuator_motors" in data:
        am = data["actuator_motors"]
        _, cm = cruise("actuator_motors")
        motors = np.vstack([am.data[f"control[{i}]"][cm] for i in range(4)])
        lo, hi = motors.min(), motors.max()
        rails = (motors <= 1e-3).mean() + (motors >= 1.0 - 1e-3).mean()
        flag = "  <-- SATURATING" if rails > 0.01 else ""
        print(f"  motors          {lo:.3f}..{hi:.3f}, {100 * rails:.1f}% on a rail{flag}")

    # --- position hold -----------------------------------------------------
    # Error against the setpoint, not raw spread. A commanded step reads as a large
    # "rms" on a vehicle that is in fact tracking it perfectly - a 0.35 m altitude step
    # in an otherwise flat hover shows up as 16 cm of apparent altitude error.
    tl_all, cl = cruise("vehicle_local_position")
    axes = {k: lpos.data[k][cl] for k in ("x", "y", "z")}
    label = "position hold"

    print(f"  {label}   " + ", ".join(
        f"{k} {v.std() * 100:6.1f} cm rms" for k, v in axes.items()))

    # Altitude travelled over the window. A run that climbs 7 m inside the analysis
    # window reports metres of "position hold" while tracking perfectly - the same trap
    # as the setpoint step, and the ladder runs all climb after takeoff.
    alt = -lpos.data["z"][cl]
    if alt.max() - alt.min() > 0.5:
        print(f"  altitude        {alt[0]:.1f} -> {alt[-1]:.1f} m "
              f"(span {alt.max() - alt.min():.1f} m)  <-- window includes a climb")

    # How far the SETPOINT itself moved over the window. Without this, a commanded step
    # is indistinguishable from tracking error: a 0.35 m altitude step in an otherwise
    # flat hover reads as 16 cm of "position hold", which looks like a control problem
    # and is not one. Reported separately rather than subtracted, because this topic is
    # NaN outside a position mode (~75% of a typical run) and interpolating across those
    # gaps would invent a setpoint that was never commanded.
    if "vehicle_local_position_setpoint" in data:
        sp = data["vehicle_local_position_setpoint"]
        tsp = times("vehicle_local_position_setpoint")
        in_window = (tsp >= t_start) & (tsp <= t_end)
        spans = []
        for k in ("x", "y", "z"):
            v = sp.data[k][in_window]
            v = v[np.isfinite(v)]
            if len(v):
                spans.append(f"{k} {(v.max() - v.min()) * 100:.0f} cm")
        if spans:
            moved = any(not s.endswith(" 0 cm") for s in spans)
            note = "  <-- setpoint moved; spread above is not all error" if moved else ""
            print(f"  setpoint range  " + ", ".join(spans) + note)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ulogs", nargs="+")
    ap.add_argument("--window", type=float, default=None,
                    help="Analyse only the last N seconds of the airborne segment. "
                         "Use it when a run departs late and the healthy start would "
                         "otherwise average the departure away.")
    args = ap.parse_args()
    for path in args.ulogs:
        try:
            analyse(path, args.window)
        except Exception as exc:  # one bad log must not stop a batch
            print(f"\n=== {path}\n  FAILED: {exc}")


if __name__ == "__main__":
    main()
