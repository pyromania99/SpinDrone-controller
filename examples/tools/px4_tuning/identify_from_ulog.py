#!/usr/bin/env python3
"""
identify_from_ulog.py — read the real airframe's effectiveness and inertias out of a ulog.

Companion to derive_px4_params.py, which answers "what should PX4 hold if Pegasus is the
truth". This answers "what does the actual vehicle do", from a flight log plus two
tape-measure numbers (takeoff mass, centre-to-motor distance).

The one thing a ulog cannot give you
------------------------------------
MC_EIG_TRQ_MAX is a pure function of CA_ROTOR*_CT/KM/PX/PY. A log only echoes whatever was
typed into those, so if they are wrong the log repeats the wrong number. What a log DOES
give, with no airframe knowledge at all, is the ratio

    c_axis  =  d(omega)/dt / tau_normalised        [rad/s^2 per unit command]

That is the eigen law's NDI loop gain: the controller multiplies by I and divides by
TRQ_MAX, so an error in one cancels an error in the other. Carry the ratio.

The subtlety this script exists to handle
-----------------------------------------
The firmware normalises the mixer using the CONFIGURED geometry, then that mixer drives the
REAL vehicle. So the physical torque produced by one unit of command is

    K = diag( B_real @ mix(B_configured) )

and NOT 1/scale(B_real), which is what you would get by running derive_px4_params.py on
corrected parameters. The two differ by exactly the factor the parameters are wrong by.
I = K / c uses the first; the TRQ_MAX you should then flash uses the second.

We do not assume X or plus. The logged CA_ROTOR*_PX/PY pattern is taken as correct in
DIRECTION and wrong only in SCALE, rescaled so the furthest rotor sits at --arm. That
assumption is then tested: a mixer whose layout disagrees with the frame produces large
roll/pitch cross-coupling, and the script prints predicted against measured.

Usage
-----
  python3 identify_from_ulog.py ~/Downloads/logs/ev_delay.ulg --mass 0.500 --arm 0.50
"""
from __future__ import annotations

import argparse

import numpy as np
from pyulog import ULog

GRAVITY = 9.81
TOPICS = ["vehicle_torque_setpoint", "vehicle_angular_velocity", "actuator_motors",
          "vehicle_local_position", "vehicle_land_detected"]
AXES = ("roll", "pitch", "yaw")


def build_effectiveness(ct, km, px, py):
    """PX4 ActuatorEffectivenessRotors with axis = (0,0,-1) FRD.

    moment = ct*(position x axis) - ct*km*axis  ->  (-ct*py, ct*px, ct*km); thrust -ct.
    """
    return np.vstack([-ct * py, ct * px, ct * km, -ct])


def px4_mix(B):
    """pinv(B) with PX4's normalisation. Replicates, line for line,
    ControlAllocationPseudoInverse::updateControlAllocationMatrixScale() +
    normalizeControlAllocationMatrix(): roll and pitch share the LARGER of the two scales.
    Returns the normalised mix and that shared scale (1/scale is TRQ_MAX).
    """
    mix = np.linalg.pinv(B)
    scales = []
    for axis in (0, 1):
        n = int((np.abs(mix[:, axis]) > 1e-3).sum())
        scales.append(np.sqrt((mix[:, axis] ** 2).sum() / (n / 2.0)) if n else 1.0)
    scale = max(scales)
    out = mix.copy()
    out[:, 0] /= scale
    out[:, 1] /= scale
    return out, scale


def configured_geometry(params):
    n = int(params.get("CA_ROTOR_COUNT", 4))
    g = lambda k, i: float(params.get(f"CA_ROTOR{i}_{k}", 0.0))
    return (np.array([g("CT", i) for i in range(n)]),
            np.array([g("KM", i) for i in range(n)]),
            np.array([g("PX", i) for i in range(n)]),
            np.array([g("PY", i) for i in range(n)]))


def airborne_window(data):
    ld = data["vehicle_land_detected"]
    flying = ld["timestamp"][ld["landed"] == 0] / 1e6
    if len(flying) < 2:
        raise SystemExit("log never leaves the ground")
    return flying.min() + 1.0, flying.max() - 1.0


def effectiveness_from_data(data, t0, t1, lags):
    """Fit the full 3x3 d(omega)/dt = M @ tau_norm at the lag that maximises total R^2.

    The 3x3, not three separate 1x1 fits: the off-diagonals are the test of whether the
    mixer's layout matches the frame, and they are free to compute.
    """
    av, tq = data["vehicle_angular_velocity"], data["vehicle_torque_setpoint"]
    t_av, t_tq = av["timestamp"] / 1e6, tq["timestamp"] / 1e6
    sel = (t_av > t0) & (t_av < t1)
    t = t_av[sel]
    wdot = [av[f"xyz_derivative[{i}]"][sel] for i in range(3)]

    best = None
    for lag in lags:
        tau = [np.interp(t - lag, t_tq, tq[f"xyz[{i}]"]) for i in range(3)]
        A = np.vstack(tau + [np.ones_like(t)]).T
        M = np.zeros((3, 3))
        r2 = np.zeros(3)
        for i in range(3):
            coef, *_ = np.linalg.lstsq(A, wdot[i], rcond=None)
            M[i] = coef[:3]
            r2[i] = 1 - ((wdot[i] - A @ coef) ** 2).sum() / ((wdot[i] - wdot[i].mean()) ** 2).sum()
        if best is None or r2[:2].sum() > best["r2"][:2].sum():
            best = {"lag": lag, "M": M, "r2": r2}

    # Split-half stability of the diagonal: the honest error bar. The formal standard error
    # is meaningless here because the residuals are autocorrelated and the data is closed
    # loop, so a slope that moves between halves of one flight is the real uncertainty.
    half = t < 0.5 * (t[0] + t[-1])
    spread = np.zeros(3)
    for i in range(3):
        vals = []
        for mask in (half, ~half):
            tau = [np.interp(t[mask] - best["lag"], t_tq, tq[f"xyz[{j}]"]) for j in range(3)]
            A = np.vstack(tau + [np.ones(mask.sum())]).T
            vals.append(np.linalg.lstsq(A, wdot[i][mask], rcond=None)[0][i])
        spread[i] = abs(vals[0] - vals[1]) / max(abs(0.5 * (vals[0] + vals[1])), 1e-9)
    best["spread"] = spread
    best["n"] = int(sel.sum())
    return best


def band_gain(data, t0, t1, band, fs=200.0, nper=2048):
    """|G| in a frequency band, from the cross-spectrum. A pure input delay rotates the
    phase of Pxy but cannot change |Pxy/Pxx|, so this is immune to the lag guess that the
    time-domain fit has to make. If the two disagree, the time-domain number is suspect.
    """
    av, tq = data["vehicle_angular_velocity"], data["vehicle_torque_setpoint"]
    t_av, t_tq = av["timestamp"] / 1e6, tq["timestamp"] / 1e6
    sel = (t_av > t0) & (t_av < t1)
    tu = np.arange(t_av[sel][0], t_av[sel][-1], 1 / fs)
    out = []
    for i in range(3):
        x = np.interp(tu, t_tq, tq[f"xyz[{i}]"])
        y = np.interp(tu, t_av[sel], av[f"xyz_derivative[{i}]"][sel])
        step, w = nper // 2, np.hanning(nper)
        Pxy = np.zeros(nper // 2 + 1, complex)
        Pxx = np.zeros(nper // 2 + 1)
        Pyy = np.zeros(nper // 2 + 1)
        k = 0
        for s in range(0, len(x) - nper + 1, step):
            X = np.fft.rfft((x[s:s + nper] - x[s:s + nper].mean()) * w)
            Y = np.fft.rfft((y[s:s + nper] - y[s:s + nper].mean()) * w)
            Pxy += np.conj(X) * Y
            Pxx += np.abs(X) ** 2
            Pyy += np.abs(Y) ** 2
            k += 1
        f = np.fft.rfftfreq(nper, 1 / fs)
        b = (f >= band[0]) & (f <= band[1])
        G = Pxy[b].sum() / Pxx[b].sum()
        coh = float((np.abs(Pxy[b]) ** 2 / (Pxx[b] * Pyy[b].real)).mean())
        out.append((abs(G), coh, np.degrees(np.angle(G))))
    return out


def hover_thrust_sum(data, t0, t1, n_rotors):
    am, lp = data["actuator_motors"], data["vehicle_local_position"]
    t = am["timestamp"] / 1e6
    az = np.interp(t, lp["timestamp"] / 1e6, lp["az"])
    vz = np.interp(t, lp["timestamp"] / 1e6, lp["vz"])
    total = np.vstack([am[f"control[{i}]"] for i in range(n_rotors)]).sum(axis=0)
    sel = (t > t0) & (t < t1) & (np.abs(az) < 0.7) & (np.abs(vz) < 0.3) & (total > 0.4)
    if sel.sum() < 100:
        raise SystemExit("not enough steady-hover samples to estimate CT")
    return total[sel].mean(), total[sel].std(), int(sel.sum())


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ulog")
    ap.add_argument("--mass", type=float, required=True, help="takeoff mass [kg]")
    ap.add_argument("--arm", type=float, required=True,
                    help="centre-to-motor distance [m] of the furthest rotor")
    ap.add_argument("--band", type=float, nargs=2, default=(0.5, 3.0),
                    help="frequency band [Hz] for the delay-free cross-check")
    args = ap.parse_args()

    ulog = ULog(args.ulog, TOPICS)
    data = {m.name: m.data for m in ulog.data_list}
    if missing := [t for t in TOPICS if t not in data]:
        raise SystemExit(f"log is missing {missing}")
    params = ulog.initial_parameters

    t0, t1 = airborne_window(data)
    av_t = data["vehicle_angular_velocity"]["timestamp"] / 1e6
    n_win = int(((av_t > t0) & (av_t < t1)).sum())
    rate = n_win / (t1 - t0)

    print(f"log       {args.ulog}")
    print(f"build     {ulog.msg_info_dict.get('ver_hw')} / {ulog.msg_info_dict.get('ver_sw_branch')}")
    print(f"airborne  {t0:.1f}-{t1:.1f} s, {n_win} gyro samples ({rate:.0f} Hz avg)"
          + ("" if rate > 200 else "   <-- TOO SLOW, xyz_derivative is aliased, stop here"))
    has_eig = any(k.startswith("MC_EIG_") for k in params)
    print(f"mc_controller params present: {'yes' if has_eig else 'NO (this was a stock PD flight)'}")

    # ---- 1. what the firmware was actually mixing with ----
    ct_c, km_c, px_c, py_c = configured_geometry(params)
    mix_cfg, scale_cfg = px4_mix(build_effectiveness(ct_c, km_c, px_c, py_c))
    radii = np.hypot(px_c, py_c)
    print(f"\n--- 1. configured mixer (from the log's own CA_ROTOR*) ---")
    print(f"  {len(ct_c)} rotors, CT {ct_c[0]:.2f}, radii {np.round(radii,3)}")
    print(f"  1/scale = {1/scale_cfg:.4f}  <-- the TRQ_MAX these parameters imply; believe it "
          f"only if\n            the radii above are real metres, which they usually are not")

    # ---- 2. what the vehicle actually did ----
    fit = effectiveness_from_data(data, t0, t1, np.arange(0.0, 0.121, 0.005))
    M = fit["M"]
    print(f"\n--- 2. measured effectiveness, d(omega)/dt = M @ tau  [rad/s^2 per unit] ---")
    print(f"  best input lag {fit['lag']*1000:.0f} ms (actuator + gyro filter), n = {fit['n']}")
    for i, nm in enumerate(AXES):
        flag = "" if fit["r2"][i] > 0.3 else "   <-- NOT IDENTIFIED, axis not excited"
        print(f"  {nm:<6} diag {M[i,i]:8.1f}   R2 {fit['r2'][i]:.3f}   "
              f"split-half {fit['spread'][i]*100:3.0f}%{flag}")
    print(f"  cross-coupling: roll<-pitch {100*abs(M[0,1]/M[0,0]):.1f}%, "
          f"pitch<-roll {100*abs(M[1,0]/M[1,1]):.1f}%")

    # ---- 3. delay-free cross-check ----
    bg = band_gain(data, t0, t1, tuple(args.band))
    print(f"\n--- 3. cross-check: band gain over {args.band[0]}-{args.band[1]} Hz (immune to the lag guess) ---")
    for i, nm in enumerate(AXES[:2]):
        print(f"  {nm:<6} |G| {bg[i][0]:8.1f}   coherence {bg[i][1]:.2f}   phase {bg[i][2]:6.1f} deg")
    td = M[0, 0] / M[1, 1]
    fd = bg[0][0] / bg[1][0]
    print(f"  roll/pitch gain ratio:  time domain {td:.3f}   frequency domain {fd:.3f}")
    if abs(td - fd) > 0.05:
        print("  ** they disagree: the roll/pitch asymmetry is not well determined by this log.")

    # ---- 4. absolute scale ----
    mean, std, n = hover_thrust_sum(data, t0, t1, len(ct_c))
    ct_real = args.mass * GRAVITY / mean
    unit = radii / radii.max()
    px_r, py_r = px_c / radii.max() * args.arm, py_c / radii.max() * args.arm
    B_real = build_effectiveness(np.full(len(ct_c), ct_real), km_c, px_r, py_r)
    realised = B_real @ mix_cfg
    _, scale_new = px4_mix(B_real)
    print(f"\n--- 4. absolute scale (needs mass and arm) ---")
    print(f"  hover sum of motor commands   {mean:.3f} +- {std:.3f}  ({n} samples)")
    print(f"  CA_ROTOR*_CT                  {ct_real:.3f} N   (thrust/weight {len(ct_c)*ct_real/(args.mass*GRAVITY):.2f})")
    print(f"  predicted cross-coupling from this geometry: "
          f"roll<-pitch {100*abs(realised[0,1]/realised[0,0]):.1f}%, "
          f"pitch<-roll {100*abs(realised[1,0]/realised[1,1]):.1f}%")
    print(f"     compare with the measured {100*abs(M[0,1]/M[0,0]):.1f}% / {100*abs(M[1,0]/M[1,1]):.1f}% above — "
          f"a mismatch means the\n     LAYOUT in CA_ROTOR*_PX/PY is wrong, not just its scale.")
    print(f"  MC_EIG_TRQ_MAX to flash       {1/scale_new:.4f}   (once CA_ROTOR*_PX/PY are corrected)")

    # ---- 5. inertias ----
    print(f"\n--- 5. inertias:  I = (torque the CONFIGURED mixer really produced) / c ---")
    for i, (nm, param) in enumerate(zip(AXES, ("MC_EIG_IXX", "MC_EIG_IYY", "MC_EIG_IZZ"))):
        if fit["r2"][i] < 0.3:
            print(f"  {param}   --   unmeasured, axis not excited in this log")
        else:
            print(f"  {param}   {realised[i,i]/M[i,i]:.5f} kg.m^2   "
                  f"(realised {realised[i,i]:.4f} N.m per unit / c {M[i,i]:.1f})")
    if fit["r2"][0] > 0.3:
        Ixx = realised[0, 0] / M[0, 0]
        print(f"\n  sanity check: that Ixx is an equivalent {Ixx/(2*args.arm**2)*1000:.0f} g per motor "
              f"at R = {args.arm:.3f} m.\n  Both X and plus give I = 2*mu*R^2, so this holds either way. "
              "If it is not roughly\n  your motor+prop+ESC mass, one of mass, arm or layout is wrong.")


if __name__ == "__main__":
    main()
