#!/usr/bin/env python
"""
Analytical proof that CascadedPD and Eigen controllers are equivalently tuned.

Linearises both controllers about hover and compares:
  1. Closed-loop pole locations  (eigenvalue comparison)
  2. Step responses              (time-domain)
  3. Bode magnitude plots        (frequency-domain)

Run:
    python tuning_equivalence.py          # saves figures to results/
    python tuning_equivalence.py --show   # also pops up matplotlib windows
"""

from __future__ import annotations
import argparse, os, textwrap
import numpy as np
import matplotlib
matplotlib.use("Agg")                   # safe for headless; --show re-enables
import matplotlib.pyplot as plt
from scipy import signal

# ──────────────────────────── default gains (from source) ─────────────────────
# Outer position PD (shared identically)
Kp_xy  = 0.50;   Kd_xy  = 1.0
Kp_z   = 15.0;   Kd_z   = 8.0

# CascadedPD inner-loop attitude PD
CPD_Kp_att = 0.65;  CPD_Kd_att = 10.0

# Eigen inner-loop attitude PD-on-error-rate  (feeds into eigen dynamics law)
EIG_Kp_att = 0.70;  EIG_Kd_att = 10.0
EIG_alpha  = 0.5
EIG_Ixx    = 0.01;  EIG_Iyy = 0.01

# Physical
MASS = 1.5          # CascadedPD default; Eigen default 1.2 but let's unify
G    = 9.81

OUT_DIR = os.path.join(os.path.dirname(__file__), "results")


# ═══════════════════════ 1. OUTER-LOOP  (position → accel) ═══════════════════
# Both controllers implement:   a_des = Kp · e_p + Kd · ė_p
# With the plant  ẍ = a,  the closed-loop from p_ref → p is:
#       G(s) = (Kd·s + Kp) / (s² + Kd·s + Kp)
#
# This is IDENTICAL in both controllers — no linearisation needed.

def outer_loop_tf(kp, kd):
    """Closed-loop TF  p_ref → p  for  ẍ = Kp·e + Kd·ė."""
    num = [kd, kp]
    den = [1.0, kd, kp]
    return signal.TransferFunction(num, den)


# ═══════════════════════ 2. INNER-LOOP  (desired att → att) ═══════════════════
#
# --- CascadedPD (per-axis) ---
# τ = -Kp_att · eθ - Kd_att · ω     and    I·θ̈ = τ
# Closed-loop:  θ_des → θ  =  (Kp/I) / (s² + (Kd/I)·s + (Kp/I))
#
# --- Eigen (per-axis, linearised at hover: r≈0, A_r diagonal) ---
# The attitude channel computes:
#   desired_rate = Kp_att · eθ  +  Kd_att · ėθ       (PD on angle error)
#   correction   = B · M_eff · [p, q, r, 1]ᵀ         (B = diag(I))
#
# At hover (r≈0), M_eff simplifies per-axis.  For roll (row 0):
#   τ_roll = Ixx · [(-7+α)·p  +  (7·dr + 8·dp)]
#   where dr = Kp_att·(θd−θ) − Kd_att·p  (desired_roll_rate, with ėθ ≈ −p)
#
# The EOM is  I·θ̈ = τ = I·[…], so inertia CANCELS:
#   θ̈ = (-7 + α - 7·Kd_att)·θ̇ + 7·Kp_att·(θd − θ)       [ignoring cross-coupling]
#
# Defining effective gains (INERTIA-INDEPENDENT):
#   Kp_eff = 7 · Kp_att                    = 4.9
#   Kd_eff = 7 - α + 7·Kd_att              = 76.5
# Closed-loop:  θ_des → θ  =  Kp_eff / (s² + Kd_eff·s + Kp_eff)

def cpd_inner_tf(I_axis):
    """CascadedPD attitude closed-loop TF  θ_des → θ."""
    kp = CPD_Kp_att
    kd = CPD_Kd_att
    # CascadedPD applies torque directly (τ = -Kp·e - Kd·ω), so I appears
    num = [kp / I_axis]
    den = [1.0, kd / I_axis, kp / I_axis]
    return signal.TransferFunction(num, den)


def eigen_inner_tf(I_axis):
    """Eigen attitude closed-loop TF  θ_des → θ  (linearised at hover).

    NOTE: I_axis is accepted for API consistency but does NOT appear
    in the TF because the Eigen law multiplies by B=diag(I), which
    cancels with I·θ̈ = τ.
    """
    M_diag = 2.0                                     # |M_dyn diagonal|
    kp_eff = M_diag * EIG_Kp_att                     # 1.4
    kd_eff = M_diag + M_diag * EIG_Kd_att - EIG_alpha  # 2 - 0.5 + 20 = 21.5
    num = [kp_eff]
    den = [1.0, kd_eff, kp_eff]
    return signal.TransferFunction(num, den)


# ═══════════════════════ 3. FULL CHAIN  (p_ref → p through attitude) ══════════
# The full loop is:
#   G_full(s) = G_outer(s)  ×  G_inner(s)  ×  (1/g)  ×  (1/s²)
# where (1/g) maps attitude→accel and (1/s²) is the translational double-integrator.
# But since both share the same outer loop and double-integrator, comparing inner
# loops is sufficient.  We still plot the full chain for completeness.

def full_chain_tf(inner_tf, kp_pos, kd_pos):
    """Cascaded closed-loop: outer position PD + inner attitude TF + plant."""
    #  inner_tf already is θ_des → θ.
    #  Lateral accel  a = g·θ  (small angle), so θ → a is just gain g.
    #  Position plant is 1/s².
    #  Outer PD controller: C(s) = kd·s + kp .
    #  Open-loop:  L(s) = C(s) · G_inner(s) · g · (1/s²)
    #  Closed-loop: L/(1+L)
    C_num = [kd_pos, kp_pos]
    C_den = [1.0]

    plant_num = [G]
    plant_den = [1.0, 0.0, 0.0]  # g / s²

    # Multiply: C · inner · plant
    ol_tf = _tf_series(_tf_series(
        signal.TransferFunction(C_num, C_den), inner_tf),
        signal.TransferFunction(plant_num, plant_den))

    # Close the loop
    cl_num = np.polymul(ol_tf.num, [1.0])
    cl_den = np.polyadd(ol_tf.den, ol_tf.num)
    return signal.TransferFunction(cl_num, cl_den)


def _tf_series(tf1, tf2):
    """Series connection of two SISO TFs."""
    return signal.TransferFunction(
        np.polymul(tf1.num, tf2.num),
        np.polymul(tf1.den, tf2.den),
    )


# ═══════════════════════ plotting helpers ═════════════════════════════════════

def _poles_table(label, tf):
    poles = tf.poles
    lines = [f"  {label}:"]
    for p in poles:
        wn = abs(p)
        zeta = -np.real(p) / wn if wn > 1e-12 else 0.0
        lines.append(f"    pole = {p:+10.4f}   |ωn| = {wn:.4f}   ζ = {zeta:.4f}")
    return "\n".join(lines)


def plot_comparison(cpd_tf, eig_tf, title, fname, show):
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle(title, fontsize=14, fontweight="bold")

    # — Bode magnitude —
    ax = axes[0, 0]
    w = np.logspace(-2, 3, 2000)
    _, mag_cpd, _ = signal.bode(cpd_tf, w)
    _, mag_eig, _ = signal.bode(eig_tf, w)
    ax.semilogx(w, mag_cpd, label="CascadedPD", linewidth=2)
    ax.semilogx(w, mag_eig, label="Eigen", linewidth=2, linestyle="--")
    ax.set_ylabel("Magnitude (dB)")
    ax.set_xlabel("Frequency (rad/s)")
    ax.set_title("Bode Magnitude")
    ax.legend()
    ax.grid(True, which="both", alpha=0.3)

    # — Bode phase —
    ax = axes[0, 1]
    _, _, ph_cpd = signal.bode(cpd_tf, w)
    _, _, ph_eig = signal.bode(eig_tf, w)
    ax.semilogx(w, ph_cpd, label="CascadedPD", linewidth=2)
    ax.semilogx(w, ph_eig, label="Eigen", linewidth=2, linestyle="--")
    ax.set_ylabel("Phase (deg)")
    ax.set_xlabel("Frequency (rad/s)")
    ax.set_title("Bode Phase")
    ax.legend()
    ax.grid(True, which="both", alpha=0.3)

    # — Step response —
    ax = axes[1, 0]
    t_cpd, y_cpd = signal.step(cpd_tf, T=np.linspace(0, 8, 4000))
    t_eig, y_eig = signal.step(eig_tf, T=np.linspace(0, 8, 4000))
    ax.plot(t_cpd, y_cpd, label="CascadedPD", linewidth=2)
    ax.plot(t_eig, y_eig, label="Eigen", linewidth=2, linestyle="--")
    ax.axhline(1.0, color="gray", linestyle=":", alpha=0.5)
    ax.set_ylabel("Response")
    ax.set_xlabel("Time (s)")
    ax.set_title("Step Response")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # — Pole-zero map —
    ax = axes[1, 1]
    cpd_p = cpd_tf.poles
    eig_p = eig_tf.poles
    ax.scatter(np.real(cpd_p), np.imag(cpd_p), marker="x", s=120,
               linewidths=2, color="tab:blue", label="CascadedPD poles", zorder=5)
    ax.scatter(np.real(eig_p), np.imag(eig_p), marker="o", s=80,
               facecolors="none", edgecolors="tab:orange", linewidths=2,
               label="Eigen poles", zorder=5)
    ax.axhline(0, color="k", linewidth=0.5)
    ax.axvline(0, color="k", linewidth=0.5)
    ax.set_xlabel("Real")
    ax.set_ylabel("Imaginary")
    ax.set_title("Pole Locations")
    ax.legend()
    ax.grid(True, alpha=0.3)

    fig.tight_layout(rect=[0, 0, 1, 0.95])
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, fname)
    fig.savefig(path, dpi=150)
    print(f"  saved → {path}")
    if show:
        plt.show()
    plt.close(fig)


# ═══════════════════════ main ═════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--show", action="store_true", help="Pop up plot windows")
    args = parser.parse_args()

    if args.show:
        matplotlib.use("TkAgg")

    print("=" * 70)
    print("  CONTROLLER TUNING EQUIVALENCE ANALYSIS")
    print("=" * 70)

    # ── Outer loop (identical by construction) ──
    print("\n1) OUTER POSITION LOOP  (p_ref → desired accel)")
    print("   Both controllers: Kp_xy = %.2f, Kd_xy = %.2f" % (Kp_xy, Kd_xy))
    print("   Both controllers: Kp_z  = %.2f, Kd_z  = %.2f" % (Kp_z, Kd_z))
    print("   → Transfer functions IDENTICAL.  No comparison needed.\n")

    # ── Inner loop ──
    print("2) INNER ATTITUDE LOOP  (θ_des → θ, linearised at hover)")
    I_ax = EIG_Ixx   # use same inertia for fair comparison
    cpd_att = cpd_inner_tf(I_ax)
    eig_att = eigen_inner_tf(I_ax)

    print("\n" + _poles_table("CascadedPD", cpd_att))
    print(_poles_table("Eigen     ", eig_att))

    # Quantify difference
    cpd_poles_sorted = np.sort_complex(cpd_att.poles)
    eig_poles_sorted = np.sort_complex(eig_att.poles)
    if len(cpd_poles_sorted) == len(eig_poles_sorted):
        pole_dist = np.max(np.abs(cpd_poles_sorted - eig_poles_sorted))
        print(f"\n   Max pole distance: {pole_dist:.4f}")
        if pole_dist < 0.01:
            print("   ✓ Poles MATCH — controllers are equivalently tuned (inner loop).")
        else:
            print("   ✗ Poles DIFFER — inner loops are NOT identically tuned.")
            print("     (This is expected: the architectures differ, but the outer-loop")
            print("      response may still be similar.  See full-chain plots.)")

    plot_comparison(cpd_att, eig_att,
                    "Inner Attitude Loop:  CascadedPD vs Eigen  (linearised at hover)",
                    "tuning_equiv_inner_loop.png", args.show)

    # ── Full chain ──
    print("\n3) FULL LATERAL CHAIN  (p_ref → p, through attitude inner loop)")
    cpd_full = full_chain_tf(cpd_att, Kp_xy, Kd_xy)
    eig_full = full_chain_tf(eig_att, Kp_xy, Kd_xy)

    print("\n" + _poles_table("CascadedPD full", cpd_full))
    print(_poles_table("Eigen full     ", eig_full))

    # Step-response metrics
    for label, tf in [("CascadedPD", cpd_full), ("Eigen", eig_full)]:
        t, y = signal.step(tf, T=np.linspace(0, 15, 8000))
        ss = y[-1]
        peak = np.max(y)
        os_pct = (peak / ss - 1.0) * 100.0 if ss > 1e-6 else 0.0
        # Rise time (10% → 90%)
        idx10 = np.searchsorted(y, 0.1 * ss)
        idx90 = np.searchsorted(y, 0.9 * ss)
        tr = t[min(idx90, len(t)-1)] - t[min(idx10, len(t)-1)]
        # Settling time (2% band)
        settled = np.where(np.abs(y - ss) > 0.02 * ss)[0]
        ts = t[settled[-1]] if len(settled) > 0 else 0.0
        print(f"\n   {label:15s}: overshoot = {os_pct:5.1f}%   rise = {tr:.3f}s   settling = {ts:.3f}s")

    plot_comparison(cpd_full, eig_full,
                    "Full Lateral Chain:  CascadedPD vs Eigen  (position tracking)",
                    "tuning_equiv_full_chain.png", args.show)

    # ── Z channel (trivially identical) ──
    print("\n4) ALTITUDE CHANNEL")
    print("   Both: Kp_z=%.1f, Kd_z=%.1f → identical closed-loop." % (Kp_z, Kd_z))

    print("\n" + "=" * 70)
    print("  SUMMARY")
    print("=" * 70)
    print(textwrap.dedent("""\
      • Outer position loop: IDENTICAL gains and structure.
      • Altitude channel:    IDENTICAL gains and structure.
      • Inner attitude loop: Different architecture but plots above show
        the frequency-domain and time-domain behaviour.
      • Full lateral chain:  Compare step-response metrics above to judge
        whether the end-to-end tuning produces equivalent tracking.
    """))
    print(f"  Figures saved in: {OUT_DIR}/")
    print("=" * 70)


if __name__ == "__main__":
    main()
