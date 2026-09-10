Frequency-Domain Analysis of Actuator Lag in the Spinning-Quadrotor Loop

Scope. This is meant to be a self-contained read: an input-agnostic
statement of the two physical blocks, specialization to the steady-spin
command $u=Ve^{-irt}$, a comparison of two actuator models (first-order lag
vs. pure transport delay — the model your papers actually commit to),
substitution of $V$ by the real feedback law with full Laplace bookkeeping,
the compensator machinery ($G^\star$, $\Theta_\phi$, $\Theta_{\text{ideal}}$,
$\Theta_{\text{practical}}$, $b^\star$), a cleaner $\rho$–$\theta$
reparametrization of $M_{\text{dyn}}$ that separates magnitude from phase and
extends naturally to the gyroscopic term, a block statement of the relevant
lemmas followed by an explicit stress test of each, a summary table of every
lag mechanism and its fix, and a glossary of every symbol used.

Notation: roll–pitch pairs are packed as complex scalars $\zeta=p+iq$ or
$u=u_x+iu_y$ (two simultaneous real channels relabeled as one complex number
— never a discard-the-imaginary-part device); $S=\begin{pmatrix}0&-1\1&0\end{pmatrix}$
implements multiplication by $i$ on the corresponding real 2-vector.

Glossary

Placed first, for reference while reading — every symbol used below, in one
place.

Symbol Meaning Units / type





$r$

Vehicle spin rate (measured, frozen per A1)

rad/s

$T$

Actuator time constant, first-order lag model

s

$\tau_a$

Actuator transport delay, committed model (A2)

s

$rT$, $r\tau_a$

Dimensionless spin-rate$\times$lag product; governs droop/phase (lag model) or rotation only (delay model)

dimensionless

$s$

Laplace variable (complex frequency)

rad/s (complex)

$G_a(s)$

Actuator transfer function

—

$G_p(s)$

Rigid-body (plant) transfer function, roll–pitch channel

—

$\omega_\perp=p+iq$

Complex-packed roll/pitch angular rate

rad/s

$p,,q$

Roll rate, pitch rate

rad/s

$I_\perp,,I_z$

Roll/pitch (transverse) and yaw (spin-axis) principal inertias

kg·m²

$\lambda=(I_z-I_\perp)/I_\perp$

Inertia ratio; sets the gyroscopic precession rate $\lambda r$

dimensionless

$a$

Aerodynamic damping coefficient (open-loop plant, negative); also the real part of the desired closed-loop pole in $M_{\text{dyn}}=aI+bS$ — same letter, two related-but-distinct roles, disambiguated by context in each Part

1/s

$b$

Off-diagonal entry of $M_{\text{dyn}}$; the free design knob that rotates the closed-loop pole

1/s

$b^\star$

Value of $b$ that exactly cancels the delay's rotation $r\tau_a$ on the tracking-error dynamics

1/s

$\theta=\arctan(b/a)$

Angle of the closed-loop pole $a+ib$ in the complex plane

rad

$\psi=-r\tau_a$

Rotation imposed by the delay on the tracking-error dynamics

rad

$\alpha=\theta+\psi$

Measured misalignment driving the online adaptation law

rad

$\rho=\sqrt{a^2+b^2}=

a+ib

$

$R(\theta)$

Standard $2\times2$ rotation matrix by angle $\theta$

—

$M_{\text{dyn}}$, $M_{rp}$

Desired closed-loop error-dynamics matrix for the roll–pitch block

1/s (matrix)

$S$

Skew matrix $\begin{pmatrix}0&-1\1&0\end{pmatrix}$; real-matrix equivalent of multiplying by $i$

—

$\tau=\tau_x+i\tau_y$

Complex-packed commanded/actual roll–pitch torque

N·m

$\tau_{\text{desired}}$

Torque command that would achieve $\dot x=M_{\text{dyn}}x$ under a perfect actuator

N·m

$x=(p,q)^\top$

Real roll/pitch rate state vector

rad/s

$A(r)=\lambda r S$

Plant's gyroscopic coupling matrix, as it appears in $C(r)=B^{-1}(M-A(r))$

1/s (matrix)

$\Delta J$

Inertia (model) error; $|\Delta J|\le\delta_J$ (A5)

kg·m²

$G^\star$

Exact static feedforward compensator (lag model) cancelling both droop and phase at frequency $r$

—

$\Theta_\phi$

Phase-only compensator; the rotation part of $G^\star$ with magnitude stripped out

—

$\Theta_{\text{ideal}}(s)=1+sT$

Exact dynamic inverse of the lag model at every $s$; not physically realizable

—

$\Theta_{\text{practical}}(s)$

Realizable lead-compensator approximation of $\Theta_{\text{ideal}}$, with roll-off time $T_f$

—

$T_f$

Roll-off time constant of the practical lead compensator; trades cancellation bandwidth against noise amplification

s

$\omega$ (Part II)

Frequency of a translational/maneuvering command in the inertial frame (envelope's second axis, called $v$ elsewhere)

rad/s

$\omega_c$

Closed-loop attitude bandwidth (A3: $\omega_c\tau_a\ll1$)

rad/s

$D$

Bound on additive disturbance torque magnitude, $\sup_t

d_c(t)

$\varepsilon_T$

Lag asymmetry between roll and pitch actuator time constants

dimensionless

$\beta$

Yaw-channel passive damping rate ($\dot r=-\beta r$)

1/s

$d_{\text{yaw}}$

Disturbance bound on the (passive) yaw channel

N·m

$\phi_{\max}$

Design margin on the lag-compensation angle, kept below $\pi/2$ for realizability

rad

$\kappa=\dfrac{\phi_{\max}}{\pi/2}$

Fractional design margin used in the certified spin ceiling $r\le\kappa,\pi/(2\tau_a)$

dimensionless

Part I — The open-loop chain, agnostic to the input

Two physical blocks in series, no feedback:

$$
\hat u(t) ;\xrightarrow{\ G_a(s)\ \text{(actuator)}\ }; \tau(t) ;\xrightarrow{\ G_p(s)\ \text{(rigid body)}\ }; \omega_\perp(t).
$$

Actuator, general form. Left as an unspecified LTI operator $G_a(s)$ for
now — specialized in Part III.

Plant. From Euler's equations, near-symmetric roll/pitch inertia
$I_x\approx I_y\equiv I_\perp$, aerodynamic damping $a<0$, spin rate $r$
frozen (quasi-static, per A1):

$$
I_\perp\dot p = a,p+(I_\perp-I_z)qr+\tau_x,\qquad I_\perp\dot q = a,q+(I_z-I_\perp)rp+\tau_y.
$$

Packing $\omega_\perp=p+iq$, $\tau=\tau_x+i\tau_y$, $\lambda\equiv(I_z-I_\perp)/I_\perp$:

$$
\dot\omega_\perp = (a+i\lambda r),\omega_\perp + \frac{\tau}{I_\perp}
\quad\Longrightarrow\quad
G_p(s) = \frac{1/I_\perp}{s-a-i\lambda r}.
$$

Why the cascade is a product $G_a(s)G_p(s)$. Not an axiom — ordinary
substitution, licensed by one fact: the torque leaving the actuator and the
torque entering the plant equation are the same signal, so their Laplace
transforms are the same function $\mathcal T(s)$. From
$\mathcal T(s)=G_a(s)\hat U(s)$ and $\Omega_\perp(s)=G_p(s)\mathcal T(s)$:

$$
\Omega_\perp(s) = G_a(s),G_p(s),\hat U(s).
$$

This assumes no loading — the plant's motion does not feed back and
change the actuator's own dynamics, reasonable here since motor/ESC response
is set by its own electro-mechanics, not airframe inertia.

Response to an arbitrary input — the residue idea. For any $\hat u(t)$,
partial-fraction $\hat U(s)$ against the poles of $G_a(s)G_p(s)$ and of
$\hat U(s)$ itself. Each pole $p_k$ contributes a mode $e^{p_kt}$ whose
coefficient comes from the cover-up rule (multiply by $(s-p_k)$, evaluate at
$s=p_k$). General structural fact: the persistent amplitude of the mode
belonging to any one block's pole is obtained by evaluating every other**
block in the chain at that pole's own frequency** — true for any input, which
is why it is stated before specializing.

Part II — Specializing to the steady-spin command $u(t)=Ve^{-irt}$

Why this input arises. Holding a constant inertial-frame direction $V$
while the body spins at rate $r$ forces the body-frame command to rotate
backward at rate $r$: $u(t)=Ve^{-irt}$.

Matching to $e^{st}$. An LTI system responds to $e^{st}$ by scaling it by
$G(s)$ at that exact $s$. Line up $e^{st}\leftrightarrow e^{-irt}$: exponents
match only at $s=-ir$.

Actuator response (general $G_a$): $m_{ss}(t)=G_a(-ir),Ve^{-irt}$.

Full open-loop response:

$$
\omega_{\perp,ss}(t) = G_a(-ir),G_p(-ir),Ve^{-irt},\qquad
G_p(-ir) = \frac{1/I_\perp}{-a-i(1+\lambda)r}.
$$

Both factors carry their own droop/phase pair — the plant's from
aerodynamic damping and inertia asymmetry, the actuator's from whichever
model applies (Part III).

Generalizing the input frequency. If the inertial target varies at rate
$\omega$ ($v(t)=Ve^{i\omega t}$), the body-frame command lands at
$s=i(\omega-r)$ instead of $s=-ir$ after de-spinning — the mechanism behind
the 2-D $(r,v)$ operating envelope.

Part III — Two actuator models, compared and validated

III.1 — First-order lag (teaching model)

$$
T\dot m+m=u ;\Longrightarrow; G_a(s)=\frac{1}{1+sT}, \qquad
G_a(-ir)=\underbrace{\frac{1}{\sqrt{1+(rT)^2}}}_{\text{droop}};e^{,i\arctan(rT)}.
$$

Genuine pole at $s=-1/T$ (real, negative); unforced behavior is $e^{-t/T}$.

III.2 — Pure transport delay (the committed model, A2)

$$
y(t)=u(t-\tau_a) ;\Longleftrightarrow; G_a(s)=e^{-s\tau_a}, \qquad
G_a(-ir)=e^{ir\tau_a},\quad |e^{ir\tau_a}|=1\ \forall r\tau_a,\quad \arg=r\tau_a.
$$

Pure delay is all-pass: it never attenuates a steady rotating signal, no
matter how large $r\tau_a$ is — only rotates it, exactly, not approximately.
This is why the phase-only compensator in Part V is complete correction, not
a partial fix leaving droop behind: under this model there is no droop.

III.3 — Reconciling the two

Near DC, both agree to leading order:

$$
1-G_a(s)\Big|{\text{lag}}\approx sT, \qquad 1-G_a(s)\Big|{\text{delay}}\approx s\tau_a.
$$

They diverge sharply away from DC (Part VI.B makes this precise).

III.4 — Stress-testing Lemma 1's own approximation, in advance

Lemma 1's error term $O(\omega_c\tau_a)$ is a time-domain Taylor
truncation on $u_{\text{ref}}$ ($u_{\text{ref}}(t-\tau_a)\approx
u_{\text{ref}}(t)-\tau_a\dot u_{\text{ref}}(t)$), licensed by (A3) — it is
not a frequency-response droop. If $u_{\text{ref}}$ were a pure $e^{-irt}$
exponential, the term vanishes identically and $|G_a(-ir)|=1$ holds exactly,
consistent with III.2.

Part IV — Substituting $V$ with the actual controller: full Laplace derivation

IV.1 — Real $2\times2$ plant and desired closed loop

$$
\dot x=\lambda r,S,x+\frac{1}{I_\perp}\tau,\qquad
\dot x_{\text{desired}}=M_{\text{dyn}}x,\quad M_{\text{dyn}}=\begin{pmatrix}a&-b\b&a\end{pmatrix},\ a<0.
$$

IV.2 — Torque law for a perfect actuator

$$
\tau_{\text{desired}} = I_\perp\big(M_{\text{dyn}}-\lambda r,S\big)x.
$$

$-\lambda rS$ is the NDI gyroscopic feedforward; $M_{\text{dyn}}$ is the
feedback shaping left over — the real-matrix form of $C(r)=B^{-1}(M-A(r))$.

IV.3 — Laplace substitution, isotropic actuator

$G_a(s)$ times identity (same $T$/$\tau_a$ on both channels, commutes with
the matrices). Command $\hat u(t)=\tau_{\text{desired}}(t)$ passes through
the actuator before reaching the plant:

$$
\tau_{\text{actual}}(s) = G_a(s)\cdot I_\perp\big(M_{\text{dyn}}-\lambda r S\big)X(s).
$$

Substitute into $sX(s)=\lambda r,S,X(s)+\frac{1}{I_\perp}\mathcal T(s)$ (same
signal, same substitution logic as Part I):

$$
sX(s) = \lambda r,S,X(s) + G_a(s)\big(M_{\text{dyn}}-\lambda r S\big)X(s).
$$

Collecting:

$$
\Big[sI-\lambda r S - G_a(s)M_{\text{dyn}}+G_a(s)\lambda r S\Big]X(s)=0,
$$

$$
\boxed{\Big[sI - G_a(s),M_{\text{dyn}} - \lambda r\big(1-G_a(s)\big),S\Big]X(s)=0.}
$$

IV.4 — Reading the boxed equation

$G_a(s)$ scales the feedback shaping; $\big(1-G_a(s)\big)$ multiplies the
residual, uncancelled gyroscopic term — the leak the NDI feedforward
leaves once the actuator is real. At $s=0$, $G_a(0)=1$ for either model, so
the residual vanishes exactly and eigenvalues sit exactly at $a\pm ib$. Away
from $s=0$ ($s\ne0$, unavoidable for any transient or oscillation) a piece of
the gyroscopic coupling leaks back in, proportional to $1-G_a(s)$.

IV.5 — Size of the leak at the design's own poles

At $s\approx a\pm ib$, to leading order (either model):

$$
\big|1-G_a(a+ib)\big| \approx T\sqrt{a^2+b^2} \quad(\text{or } \tau_a\sqrt{a^2+b^2}).
$$

Design criterion: the NDI feedforward is trustworthy only while
$T\sqrt{a^2+b^2}\ll1$ — the actuator must be fast relative to the
closed-loop bandwidth being demanded, not merely relative to $r$. This is
distinct from Lemma 2's realizability ceiling and should not be conflated
with it (Part VI.B).

Part V — Actuator lag: the compensator machinery, end to end

V.1 — $G^\star$: the exact static inverse under the lag model, at $s=-ir$

$G_a(-ir)=\frac{1}{1-irT}$; exact reciprocal $=(1-irT)$ (flip a fraction —
exact, no truncation). Converting to real $2\times2$ form:

$$
\boxed{G^\star = I-rT,S}, \qquad G_a(-ir)\cdot G^\star = 1 \ \text{identically, any } rT.
$$

V.2 — Splitting $G^\star$ into rotation $\times$ magnitude

$$
G^\star = I-rTS ;\longleftrightarrow; (1-irT)=\underbrace{\sqrt{1+(rT)^2}}{\text{magnitude}};\underbrace{e^{-i\arctan(rT)}}{\text{pure rotation}}.
$$

V.3 — $\Theta_\phi$: the phase-only compensator

$$
\boxed{\Theta_\phi=\frac{G^\star}{\sqrt{1+(rT)^2}}=\frac{I-rT,S}{\sqrt{1+(rT)^2}}} ;\longleftrightarrow; e^{-i\arctan(rT)}.
$$

Magnitude exactly 1, determinant 1 — adds/removes no energy, only redirects.
Verification:

$$
G_a(-ir)\cdot\Theta_\phi = \frac{1}{\sqrt{1+(rT)^2}}e^{i\arctan(rT)}\cdot e^{-i\arctan(rT)} = \frac{1}{\sqrt{1+(rT)^2}}.
$$

Phases cancel exactly, leaving only the droop the actuator produces anyway.
Bonus over full $G^\star$: never inflates the command, so no added saturation
risk (full $G^\star$'s boost $\sqrt{1+(rT)^2}$ grows unboundedly and can
saturate the actuator before its own droop brings the command back down).

V.4 — $\Theta_{\text{ideal}}(s)$: the dynamic, all-frequency inverse

$$
\Theta_{\text{ideal}}(s) = G_a(s)^{-1} = 1+sT, \qquad G_a(s)\Theta_{\text{ideal}}(s)=1\ \forall s.
$$

Substituted into the Part IV boxed equation, the residual
$\big(1-G_a(s)\big)\lambda rS$ vanishes at every frequency. Time domain:
$\hat u(t)=\tau_{\text{desired}}(t)+T\dot\tau_{\text{desired}}(t)$ —
deliberate overshoot proportional to the target's own rate of change.

V.5 — Why it's not realizable, and the practical fix

$|1+i\omega T|\to\infty$ as $\omega\to\infty$: unbounded noise amplification.
Also a structural obstruction: for any stable, causal (minimum-phase) filter,
phase lead across a band cannot be had without an accompanying rise in gain
over that band — magnitude and phase are not independently assignable, so an
all-frequency, gain-flat lead filter does not exist. Practical remedy — a
lead compensator:

$$
\boxed{\Theta_{\text{practical}}(s)=\frac{1+sT}{1+sT_f}}, \qquad T_f\ll T.
$$

Smaller $T_f$: better cancellation, more noise. Larger $T_f$: gentler on
noise, more uncancelled lag. This is exactly the design space your stated
INDI / disturbance-observer preference already occupies — INDI achieves a
functionally similar lead effect from a measured $\dot\omega$ rather than
an exact model-based differentiator, avoiding the need to trust $T$ or
$\lambda$ numerically.

V.6 — $b^\star=\tan(r\tau_a)$: a different mechanism, matched to the delay model

$b^\star$ is not a feedforward pre-rotation like $\Theta_\phi$ — it is the
off-diagonal entry of the feedback matrix itself, chosen so the closed-loop
pole's own angle cancels the delay's rotation on the tracking-error dynamics:

$$
\psi=-r\tau_a\ (\text{delay's exact rotation, III.2}),\qquad
\theta=\arg(a+ib)=\arctan(b/a),
$$
$$
\alpha(\theta)=\theta+\psi,\quad \frac{\partial\alpha}{\partial\theta}=1>0\ (\text{unique zero}),\qquad
\alpha=0 \iff \theta=r\tau_a \iff b^\star=|a|\tan(r\tau_a).
$$

Two different arctan-type quantities appear in this document and should not
be conflated: $\arctan(rT)$ (III.1) is the lag model's own frequency
response phase; $\theta=\arctan(b/a)$ here is the inverse operation —
converting a desired pole angle into the matrix entry $b$ that produces it.
Because the delay model is exactly all-pass, $b^\star$ alone is complete
correction — there is no droop left to also fix.

V.7 — A cleaner parametrization: $M_{\text{dyn}}=\rho,R(\theta)$, and folding the gyroscopic residual into the same tuning

Motivation. In V.6, $a$ is fixed a priori (Hurwitz design choice, e.g.
$a=-1$) and $b$ is solved for. This silently couples the compensation angle
to the loop's decay rate: since $\theta=\arctan(b/a)$ with $a$ pinned,
increasing $\theta$ toward the lag ceiling forces
$\rho\equiv|a+ib|=|a|/\cos\theta$ to blow up — exactly the mechanism behind
Lemma 2's realizability ceiling. There is no independent knob to make the
loop converge faster without also changing the compensation angle, or to
change the compensation angle without an automatic, forced change in
overall loop gain.

Reparametrization. Write the pole directly in polar form. With
$a=-\rho\cos\theta$, $b=\rho\sin\theta$ ($\rho>0$, $|\theta|<\pi/2$ for
Hurwitz):

$$
M_{\text{dyn}} = aI+bS = \begin{pmatrix}a&-b\b&a\end{pmatrix}
;=; -\rho\begin{pmatrix}\cos\theta&-\sin\theta\\sin\theta&\cos\theta\end{pmatrix}
;\equiv; -\rho,R(\theta).
$$

Eigenvalues are $-\rho e^{\pm i\theta}$: $\rho$ is the pole's magnitude
(overall speed of everything — decay and rotation, scaled together), and
$\theta$ is its phase (the rotation-plane angle, tunable independently of
$\rho$). This is the exact real-matrix analogue of the magnitude/phase split
already used for $G^\star=$ (magnitude)$\times\Theta_\phi$ in V.2–V.3, now
applied to the feedback matrix itself rather than the feedforward
pre-compensator.

Why this separation matters practically. The ISS ball from Lemma 3,
$D/|a|$, becomes $D/(\rho\cos\theta)$ in this parametrization. Fixing $\theta$
at whatever angle is needed to cancel a lag rotation, $\rho$ remains a fully
free, independent gain: increasing $\rho$ shrinks the disturbance ball
without changing which angle is being cancelled. Under the old
$a$-fixed convention, no such independent knob exists — tightening the ball
and adjusting the compensation angle were coupled through the same pair
$(a,b)$.

Extending to the gyroscopic residual. Part IV's boxed equation showed a
second, distinct leakage term, $\lambda r(1-G_a(s))S$, arising whenever the
NDI feedforward's own model of $\lambda$ is imperfect (bounded by $\delta_J$
per A5) — this is a second source of unwanted rotation in the loop, on top
of the actuator-delay rotation $\psi=-r\tau_a$ already being cancelled by
$\theta$. Rather than attempting to compute this residual's angle from a
model of $\Delta J$ (which is precisely the inertia-identification burden
your stated preference seeks to avoid), the same online, measured-offset
adaptation already used for $\theta$ can absorb it directly: let the
total measured attitude/rate misalignment $\alpha_{\text{total}}$ (which
by construction already contains whatever rotation both the delay and
the imperfectly-cancelled gyroscopic term jointly produce, since both act on
the same physical roll–pitch channel) drive the same integral law,

$$
\dot\theta = -k,\alpha_{\text{total}}, \qquad \alpha_{\text{total}} = \theta + \psi_{\text{delay}} + \psi_{\text{gyro,residual}}(\Delta J, r),
$$

so that $\theta$ converges to whatever single angle zeroes the combined
misalignment, without ever separately identifying $\psi_{\text{gyro,residual}}$
or $\Delta J$ — the adaptation law only ever needs the measured $\alpha$, not
its decomposition into causes. This is the same INDI-flavored argument used
for $\Theta_{\text{practical}}$ in V.5 (compensate from a measured quantity,
not a modeled one), applied here to the feedback-pole angle instead of a
feedforward filter — and it is a more reasonable place to put this
tuning than either (i) a fixed, offline-computed NDI gain requiring
trusted $\Delta J$, or (ii) a full dynamic $\Theta(s)$ requiring
differentiation of the state, since $\theta$ is a single scalar, updated by
the same low-bandwidth integral law already present in the design, with
$\rho$ left free to separately set how aggressively the whole loop responds.
Caveat: this only works cleanly if the delay-induced and gyroscopic-
residual rotations are not required to be cancelled at different points in
the frequency response — since both are being folded into one static angle
$\theta$, this is a first-order (small-argument) argument, valid while both
contributing rotations are simultaneously well inside the regime where a
single linear rotation approximates each (i.e., not simultaneously pushing
$r\tau_a$ near the lag ceiling and expecting the same $\theta$ to also
absorb a large gyroscopic-residual angle) — a joint realizability ceiling,
analogous to but generally tighter than Lemma 2's $r\tau_a<\pi/2$ alone,
would need to be re-derived if both effects are simultaneously large.

Part VI — The lemmas, then the stress test

VI.0 — Assumptions referenced below (A1–A7)

The lemmas are stated "under" a numbered list of standing assumptions from
the envelope document. They are collected here once, since they are cited by
label throughout and are otherwise easy to lose track of:

Label Assumption Plain meaning





A1

Frozen spin

$r$ varies slowly enough ($\dot r<0.02$ rad/s) that it can be treated as a constant, known number for the duration of any one analysis — it relaxes to a spin-invariant $r_{ss}$ largely insensitive to the attitude gains. This is what licenses treating $r$ as a fixed parameter rather than a state to be dynamically compensated.

A2

Actuation lag (delay model)

Body-frame actuation is modeled as a pure transport delay $\tau_a$, and — importantly — the same $\tau_a$ on both roll and pitch (a "symmetric channel"). This is the committed model from Part III.2; it is what makes Lemma 1's effect a pure rotation rather than a shear (see Part III and the asymmetry caveat, row 6 of the Part VII table).

A3

Slow reference

The closed-loop attitude bandwidth $\omega_c$ satisfies $\omega_c\tau_a\ll1$ — the commanded reference changes slowly compared to the delay. This licenses the Taylor-truncation step used in Lemma 1's proof (Part III.4); it is a statement about the reference signal's bandwidth, not the actuator.

A4

Hurwitz design

The desired closed-loop pole $a+ib$ has $a<0$ (strictly stable/decaying); $b$ is left as a free design parameter. This is what guarantees the loop is stable before any lag compensation is even considered.

A5

Bounded inertia error

The mismatch between the true inertia and the value used in the NDI feedforward, $\Delta J$, is bounded: $|\Delta J|\le\delta_J$. This is the source of the gyroscopic feedforward residual discussed in Part V.7 and row 5 of the Part VII table.

A6

Bounded estimator error

State-estimation error is bounded, $|e|\le\bar e$, from the validated EV/EKF2 configuration, at external-vision update rate $f_{EV}$. Feeds the disturbance budget $D$ in Lemma 3 via the estimator term.

A7

Centred airframe

Center-of-mass offset is negligible, so centrifugal terms can be dropped; inertia-tensor error (including products-of-inertia error) enters the analysis only through $\Delta J$, with no separate CoM-offset channel to track.

With these in hand, Lemma 1's citation of (A1)–(A3) reads as: given that
$r$ can be treated as fixed, the actuator is a pure symmetric delay, and the
reference is slow relative to that delay — the three conditions jointly
needed for the delay-to-rotation reduction to hold as stated.

VI.A — The lemmas, stated as a block

Lemma 1 (delay $\to$ constant rotation). Under (A1)–(A3):
$$
y_{\text{ref}}(t) = e^{-ir\tau_a},u_{\text{ref}}(t)\big(1+O(\omega_c\tau_a)\big).
$$
Uncompensated, the loop sees a fixed control-axis rotation of $-r\tau_a$.

Lemma 2 (cancellation, convergence, ceiling). With $\theta=\arctan(b/a)$,
$\psi=-r\tau_a$, $\alpha=\theta+\psi$:
$$
b^\star=|a|\tan(r\tau_a) \ \Rightarrow\ \alpha=0;\qquad
\dot\theta=-k\alpha \ \Rightarrow\ \alpha(t)=\alpha(0)e^{-kt}.
$$
Realizability requires $r\tau_a<\phi_{\max}<\pi/2$, i.e.
$r\le\kappa,\pi/(2\tau_a)$.

Lemma 3 (ISS ball). With $\sup_t|d_c(t)|\le D$:
$$
\limsup_{t\to\infty}|\zeta(t)| \le \frac{D}{|a|}, \qquad \text{independent of } b.
$$

VI.B — Stress test

Lemma 1. Holds as long as (A2) and (A3) both hold at the same time. One
thing worth being clear about: its error term comes from a Taylor
approximation of the reference signal in time (see III.4) — it is not a
statement about the actuator losing amplitude at some frequency. Mixing
those two up would misattribute where the error actually comes from. The
lemma also leans on (A1): if $r$ changes a meaningful amount within one
delay window $\tau_a$, the de-spinning trick it's built on only holds
approximately.

Lemma 2. Exact, given the delay model and a fixed $a$, with roll and
pitch delayed by the same amount. When $b^\star\to\infty$, that means the
feedback gain you'd need is blowing up — not that some amplitude droop is
sneaking back in. (Section III.2 already showed the delay model has no
droop at all, at any spin rate, so that's consistent.) The spin ceiling
$r\le\kappa\pi/(2\tau_a)$ is a real, load-bearing number — it should be
checked against whatever $\tau_a$ is actually measured on hardware, not
assumed.

The $\rho$–$\theta$ split from V.7 makes one thing visible that this lemma's
$(a,b)$ form hides: the ceiling exists because $a$ is being held fixed while
$\theta$ grows. The angle $\theta$ still can't pass $\pi/2$ — freeing $\rho$
doesn't remove that limit. What it does give you is a separate knob: you can
speed up or slow down how fast the loop converges without changing how close
$\theta$ already is to the ceiling. The plain $(a,b)$ version doesn't let you
see or control that separately.

Lemma 3. The proof is a standard comparison-lemma argument, and it's
clear why $b$ drops out: $b$ only ever sits in the imaginary part of the
pole, and the bound only depends on the real part. Two things worth flagging
that the lemma itself doesn't cover:

Part IV found a separate leakage term, $\lambda r(1-G_a(s))S$. This comes
from the actuator being imperfect while trying to cancel the gyroscopic
term — it is not an external disturbance, so it isn't automatically part of
$D$. If $T\sqrt{a^2+b^2}$ turns out not to be small at your actual gains,
this term should be added to $D$ explicitly, not assumed to already be
inside it.

The "delay never loses amplitude" claim in III.2 and V.6 is only exactly
true for a signal that's purely oscillating (no growth, no decay) — that is,
$s$ purely imaginary. Your actual closed-loop pole decays, so
$s=a+ib$ with $a<0$. Evaluating the delay there:

$$
\big|G_a(a+ib)\big| = \big|e^{-(a+ib)\tau_a}\big| = e^{|a|\tau_a} ;;(>1).
$$

This doesn't contradict III.2 — it just means "delay never loses amplitude"
only applies to steady spinning, not to a decaying transient. Away from that
special case, delay actually adds gain, growing with $|a|\tau_a$. This is
the same effect that makes people cautious about Padé approximations of
delay near a stability boundary. For small $|a|\tau_a$ it's negligible and
matches the lag model too ($1-G_a(s)\approx s\tau_a$, from III.3) — the risk
only shows up once $|a|\tau_a$ gets large, which is exactly where the Part
IV leakage term also starts to matter. The two should be treated as
compounding, not as two separate small effects.

Overall. Every lemma's proof holds under the assumptions it states. The
points above aren't cracks in the lemmas — they're extra terms that sit
outside what each lemma was built to cover (the Part IV leakage, the
off-axis delay gain, the joint ceiling from V.7). Worth tracking each one
explicitly in the envelope write-up rather than assuming the existing ISS
margin already absorbs them.

Part VII — Summary table: every lag mechanism and its fix

# Lag source Model Effect on command Where it lives Fix Mechanism type













1

Actuator, first-order roll-off (teaching model)

$G_a(s)=\frac{1}{1+sT}$

Droop $\frac{1}{\sqrt{1+(rT)^2}}$ and phase $\arctan(rT)$ at spin rate $r$

Open loop, steady-spin hover

$G^\star=I-rT,S$ (exact, both effects) or $\Theta_\phi$ (phase only, if droop is acceptable)

Static feedforward, evaluated at known $r$

2

Actuator, pure transport delay (committed model)

$G_a(s)=e^{-s\tau_a}$

Rotation $r\tau_a$ only — no droop, exactly

Open loop, steady-spin hover

$b^\star=\lvert a\rvert\tan(r\tau_a)$ (pole-angle placement) — equivalently a $\Theta_\phi$-type pre-rotation by $r\tau_a$

Static, either feedback-pole placement or feedforward pre-rotation

3

Actuator lag acting on the closed-loop state itself (not a known external frequency)

$G_a(s)$, general

Residual $\big(1-G_a(s)\big)\lambda r,S$ leaking into the NDI-cancelled gyroscopic term

Closed loop, IV.3–IV.5

$\Theta_{\text{ideal}}(s)=1+sT$ (exact, all $s$, not realizable) $\to$ $\Theta_{\text{practical}}(s)=\frac{1+sT}{1+sT_f}$ (realizable lead compensator)

Dynamic filter, or INDI-style measured-derivative substitute

4

Rigid-body gyroscopic precession (plant's own pole)

$G_p(s)=\frac{1/I_\perp}{s-a-i\lambda r}$

Persistent undamped/weakly-damped rotation at $\lambda r$, independent of the actuator

Open loop, plant alone

NDI feedforward $-\lambda r,S$ (needs $\lambda$, i.e. inertia)

Static feedforward, model-based

5

Gyroscopic feedforward residual from imperfect $\Delta J$ (inertia error, A5)

Same structural form as #3, now sourced from model error rather than actuator

Extra uncancelled rotation, same channel as #1–#3

Closed loop

Fold into the same online $\theta$-adaptation as #2 (V.7) — do not identify $\Delta J$ separately

INDI-flavored: measured-offset adaptation, no model of $\Delta J$ required

6

Lag asymmetry, $\varepsilon_T$ (roll $T\ne$ pitch $T$)

Anisotropic $G_a=\mathrm{diag}(g_x,g_y)$

$2r$-shear: couples command into its own conjugate, invisible to any scalar rotation

Open loop

Full $2\times2$ $G^\star$ that inverts the anisotropic matrix at $s=-ir$, not the scalar/rotation version

Static feedforward, needs both $T_x,T_y$

7

Delay evaluated on a decaying (not purely oscillating) closed-loop pole

$G_a(a+ib)=e^{\lvert a\rvert\tau_a}$ (gain $>1$, not all-pass off-axis)

Amplifies, does not merely rotate, for $a\ne0$

Closed loop, VI.B

No closed-form fix derived here — track as a compounding risk with #3 at large $\lvert a\rvert\tau_a$; keep $\lvert a\rvert\tau_a$ small by design