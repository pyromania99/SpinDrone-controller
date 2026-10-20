# Numerical Analysis of Glauert Inflow Formula for Single-Rotor Helicopter in Steady-Level Flight below Stall-Flutter Limit

*Extracted text. Regenerable; edit notes.md instead.*

aerospace Article

Numerical Analysis of Glauert Inflow Formula for Single-Rotor Helicopter in Steady-Level Flight below Stall-Flutter Limit Marjan Dodic 1,2 , Branimir Krstic 1, * , Bosko Rasuo 2 , Mirko Dinulovic 2 and Aleksandar Bengin 2 1

2

*

Section of Aerospace Engineering and Mechanics, Department of Military Mechanical Engineering, Military Academy, University of Defence in Belgrade, Veljka Lukica Kurjaka 33, 11042 Belgrade, Serbia Department of Aerospace Engineering, Faculty of Mechanical Engineering, University of Belgrade, Kraljice Marije 16, 11120 Belgrade, Serbia Correspondence: branimir.krstic@va.mod.gov.rs

Abstract: This article addresses the numerical computation problem of induced inflow ratio based on the helicopter momentum theory in forward flight. The Glauert inflow formula (equation) is a nonlinear equation usually solved by the Newton–Raphson method in a relatively small number of iterations. However, many high-order convergence multipoint iterative methods have been developed over the last decade. The study examines several selected methods in terms of finding ones that provide a solution in only one iteration with acceptable accuracy. Furthermore, the influence of initial guesses on the accuracy of the obtained solutions has been investigated. In this regard, the practical range of parameters of the Glauert inflow equation for helicopters in forward flight is roughly determined by simplified modeling of a power and stall-flutter limitation. For these purposes, a basic low-fidelity longitudinal trim model of a single-rotor helicopter in steady-level flight is modified and numerically solved by a symbolic transformation of a system of 20+ nonlinear equations into a single nonlinear equation. Keywords: Glauert inflow formula; numerical solution; multipoint iterative methods; nonlinear equations; single-rotor helicopter; steady-level flight; stall-flutter limit Citation: Dodic, M.; Krstic, B.; Rasuo, B.; Dinulovic, M.; Bengin, A. Numerical Analysis of Glauert Inflow Formula for Single-Rotor Helicopter in Steady-Level Flight below Stall-Flutter Limit. Aerospace 2023, 10,

238. https://doi.org/10.3390/

aerospace10030238 Academic Editor: Konstantinos Kontis Received: 7 January 2023 Revised: 20 February 2023 Accepted: 24 February 2023 Published: 28 February 2023

Copyright: © 2023 by the authors. Licensee MDPI, Basel, Switzerland. This article is an open access article distributed under the terms and conditions of the Creative Commons Attribution (CC BY) license (https:// creativecommons.org/licenses/by/ 4.0/).

1. Introduction

Momentum theory is the first analytical tool for theoretically analyzing rotor thrust in forward flight conditions. It was developed back in 1926 by Hermann Glauert as a part of a broader, general theory for autogyro applications [1]. This basic theory represents a forerunner of the niches of today’s rotary-wing aerodynamic models [2]. Regardless of their complexity and primary purpose, all these models deal with a problem of induced velocity distribution at the rotor disc. In the early days, uniform distribution was routinely employed for rotorcraft performance estimation, real-time simulations, flight dynamics, and control applications. However, decades of strong computational development have enabled increasingly complex and more realistic aerodynamic models, including free-wake analysis, computational fluid dynamics, comprehensive tools for fluid-structure interaction, and dual-solver hybrid computational approaches [3]. Still, the current CPU/GPU performance level limits the use of CFD and CFD/CSD techniques in real-time applications. In this regard, when it comes to robust mathematical models applied in flight simulators, dynamic control, and aeroelastic stability evaluations, as well as preliminary performance estimations, nonuniform, both static and dynamic inflow models still play a dominant role [2,4]. This group of models includes linear ones whose induced velocity distribution is based on its main value at the rotor disc center, obtained by the momentum theory. Based on that, the problem of solving the Glauert inflow formula (GIF) is actual, and the momentum theory for vertical and forward flight is still a subject of improvement [5,6]. Momentum theory for forward flight is an analytical tool, and GIF, which relates forward flight speed, downwash flow, and thrust, has a complex analytical solution. Although

Aerospace 2023, 10, 238. https://doi.org/10.3390/aerospace10030238

https://www.mdpi.com/journal/aerospace

the edgewise flight assumption leads to less accurate solutions for low advance ratios, it is still used in practice, especially for performance estimation during the preliminary design of the helicopter [7]. In the last more than a half-century, the numerical solution of GIF represents a standard approach in the rotorcraft community, and the classic Newton– Raphson (NR) method for solving nonlinear equations is commonly used. It is robust and has good local convergence. However, like all iterative one-point methods, it has a theoretical limitation established by Traub [8], who says that an iterative function φ( xk ) that explicitly depends on the p − 1 derivative of the function f ( x ) whose zero is required cannot attain an order of convergence higher than p. Consequently, the NR method has a convergence order p = 2, which provides approximately two significant digits in one iteration. Increasing the order of convergence of one-point methods is only possible by increasing the number of derivatives. However, it limits computational efficiency, which has resulted in decades of stagnation in this area [9]. The rapid development of computer hardware, symbolic algebra software, and high-precision arithmetic leads to increased interest in multipoint iterative methods because of their computational efficiency, which is much greater than the efficiency of one-point methods [10]. During the first decade of the 21st century, at least 200 new multipoint methods have been developed, and that number continues to increase [11]. On the other hand, Newton-based methods are also being developed and used to solve various engineering problems. Quasi-Newton methods are used in machine learning, neural network training, fluid-structure interaction, and other engineering areas [12–15]. Recently, the NR method has been used in coupling with other iterative algorithms as an alternative to the traditional finite element method for large multiphysics problems [16]. Jacobian-free NR-based methods for solving multiphysics nonlinear systems are also being improved [17]. Although powerful numerical methods have been generated in the last 30 years, the rotorcraft industry and science still dominantly use a method that originated more than 300 years ago. It is indisputable that the NR method has a quality that keeps it alone at the top of the list of used numerical methods for solving nonlinear equations. Still, there is a need to examine the utility value of newly developed ones. If the GIF is solved numerically by a novel multipoint method, that would primarily increase the computational efficiency. In addition, if an acceptable solution accuracy is reached in just one iteration step, that would increase the number of users preferring classical approximations rather than numerical methods. Finally, novel dual-solver hybrid approaches reopen doors for simple theories, such as momentum theory, to be used in a couple with high-fidelity models, which gives additional scientific importance to improving the GIF solution procedure. It is essential to point out that a mathematical analysis of GIF cannot be found in the literature [18]. This paper aims to change that fact and use breakthroughs in symbolic and numerical mathematics to improve solving procedure of the low-fidelity model of single-rotor helicopter longitudinal force equilibrium in forward flight, which is necessary to use to determine the parameters of the GIF equation.

2. GIF Formulation and Solution Procedure

According to the momentum theory for forward flight (Figure 1), the GIF is a transcendent fixed-point equation for the induced inflow ratio:

CT

λi = q ,

2 µ2 + ( µ z + λ i )2

(1)

which can be expressed in algebraic irrational zero-point form as

CT

λi − q = 0,

2 µ2 + ( µ z + λ i )2

(2)

2 𝜇

𝜆 with the algebraic zero‐point irrational form given as 𝜆

𝜇tan𝛼

0,

(5)

and rational form: 4𝜆

8𝜇tan𝛼 𝜆

4𝜇 tan 𝛼

1 𝜆 ‐8𝜇3 tanα 𝜆

4𝜇4 tan2 αD

𝐶

0.

(6)

(3) and (6)

areas quartic algebraic equations that can be represented as and inEquations rational (normal) form follows

A4 𝑥

A 𝑥 A 𝑥

A 𝑥 A

0. (7) (3) 4λ4i + 8µ tan α D λ3i + 4µ2 tan2 α D + 1 λ2i − CT2 = 0. Constants A0 , A , A , A , and A4 are given in Table 1.

Figure1.1.Rotor Rotordisc discininforward forwardflight. flight. Figure Table

1. Coefficients

of the GIF rational form. Instead of the induced inflow ratio λi , the rotor inflow ratio λ can be used as a variable in Equation (1): Constant Variable 𝒙 C 𝝀𝒊 Variable 𝒙 𝝀 λ = λi + µ z = 4 p T + µ tan α D (4)

A4

4

2 µ2 + λ2

A3

8𝜇tanα ‐ 8𝜇 tanαD

D

A2

𝛼 given

1 as

4𝜇2 tan2 αD 1 4𝜇2 tan with the algebraic zero-point irrational form

A

0 8𝜇3 tan αD 4

C

A

𝐶 4𝜇 tan 𝛼 λ − p T‐ 𝐶 − µ tan α D = 0, (5)

2 µ2 + λ2

Equations (2) and (5) are irrational because they have an unknown in the radicand. and rational(3) form: Equations and (6) represent their normal (rational) forms, respectively. Although no  in the literature  recommendation can be found dealing with the question of which GIF 4λ4 − 8µ tan α D λ3 + 4µ2 tan2 α D + 1 λ2 −8µ3 tan α D λ + 4µ4 tan2 α D − CT2 = 0. (6) form is the best for numerical solving, there is a clear reason for preferring an irrational form. Namely, in addition to the roots of the original (irrational) form, the normal form Equations (3) and (6) are quartic algebraic equations that can be represented as may contain some superfluous roots that are not solutions of the original form [19]. On

A4 x4 + A3 x3 + A2 x2 + A1 x + A0 = 0.

(7)

Constants A0 , A1 , A2 , A3 , and A4 are given in Table 1. Table 1. Coefficients of the GIF rational form. Constant

Variable x=λi

Variable x=λ

A4

A3

A2

A1

A0

4 8µtan α D  4µ2 tan2 α D + 1 0

− CT2

4 − 8µ tan α D 4µ2 (tan 2 α D +1) − 8µ3 tan α D 4µ4 tan2 α D − CT2

Equations (2) and (5) are irrational because they have an unknown in the radicand. Equations (3) and (6) represent their normal (rational) forms, respectively. Although no recommendation can be found in the literature dealing with the question of which GIF form is the best for numerical solving, there is a clear reason for preferring an irrational form. Namely, in addition to the roots of the original (irrational) form, the normal form may contain some superfluous roots that are not solutions of the original form [19]. On the other hand, the rational form is more suitable for calculating derivatives, which can be a significant factor if a one-point method of a high order of convergence is engaged to

solve the equation. There are four forms of the GIF, which can be solved numerically, and Equations (2), (3), (5), and (6) represent them. Procedures for numerical solving of GIF can be found in [20] (pp. 97–98) and [21] (pp. 126, 579–580). Both authors use the NR method to solve an irrational form for λ (Equation (5)) and state that three-four iterations are sufficient in most cases. Leishman [20] suggests two numerical methods with practical engineering use: a simple fixed-point iteration (FPI) and NR method, where FPI typically requires 10–15 iterations, but in some cases, especially at lower advance ratios, a larger number of iterations may be necessary. For stopping criteria, the author uses an error estimator as follows: λ n +1 − λ n , λ n +1

ε=

(8)

and states that when ε ≤ 0.0005 convergence is said to occur. According to Leishman, the initial value for λ0 is usually the inflow ratio for hover: λh =

p

CT /2.

(9)

W. Johnson [21,22] recommends the initial value for λ0 as λwj = q

λ2h

( λ h + µ z )2 + µ2

+ µz .

(10)

In addition, the author proposes using a relaxation factor r f = 0.5 for convergence improvement: f (λ) . (11) λ n +1 = λ n − r f 0 f (λ) The initial value given in Equation (10) can be modified to become even closer to the root as λ2h λmd = q + µz , (12) ( λ e + µ z )2 + µ2 where λe is the main inflow ratio for edgewise flight (α D = 0): 1 λe = √ 2

rq

µ4 + 4λ4h − µ2 .

(13)

Next to the numerical solution for λ, the GIF can also be solved for λi . However, in the literature, no analysis compares the number of iterations and other parameters of the numerical solution for those two variables. Therefore, six initial guesses (values) for λ0 are considered in this study, as it is shown in Table 2. Table 2. Initial guesses λ0 for GIF numerical solutions. Main Inflow Ratio λ Equations (5) and (6)

Induced Inflow Ratio λi , Equations (2) and (3)

Reference

λh

λh

Leishman [20]

λ2h

+ µz ( λ h + µ z )2 + µ2 λ2h q + µz ( λ e + µ z )2 + µ2

q

λ2h

q

( λ h + µ z )2 + µ2 λ2h q ( λ e + µ z )2 + µ2

W. Johnson [15,16] Current study

A few years ago, GIF’s analytical, closed-form solution was presented and offered to the rotorcraft community as a benchmark solution in helicopter aerodynamics for comparing numerical results [18]. It is a relatively new, easily programmable solution based on

linear algebra and symbolic mathematics, with the potential to be used instead of numerical solving. Since the author examined only one case (µ = 0.1, CT = 0.003, α D = 4◦ ) with the primary pedagogical goal of demonstrating the power of novel mathematical tools, the analytical solution is not taken as a benchmark solution. However, it is evaluated, along with numerical methods.

3. Models and Methods

GIF has only three parameters: advance ratio µ, thrust coefficient CT , and rotor disc incidence angle (angle of attack) α D . These parameters are mutually determined, and their ranges need to be defined. Only for this research, the union of all µ, CT , and α D combinations that may occur in steady-level flight conditions are named the Acceptable Parameter Range (APR). It is a fictive union of all steady-level flight regimes of an infinite number of single-rotor helicopters that may exist in practice. Accordingly, two tasks are imposed:

• •

Definition of a method for determining the APR; Presentation of the APR with a finite number of cases.

Precise determination of APR boundaries is a complex and time-consuming process requiring sophisticated and computationally demanding tools. The main goal is to exclude from the numerical analysis as many cases as possible that have no practical significance and to ensure that a potential conclusion about the use of a particular numerical method in solving the Glauert inflow equation refers to the widest possible ranges of steady-level flight conditions, take-off masses, geometric and aerodynamic characteristics of helicopters. Therefore, it is enough to use a robust, fast, and sufficiently reliable tool capable of providing that the APR boundaries it sets encompass all possible cases. For that reason, the following steps are adopted: 1. 2. 3. 4.

Selection of a finite number of cases for APR boundaries determination purposes; Establishing the APR boundaries as CT min = f 1 (µ), CT max = f 2 (µ), α D min = f 4 (µ) and α D max = f 4 (µ); Establishing a more expansive APR zone with an acceptable safety margin; Determination of the finite number of possible cases as APR zone representatives for numerical methods evaluation purposes.

The range of possible cases is very wide. There are many ultralight RW-UAV applications and a few single-rotor helicopters with a maximum take-off mass (MTOM) below

300 kg. The MTOM of the largest helicopter in the world is greater than 50,000 kg. The

Official World Record in the category Rotorcraft–Altitude in horizontal flight set above ISA troposphere boundary (11,010 m) by sky-crane helicopter Sikorsky CH-54B on 4 November

1971. The fastest helicopter in the world is the British Lynx, with an achieved forward flight

speed of 400.87 km/h. The Lynx held this title for 37 years. Considering the high variability in rotorcraft take-off mass, flight speed, altitude, and other characteristics, around 7 million points are selected and divided into four working groups (WG), as shown in Table 3. The selection process is based on past and current design trends [21–26]. Next, the parameters a, b, and k are used to determine helicopter main and tail rotor diameter, angular velocity, and equivalent helicopter drag/lift area by statistics. Finally, the control group (CG) is used for grid density validation. Table 3. Finite number of cases of Working (WG) and Control Groups (CG). Characteristics MTOMmin /∆m/MTOMmax 1 hmin /∆h/hmax fe,max /fe,mean /fe,min Vmin /∆V/Vmax

Group 1

WG

CG

1000/500/8500 1000/200/8800

Group 2

Group 3

9000/2000/19,000 20,000/5000/35,000 9000/400/19,600 20,000/800/40,000 0/500/11,500 Calculated based on [21–26] 1/1/131

Group 4 40,000/10,000/80,000 41,600/1600/80,000

Aerospace 2023, 10, x FOR PEER REVIEW

Table 3. Cont.

σCharacteristics min/σmean/σmax σmina/σ minmean /amax/σmax a /amax max min bmin/b bmin /bmax kmin/kmax kmin /kmax No. of No. ofWG WG points points No. No.ofofCG CG points points TotalNo. No.of ofWG WG points: points: Total 1

Group 1

Group 20.04/0.01/0.2 Group 3

Group 4

3,621,888 3,621,888 32,823,360 32,823,360 7,017,408 7,017,408

0.04/0.01/0.2 130/150 130/150 0.85/1.15 0.85/1.15 0.39/1.35 0.39/1.35 1,358,208 905,472 1,358,208 905,472 22,155,768 21,335,184 22,155,768 21,335,184 Total

CG

points: Total No.No. of of

CG

points:

1,131,840 1,131,840 20,514,600 20,514,600 96,828,912 96,828,912

® LFEM code). Three differentTOM

TOM

used for each

MTOM:

mean, (see®the Julia Three different areare used for each

MTOM:

empty,empty, mean, and maxand (seemax the Julia

LFEM

code).

The process is to develop an algorithm for GIF pa‐ The first first task taskininthe theAPR APRestablishing establishing process is to develop an algorithm for GIF rameter determination. Considering the parameter determination. Considering thenumber numberofofcalculations calculationsthat thatmust must be be performed performed to determine the

APR

zone is measured in millions, a simplified approach to the problem to determine the APR zone is measured in millions, a simplified approach to the probof helicopter‐trimmed flight is necessary. In this regard, only the longitudinal force lem of helicopter-trimmed flight is necessary. In this regard, only the longitudinal equi‐ force librium model

(LFEM)

is used, as shown in Figure

2. 2.

equilibrium model

(LFEM)

is used, as shown in Figure

Figure

2. (a)

parameters within longitudinal force Figure 2. (a) A A simplified simplified view view of of determining determining the the GIF GIF parameters within the the longitudinal force equilibrium model

(LFEM);

(b) Single‐rotor helicopter longitudinal force model in forward flight. flight. equilibrium model (LFEM); (b) Single-rotor helicopter longitudinal force model in forward

LFEM is is based basedon onaasimple simplelow-fidelity low‐fidelity longitudinal trim model given in [27] longitudinal trim model given in [27] (pp. (pp. 116– 116–118). The longitudinal equilibrium equations for steady‐level 118). The longitudinal forceforce equilibrium equations for steady-level flightflight are: are: 𝑇 sin 𝐻 cos 𝛼 𝐷 cos ∆𝛼 𝐻 0, 2 T sin (α D )𝛼+ L F 𝐿sinsin (∆α∆𝛼 D ) − Hc cos ( α D ) − D cos( ∆α D ) − HTR = 0,

(14) (14)

and and 𝑇 cos 𝛼 𝐻 sin 𝛼 cos 𝛼 𝑊 𝐿 cos ∆𝛼 𝐷 sin ∆𝛼 0. (15) T cos(α D ) + Hc sin(α D ) cos(α D ) − Wog − L f cos(∆α D ) − D sin(∆α D ) = 0. (15) The lack of complete trim equations (longitudinal moment balance and lateral force The lack balance) of complete triminequations (longitudinal moment solutions, balance and lateral force and moment results a lower quality of the obtained which is com‐ and moment balance) results in a margin lower quality of the obtained which is compensated by increasing the safety of the established APRsolutions, boundaries. Because of pensated bycomputational increasing the requirements, safety margin of the established APRimperfections, boundaries. Because of demanding flapping effects, rotor and non‐ linearities are also neglected.

demanding computational requirements, flapping effects, rotor imperfections, and nonlinearities are also neglected. On the other hand, usual small angle assumption, typical for simplified low-fidelity models, cannot be applied, which significantly complicates the mathematical model and increases the number of nonlinear equations that must be solved simultaneously. In addition, airframe lift force and tail rotor drag force are addressed. The solution is obtained in two steps: LFEM transformation into a single longitudinal force equilibrium equation (LFE) and numerical solution of LFE using the NR method. LFEM and its numerical solution procedure are described in detail in Appendix A.

3.1. Available Power Limit

Many real limitations may be included in the LFEM model. The most important are available power and dynamic stall (stall-flutter) limit. Available power is strongly limited and is represented by Equation (16) which is obtained by statistical data from [23]: PTO = 0.0764WMTO 1.1455 , {ε av = 14%, ε MAX = 37%, Rc = 0.989, WMTO [kg]}.

(16)

Since the maximum errors have always been on the side of overestimating the available power, the following expression is adopted for the maximum take-off power in [kW]: 1.1455

PTO = 87.096WMTO

.

(17)

For altitudes up to 4000 m, the adopted model gives that the maximum available power is 95% of the output engine(s) power. The remaining 5% covers the power needs of the tail rotor, other consumers, and transmission losses. Furthermore, the novel hybrid and fully electrical power units were not observed because of their much lower gravimetric power density than hydrocarbon fuel-based power plants. For higher altitudes, available power is represented as a function of static pressure and temperature ratio: δ PAh>4000m = 0.95PTO √ . θ

(18)

On the other hand, the required power PR has been calculated using the following expression: PR = C p ρπR2 Vt3 , (19) where

σcd

1 fe 3

µ . (20) Fp + 8

2 πR2 c

The empirical factor k i considers blade tip losses, nonuniform inflow, and other rotor non-perfections, and is given in Table 4. The blade profile drag empirical factor Fp with an error of less than 1% for all speed ranges was developed in [28] as !  p 

1 + 1 + µ2c

5 2 3 2 4 + 7µ2c + 4µ4c

3 4 3 2 2

9 4

9 µ4

1 + µc + µ

− + µ + µ µ + µ ln , (21) 2 2 8

16 1 + µ2c

2 z 2 z

16 µc (1 + µ2c ) C p = k i λ i CT +

Fp =

q

1 + µ2c

and it is used in this study. In addition, as the small angle assumption is not employed, µc is used instead of µ in Equation (20). Table 4. The rotor induced power empirical factor k i . µ

ki

Reference

0<µ≤1 µ ≤ 0.5

0.5 < µ ≤ 1

µ ≤ 0.2

0.2 < µ ≤ 0.4

µ > 0.4

1.15 ÷ 1.2



1.075 cosh 6.76µ2

1.0 − 29.332µ + 92.439µ2 − 51.746µ3

1.2 2µ + 0.8 14µ − 4

Vast literature [28] Current study

3.2. Stall Flutter Limit

Dynamic stall is a phenomenon that allows the local angle of attack to grow beyond the static stall limit. Such conditions include high drag penalties, negative pitch damping, and amplified vibrations. Dynamic stall criteria usually represent a limit beyond which forward flight is unacceptable due to excessive pitch link vibrations, although the rotorcraft is usually still flyable. Many empirical dynamic stall models, such as the Boeing, ONERA, and the Leishman–Beddoes model, have been developed. These models are primarily used in complex rotary-wings modeling. On the other hand, F. Harris [29] developed simple stall criteria, which can be implemented in low-fidelity models: Cl

1 − µ2 + 49 µ4

CT

= MAX

σ

6 1 + 83 µ + 32 µ2

(22)

In this study, a simple assumption is made that the dynamic stall limit on the retreating blade is 50% greater than the static stall limit, and a slightly modified Harris criterion for a retreating blade stall at 0.75R is used as Clrb < Cl MAX where Clrb = Clav

rb,dynamic

,

(23)

1 + 83 µ + 3µ2 + 4µ3 + 49 µ4

(24)

1 − µ2 + 94 µ4

The maximum lift coefficient at 0.75R of the retreating blade depends on local Reynolds and Mach numbers. For this study, the blade airfoil maximum lift capabilities are based on NACA 0012 airfoil, and the influence of the local Reynolds number is modeled as Cl MAX

rb,static, Mrb <0.3

= 67.57 − 34.07 log Reloc + 5.722 log2 Reloc − 0.3141 log3 Reloc ,

(25)

where the local Reynolds number is defined as Reloc =

(0.75Vt − V cos(α D ))·c ν

(26)

Equation (25) is obtained from [30] (Figure 13) by the GraphPad PRISM® software tool for nonlinear regression analysis and is used if 3 × 105 ≤ Reloc ≤ 8 × 106 . The maximum lift coefficient on the retreating blade for lower Reynolds numbers is set to 1.0 and for higher ones to 1.75. Furthermore, the influence of the Mach number is also obtained from [30] (Figure 14) similarly: Cl MAX

rb,static, Mrb >0.3

=−

Cl MAX

rb, static

0.8

The maximum static lift coefficient Cl MAX

Mr + 1.1

( av, stat,)

Cl MAX

rb, static

0.8

(27)

can be calculated by Equations (25)

and (27) where M and Re are used instead of Mr and Reloc .

3.3. Numerical Methods and Procedures

This subsection presents two computational plans: a plan for APR boundary determination and a GIF numerical solution with certain numerical methods. In this regard, The Numerical Computation Plan 1 for APR determination is shown in Table 5. There are seven cycles of calculations: 6 cycles refer to 4 working groups (WG 1–4), and the last refers to 4 control groups (CG 1–4), as presented in Table 3. The control groups are introduced to check the influence of the grid density on the obtained results. In total, over 2.5 billion LFE calculations were performed. This demanding and computationally expensive task has been completed on a workstation with 20 processor cores. The computational code

is written in the Julia® programming language because it is one of the best solutions for large-scale numerical work [31]. Table 5. Numerical Computational Plan 1 (NCP 1). Cycle No.

Group

Model

Cycle No.

Group

Model

1. 2. 3. 4.

WG 1–4

WG 1–4

WG 1–4

WG 1–4

No limit Power limit Dynamic stall Power 1 limit

5. 6. 7.

WG 1–4

WG 1–4

CG 1–4

Power 2 limit Power 3 limit No limit

As can be seen in Table 5, there are three Power limits models. The Power 1 model is described in the previous subsection. Power 3 is practically the same, with the difference that the small angle assumption is adopted, which means that Equation (20) contains µ. Finally, Power 2 model uses the classic Gessow and Crim approach to the compressibility effect on the blade profile drag. The main task of this model is to serve as a benchmark for the Power 1 limit model. In this paper, GIF is first numerically solved by a three-point eight-order SharmaSharma (Sharma) method using multi-precision numerical computing. The Sharma method is robust and helpful for single and multiple root-finding problems that require only four functional evaluations [32]. A high level of accuracy is obtained by the Arb.Numerics Julia® package for arbitrary precision, which is set to 200 of the 2500 available significant digits. Numerical computing of GIF in this research is performed with the:

• • • • • •

Standard arithmetic precision; Arbitrary arithmetic precisions (only for the benchmark solution); Initial values are taken from Table (2); Stopping criteria Eo < ε (ε = 5 × 10−n , n = 4, 6, 8, 10, 12, 16) or “one iteration”; Stopping criteria Eo < 5 × 10−150 (only for the benchmark solution); Several one-point and multipoint iterative methods without memory for solving nonlinear equations.

The methods used are shown in Table 6 and thoroughly presented in [9–11]. As seen,

14 methods, 8 stopping criteria, 4 GIF forms, and 6 initial values were used, and 1344

numerical cycles were performed. Each cycle consists of about 3 million individual cases. For this work, GIF has been solved numerically about four billion times. The main goal of these numerical calculations is to obtain the following:

• • • • • •

Maximum, minimum, and average iteration number (ITNumb) as a function of µ for the entire APR. Maximum real relative error (Et ) for all examined cases; Percentage of cases of failed convergence, if any; Presence of non-physical roots; Quality assessment of used initial values; Quality assessment of iterative methods.

In addition to the above, it is essential to point out that each numerical method used in this research is written in the Julia® programming language in such a way that the iterative sequence will be interrupted in the case the stopping criteria are satisfied or the number of iterations is greater than 1000 (failed convergence) or the solution is a NaN, zero or infinite. While the acceptable solution is practically impossible to obtain in the case of failed convergence, in other cases (NaN, zero, or infinitive), the value from the previous iteration may be taken as a solution. Additionally, for multipoint methods, a solution from the previous step (point) of the same iteration may be delivered as a final. Consequently, if some multipoint method is used, the number of iterations (ITNumb) does not necessarily have to be an integer.

Table 6. GIF Numerical Computing Plan (NCP 2). Methods

Type

Halley Chebyshev Kiss

E5

NR

NR with f = 0.5 Ostrowski Chun Cordero Jarratt Sharma–Sharma

BWR

BWR II

KT

1P3R

1P3R

1P4R

1P5R

1P2R

2P4R

2P4R

2P4R

2P4R

3P8R

3P8R

3P8R

3P8R

Stopping Criteria ε < 5 × 10−4 ε < 5 × 10−6 ε < 5 × 10−8 ε < 5 × 10−10 ε < 5 × 10−12 ε < 5 × 10−14 ε < 5 × 10−16

1 iteration

Equations

Initial Guesses λo

Equation (6) Equation (5) Equation (3) Equation (2)

λh λwj λmd

4. Results and Discussion

Firstly, the results of the APR determination are shown and discussed. Then, the following analysis of the performance of the selected numerical methods for solving the GIF whose parameters are in the APR is presented.

4.1. APR Determination

Figure 3 (left) shows the maximum and minimum values of the angle of attack and thrust coefficient for the basic LFE model (No limits) and five additional models with limits. As can be seen, maximum α D values are restricted by Power 1 and Power 2 models. Furthermore, the matching of minimum and maximum values of Ct and α D which are obtained by these two models is excellent. However, the results obtained by Power 3 model compared to the previous two models deviate significantly in obtaining very large Ct and α D maximums for 0.05 ≤ µ ≤ 0.27 and 0.04 ≤ µ ≤0.36, respectively. Minimum values of these parameters are very similar for all three Power models, while in comparison to them, the other three models give lower CTmin . Regarding CTmax , for an advance ratio less than 0.2, the limit is provided by Power 1 model. In contrast, the emergence of dynamic stalling represents the main limiting factor for higher advance ratios. According to the above, the results obtained using Dynamic stall and Power 1 limits served as the basis for establishing the APR boundaries, shown in Figure 3 (middle). Because the very simple longitudinal force equilibrium model is used as an APR solver, a more expansive APR zone is established, as shown in Figure 3 (right). Grid density is verified by the control group, with about 97 million points. As presented in Figure S1, there are no significant differences between the working and control groups. The main goal of LFEM is to determine the APR zone, and in this regard, a dynamic stall (stall flutter) limit is fundamental, especially for an advance ratio higher than 0.2. Unlike fixed-wing aircraft, rotorcraft normally fly in mild and medium stall flutter conditions because the blades very quickly leave the retreating side of the rotor. The main consequence of stall flutter is a pitch link vibratory load often used for practical stall boundary determination. As already mentioned, a modified Harris criterion (MHC) is used in this study. Figure 4a shows MHC obtained by NCP 1 execution compared to standard Harris criteria.

Aerospace Aerospace2023, 2023,10, 10,238 x FOR PEER REVIEW

11 of

of 31 11 30

Figure 3. Graphical representation of the APR boundaries determination: NCP 1 results (left), Power 1 and Dynamic stall limits (middle), APR with safety margin (right).

The data from [33] are used and compared with the MHC line adapted to the given helicopter model and test flight conditions for validation. As shown in Figure 4b, there is a relatively good matching of MHC with the pitch link vibratory limit. The advantage of such a modification on the Harris criterion is that it allows variability of the maximum lift coefficient depending on the local (representative) Reynolds andNCP Machresults numbers the Figure of the

APRAPR

boundaries determination: (left),on Power Figure 3.3.Graphical Graphicalrepresentation representation of the boundaries determination: 1NCP

1 results

(left), retreating blade, which is the place of stall flutter occurrence. 1Power and Dynamic stall limits APR with safety (right). (right).

1 and Dynamic

stall(middle), limits (middle),

APR

withmargin safety margin The data from [33] are used and compared with the MHC line adapted to the given helicopter model and test flight conditions for validation. As shown in Figure 4b, there is a relatively good matching of MHC with the pitch link vibratory limit. The advantage of such a modification on the Harris criterion is that it allows variability of the maximum lift coefficient depending on the local (representative) Reynolds and Mach numbers on the retreating blade, which is the place of stall flutter occurrence.

Figure 4. (a) Modified Harris criterion for Dynamic stall limit obtained by WG; (b) Dynamic stall boundaries in in accordance accordance to to Lynx Lynx XZ170 XZ170 flight flighttest testdata. data. boundaries

4.2. GIF

TheNumerical data fromAnalysis. [33] are used and compared with the MHC line adapted to the given helicopter model and flight conditions validation. As shownAs in Figure 4b, there is The results of thetest

NCP

2 execution

are for given in this subsection. previously stated, a1344 relatively good matching of

MHC

with the pitch link vibratory limit. The advantage of cycles were conducted, and each cycle had 𝑁 . 4,375,561 points. Given that one such modification on the the equation Harris criterion that it allows variability of the lift cycle arequires solving twice, is

GIF

has been solved almost 11.8maximum billion times coefficient depending on the local (representative) Reynolds and Mach numbers on the for regular benchmark solutions. to thestall large number of by obtained only stall the Figure

4. (a)and

Modified Harris criterion forDue Dynamic limit obtained WG; (b)data, Dynamic retreating blade,results which are is the place of and stalldiscussed. flutter occurrence. most important presented boundaries in accordance to Lynx XZ170 flight test data.

4.2. GIF Numerical Analysis

4.2.1. Percentage Failed Convergence (PoF)

4.2. GIF

NumericalofAnalysis. The results of the NCP 2 execution are given in this subsection. As® previously stated, Iteration and failed convergence counters are built into the Julia code for allstated, used The results the NCP 2and execution are given this subsection. As previously

1344 cycles

wereof conducted, each cycle had Nin tot. = 4, 375, 561 points. Given that one numerical methods. The number of iterations greater than 1000 is empirically chosen to 4,375,561 points. Giventimes that one

1344 requires

cycles were conducted, and each cycle 𝑁 . solved cycle solving the equation twice, GIF had has been almost

11.8 billion

for stop the solver and to indicate the occurrence of failed convergence. Firstly, numerical cycle requires solving the equation twice, hasnumber been solved almostdata, 11.8only billion regular and benchmark solutions. Due to theGIF large of obtained the times most for regular and benchmark solutions. Due to the large number of obtained data, only the important results are presented and discussed. most important results are presented and discussed.

4.2.1. Percentage of Failed Convergence (PoF)

4.2.1.Iteration Percentage Failed Convergence (PoF) are built into the Julia® code for all used andoffailed convergence counters

Iteration and failed convergence counters are than built 1000 into is the Julia® code for all numerical methods. The number of iterations greater empirically chosen to used stop numerical methods. The the number of iterations greater than 1000 is empirically to the solver and to indicate occurrence of failed convergence. Firstly, numerical chosen solutions stop the solver and to indicate the occurrence of failed convergence. Firstly, numerical

of Equation (6) are examined. According to the results shown in Table 7, the presence of failed convergence correlates with stopping criteria (required accuracy). Therefore, eliminating failed convergence presence by reducing the required accuracy is possible with all methods except the BWR II. In general, a small number of cases of failed convergence (NoF) concerning the total number of performed calculations Ntot. . The percentage of failed convergence (PoF) is calculated as PoF = 100

NoF [%], Ntot.

(28)

and its distribution over the advance ratio is shown in Figure 5. It can be seen that PoF increases with the advance ratio and reaches a value of around 50 percent for the Chun and Cordero methods. In addition, an initial value does not generally significantly influence the PoF. Finally, it is essential to note that using arbitrary precision in the Sharma method to calculate benchmark solutions has completely eradicated convergence failures. Since the double-precision floating-point arithmetic is set by default for real numbers in 64-bit system architecture (Float64), the maximum number of significant digits is about 15 to 17. Therefore, the accumulation of computational error is the probable cause of these failures. Table 7. Percentage of Failed Convergence, PoF (%), ε = 5 × 10−n , Equation (6). λ0

n

One-Point Methods

Two-Point Methods

Three-Point Methods

Halley

Chebyshev

Kiss

E5

Ostrowski

Chun

Cordero

Jarrat

Sharma

BWR

BWR II

KT Diff

λh

16 14 12 10 8 6 4

2.16 0 0 0 0 0 0

2.15 0 0 0 0 0 0

2.15 0 0 0 0 0 0

2.15 0 0 0 0 0 0

0.76

1 × 10−4

0 0 0 0 0

12.39

6 × 10−3

0 0 0 0 0

12.84

6 × 10−3

0 0 0 0 0

2.15 0 0 0 0 0 0

1.62 0.015

2 × 10−4

0 0 0 0

2.68 0 0 0 0 0 0

1.28 0.037 0.024 0.017

1.2 × 10−4

5.9 × 10−5

9.8 × 10−6

1.75 0.04 0.002 0 0 0 0

λwj

16 14 12 10 8 6 4

2.12 0 0 0 0 0 0

2.08 0 0 0 0 0 0

2.10 0 0 0 0 0 0

2.11 0 0 0 0 0 0

0.63

5 × 10−5

0 0 0 0 0

13.69

5 × 10−3

0 0 0 0 0

14.13

5 × 10−3

0 0 0 0 0

2.1 0 0 0 0 0 0

1.68 0.012

5 × 10−5

0 0 0 0

2.71 0 0 0 0 0 0

1.20 0.05 0.024 0.07 0.0015

7.3 × 10−6

3 × 10−6

2.37 0.032 0.002

2 × 10−5

0 0 0

16 2.11 2.11 2.11 14 0 0 0 12 0 0 0 λm 10 0 0 0 8 0 0 0 Aerospace 2023, 10, x

FOR

PEER

REVIEW

6 0 0 0 4 0 0 0

2.11 0 0 0 0 0 0

0.64

5 × 10−5

0 0 0 0 0

13.60

4 × 10−3

0 0 0 0 0

13.95

3 × 10−3

0 0 0 0 0

2.1 0 0 0 0 0 0

1.77 0.008

7 × 10−5

0 0 0 0

2.69 0 0 0 0 0 0

1.21 0.04 0.032 0.027 0.0223

1.3 × 10−4

10−5

2.55 0.033 0.001

2 × 10−5

0 0

Figure 5. advance ratio, ratio, Equation Equation (6), (6), 𝜀ε = 55 × 10 10−16 (a) One-point

5. PoF distribution over advance

One‐point methods; (b) Two-point Two‐pointmethods; methods;(c) (c)Three-point Three‐pointmethods. methods. (b)

To confirm(3)this case of the largest convergence percentage offailures, failed convergence Equation hasassumption, much betterthe properties regarding as shown in from

7 (PoF =methods

14.13%, are ε =not

5 ×presented

10−16 , λoin=the λwjtable , Cordero method) was taken TableTable

8. One‐point

because no failures wereand de‐ ® recalculated arithmetic precision. ArbNumerics tected for 𝜀 with

1 appreciably

10 . Whenincreased

it comes to multipoint methods,The there are certainlyJulia cases package was used with 200 As a. result, the convergence was completely Equations

(2) and (5) express

excellent of convergence failures, butsignificant only if 𝜀 digits.

1 10

properties: no cases of failed convergence were found for declared precision. Table 8. Percentage of Failed Convergence, PoF (%), 𝜀

Two‐Point Methods

5

10

, Equation (3).

Three‐Point Methods

Figure 5. PoF distribution over advance ratio, Equation (6), 𝜀 (b) Two‐point methods; (c) Three‐point methods.

5

10

(a) One‐point methods;

successful = much 0%). The same result was achievedconvergence using the BWR II method, where Equation (PoF

(3) has

better properties regarding failures, as shown in the failures were present for all stopping criteria. Table 8. One‐point methods are not presented in the table because no failures were de‐ much itbetter properties regarding convergence shown tected forEquation 𝜀 1 (3)

10 has

. When comes to multipoint methods, there are failures, certainlyas cases in Table 8. One-point methods are not presented in the table because no failures were of convergence failures, but only if 𝜀 1 10 . Equations (2) and (5) express excellent −17 . When it comes to multipoint methods, there are certainly cases detected for ε < 1 × 10 properties: no cases of failed convergence were found for declared precision. of convergence failures, but only if ε ≥ 1 × 10−16 . Equations (2) and (5) express excellent properties: no cases of failed convergence were declared(3). precision. Table

8. Percentage

of Failed Convergence, PoF (%), 𝜀 found

5 10 for

, Equation −n , EquationMethods Methods PoF (%), ε = 5 × 10 Three‐Point Table 8. PercentageTwo‐Point of Failed Convergence, (3).

𝝀𝒐 λ0 λh λwj λmd

n 16 14 16 14 16 14

𝜆 Ostr. 0.003

𝜆0

0.006

𝜆 0

0.006 0

n

Ostr. Chun

16 Two-Point

0.003 Methods

0.095 Chun Cord. 14 0 0 0.095 0.628 16 0.006 0.118 0 0 14 0 0 0.118 0.771 16 0.006 0.087 0 0

14 0.087

0 0 0.571 0

0

Cord. 0.628 Jarr. 0 0 0.771 0 0 0 0.571 0 00 0

Jarr. Sh.Sh. BWR BWR II

KT

Methods 0

0 Three-Point

0 0.0015 0.0015 Sh.Sh.

BWR

BWR II

KT

0 0 0 0 0 0 00 0.0015 0.0015 0 0 0.0021 0.001 0 0 0 0 0 0 0 0 0 0 0 0.0021 0.001 0 0 0.0028 0.00150 0 00 0 0 0 0 00.0015 0 00 0.0028 0

0

0

0

4.2.2. Number of Iterations (ITNumb)

4.2.2. of Iterations (ITNumb) As inNumber the previous subsection, Equation (6) was analyzed first. Figure 6 shows the number As of iterations of one‐point methods for an (6) initial 𝜆 𝜆 and stopping cri‐ the in the previous subsection, Equation wasguess analyzed first. Figure

6 shows

terianumber 𝜀 5 of10 . Theofmaximum is relatively and reaches iterations one-point number methodsof foriterations an initial guess λ0 = λhlarge and stopping criteria

−14E5

overε 150 method and over

15 iterations

for is the Halley and The 150 = 5 for × 10the . The maximum number of iterations relatively largeKiss andmethods. reaches over minimum of iterations ranges from for twothe to Halley three. The 𝐼𝑇𝑁𝑢𝑚𝑏 is minimum about for the number E5 method and over 15 iterations andaverage Kiss methods. The fournumber iterations for all one‐point methods and is represented by dashed lines on two dia‐ of iterations ranges from two to three. The average ITNumb is about four iterations grams

6. methods and is represented by dashed lines on two diagrams in Figure 6.

for in allFigure one-point

Figure 6. ITNumb distribution over advance ratio, Equation (6), λ0 = λh , ε = 5 × 10−14 , one-point methods: (a) Halley and Kiss methods; (b) Chebyshev and E5 methods.

The total number of cases in which the required number of iterations is greater than

10 ranges from 0.02 to 4.15 percent, and three to five iterations are sufficient in 87.5 to 97.5%

of cases, depending on the chosen one-point method. The results (ITNumb vs. advance ratio) for other stopping criteria (ε = 5 × 10−n , n = 4, 6, 8, 10, 12, 16) and for multipoint methods (where λ0 = λh ) are very similar to the above (Figure 6). The required number of iterations can be drastically reduced if λwj or λmd are used instead of λh , as shown in Figure 7. The stopping criteria are essential because a small ε (high-required accuracy) causes a larger number of iterations. As can be seen in Figure S2, only one iteration in about 75% and up to two iterations in 99.8% of cases is sufficient if the Sharma method is used in conjunction with λ0 = λmd , where stopping criteria are set on ε = 5 × 10−14 .

methods (where 𝜆 𝜆 ) are very similar to the above (Figure 6). are used The required number of iterations can be drastically reduced if 𝜆 or 𝜆 instead of 𝜆 , as shown in Figure 7. The stopping criteria are essential because a small 𝜀 (high‐required accuracy) causes a larger number of iterations. As can be seen in Figure S2, only one iteration in about 75% and up to two iterations in 99.8% of cases is sufficient if the Sharma method is used in conjunction with 𝜆 𝜆 , where stopping criteria are set on 𝜀 5 10 .

Figure 7. Average ITNumb distribution of Equation (6): (a) One-point methods; (b) Two-point methods; (c) Three-point methods.

Even better results are obtained if the irrational form is solved. Namely, under the same conditions, Equation (5) is solved with the Sharma method in two or fewer iterations in all cases. Figures S3–S8 show an average number of iterations and their percentage distribution of remaining GIF forms. As can be seen, the use of improved, i.e., modified initial value (λmd ) significantly reduces the required number of iterations, regardless of the stopping criteria, the type of method used, and the form of the Equation being solved.

4.2.3. Relative Errors (Eo , Et )

Let λ HPS be the benchmark solution of GIF obtained using high precision with 200 significant digits. Then, Et is a relative error with respect to the benchmark solution Et =

λ f − λ HPS , λ HPS

(29)

where λ f is a final numerical solution reached using a standard computational precision of up to approximately 16 significant digits. As a reminder, the relative input error that was used as a stopping criterion in the iterative cycle is marked with ε and defined by Equation (8). It represents the maximum acceptable relative distance between two adjacent solutions in the iterative sequence. The relative output error is defined as Eo =

λ f − λ f −i , λf

(30)

where λ f −i is a solution from the previous iteration (i = 1) or the previous point of the same iteration step (i ∈ {1/3, 1/2, 2/3}, applicable only to multipoint methods). According to the above, all analyzed numerical methods come to the solution in two ways:

• •

• • • •

Eo < ε—an iterative process is accomplished, convergence is achieved, the input criterion is fulfilled, and the output solution is λ f ; Eo ≥ ε—the stopping criterion is not activated, but the iterative process is interrupted due to the failed convergence or due to an inappropriate solution (NaN, infinite, or zero). Then, λ f −i becomes a final numerical solution λ f . The first case (Eo < ε) can be seen in the following diagrams in Figure 8; Equation (6), all methods, all initial values; Equations (2), (3), and (5), one-point methods, all initial values; Equation (3), two-point methods, all initial values.

Anomalies can be observed on other charts (Eo ≥ ε). The occurrence of failed convergence has already been discussed, and a comparative analysis of data shown in Tables 7 and 8 with the charts in Figure 8 leads to the two groups of anomalies whose cause is different from convergence failure. The first group refers to Equation (6) solved by:

• • •

Chun and KT Diff, which gives Eo max ≈ 1 for all stopping criteria where λ0 = λh ;   KT Diff with λ0 ∈ λwj , λmd , which gives Eo max ≈ 2.5 × 10−8 ε < 10−8 ;  Sharma with λ0 ∈ λh , λwj , λmd , which gives Eo max ≈ 2.5 × 10−8 for ε < 10−8 .

The  second group includes Equations (3) and (5) solved by the Sharma method with λ0 ∈ λh , λwj , λmd , which also give Eo max > ε, as shown in Figure 8. The most common cause of anomalies is NaN, which does not necessarily represent an unfavorable outcome. For example, when finding a benchmark solution, NaN occurs in 100% of cases, but λ f −i is already a sufficiently high-quality solution after 1 + i iterations, and in that case, Eo max ≈ 5 × 10−51 . The total maximum varies with the change in advance ratio, as shown in Figure 9. The same applies to the spectrum of Eo max values given in Figure 8, while their dependence on the advance ratio is shown in Figures S9–S12.

Aerospace Aerospace 2023, 2023, 10, 10, 238 x FOR PEER REVIEW Aerospace 2023, 10, x FOR PEER REVIEW

of 30 31 1616of

Figure 8. 𝐸 𝑀𝑎𝑥 (maximum relative error between the last two iterations or points) for all GIF Figure relative error between the the last two or points) for all for GIFall forms Figure8.8.E𝐸 𝑀𝑎𝑥(maximum (maximum relative error between last iterations two iterations or points)

GIF

o Max forms solved by: (a) one‐point methods, (b) two‐point methods; (c) three‐point methods. forms solved by: (a) one‐point methods, (b) two‐point methods; (c) three‐point methods. solved by: (a) one-point methods, (b) two-point methods; (c) three-point methods.

Figure 9. Maximum output relative error of the benchmark solution. Figure9.9.Maximum Maximumoutput outputrelative relativeerror errorof ofthe thebenchmark benchmarksolution. solution. Figure

Given that the subject of the study is the examination of numerical methods with Giventhat thatthe thesubject subject study is the examinationnumerical of numerical methods with Given of of thethe study methods with stanstandard accuracy (up to 16–17 digits),is itthe is examination much more of essential to analyze maximum standard accuracy to 16–17 it ismore muchessential more essential to analyze maximum dard accuracy (up to(up 16–17 digits),digits), it is much to analyze maximum relative relative errors compared to the benchmark solution (𝐸 ), which are given in Figures 10 relative errors compared to the benchmark (𝐸 ),are which are in Figures 10 errors compared to the benchmark solutionsolution (Etmax ), which given ingiven Figures

10 and 11.

and 11. Figure 10 shows 𝐸 for all methods and initial values used in solving Equa‐ and 11.10Figure shows 𝐸 methods for alland methods and initial usedEquations in solving(2), Equa‐ Figure shows10 Etmax for all initial values usedvalues in solving (3), tions (2), (3), and (5). As can be seen, the actual minimal accuracy (the maximal error) and As(3), canand be seen, thecan actual minimal (the maximal error) tions(5). (2), (5). As be seen, the accuracy actual minimal accuracy (theEtmaximal error) max is far better 𝐸 is far better compared to the stopping criteria 𝜀 and the maximal output relative compared to the stopping criteria ε and the maximal output relative error

E

. However, 𝐸 is far better compared to the stopping criteria 𝜀 and the maximal output relative omax error 𝐸 . However, when (3) it comes to Equations and is noted 𝐸 highest 𝜀 but when to Equations and (5), is noted(3) that Et (5), ≥ εititbut only that for error it 𝐸 comes . However, when it comes to it Equations (3) and (5), is noted thatthe 𝐸 𝜀 but only for the highest(εrequired (𝜀 5 10the ).lowest Furthermore, lowest accuracy −16 ). Furthermore, required precision = 5 × 10precision accuracythe is detected for the only for the highest required precision (𝜀 5 10 ). Furthermore, the lowest accuracy is detected for the Sharma method used with 𝜆 𝜆 in all three equations, which is is detected for the Sharma method used with 𝜆 𝜆 in all three equations, which is

explained by the small number of iterations and the high order of convergence of the method. for Equation (6). These results are presented separately from Figure 11 shows 𝐸 Figure 10 because they require a more detailed analysis. Namely, all one‐point methods

17 of

31

0.5 regardless of stopping criteria.

The used in conjunction with 𝜆 indicate that 𝐸 same applies to all two‐point methods except the Chun and the three‐point Sharma and BWR methods. With omitted methods, the results are even worse (extremely high values Sharma used λ0explained. = λmd in all three equations, explained by diagrams the small of 𝐸 method ), but they arewith easily It is enough to lookwhich at the is corresponding number andanomalies the high order convergence of thebeen method. in Figureof8iterations and see that (𝐸 of ≫ 𝜀) have already detected there.

Figure 10. 𝐸 𝑀𝑎𝑥 (maximum relative error related to the benchmark solution) for all GIF forms Figure 10. Et Max (maximum relative error related to the benchmark solution) for all GIF forms solved by: (a) one‐point methods, (b) two‐point methods; (c) three‐point methods. solved by: (a) one-point methods, (b) two-point methods; (c) three-point methods.

Additionally, BWR II methods have a high maximal actual relative error (4 10 Figure 11 shows Etmax for Equation (6). These results are presented separately from 𝐸

5 10 ) for all initial values, but it is clear that this phenomenon is a consequence

Figure 10 because they require a more detailed analysis. Namely, all one-point methods of failed convergence. Completely satisfied criteria with excellent output relative used in conjunction with λh indicate that Estopping tmax ≈ 0.5 regardless of stopping criteria. The error on the one side and disappointingly large actual relative error on the other can only same applies to all two-point methods except the Chun and the three-point Sharma and be explained only by a successful convergence to superfluous roots, which is the BWR methods. With omitted methods, the results are even worse (extremely highcase. values of Etmax ), but they are easily explained. It is enough to look at the corresponding diagrams in Figure 8 and see that anomalies (Eo max  ε) have already been detected there. Additionally, BWR II methods have a high maximal actual relative error (4 × 102 < Eo max < 5 × 108 ) for all initial values, but it is clear that this phenomenon is a consequence of failed convergence. Completely satisfied stopping criteria with excellent output relative error on the one side and disappointingly large actual relative error on the other can only be explained only by a successful convergence to superfluous roots, which is the case.

Aerospace 2023, 10, 238 Aerospace Aerospace2023, 2023,10, 10,xxFOR

FORPEER

PEERREVIEW

REVIEW

18

18 ofof 30

30

Figure 11. 𝑀𝑎𝑥 for all methods and initial values—Equation (6). Figure Figure11.

11.E𝐸𝐸

𝑀𝑎𝑥for forall allmethods methodsand andinitial initialvalues—Equation values—Equation(6). (6). t Max

superfluous roots are always at distance Further 0.0035 ∆λ | ≥ 0.0035 |∆𝜆| Furtheranalysis analysisrevealed revealedsuperfluous superfluousroots rootsare arealways alwaysat ataaadistance distance||∆𝜆| 0.0035 This fact enabled the calculation of the probability of conver‐ from the physical solution. calculation of the probability of converfrom the physical solution. This fact enabled the calculation of the probability of conver‐ “wrong” solution during solving Equation (6). gence gencetowards towardsthe the“wrong” “wrong”solution solutionduring duringsolving solvingEquation Equation(6). (6).Figure Figure12 12shows showsthat that probability increases with the advance ratio and reaches over 60 percent. It be probability with thethe advance ratioratio and and reaches over over

60 percent.

It should be noted probabilityincreases increases with advance reaches

60 percent.

It should should be noted again that this applies only to 𝜆 𝜆 , which is essential considering that using 𝜆 again that this applies only to λ = λ , which is essential considering that using λ as an 0 h h noted again that this applies only to 𝜆 𝜆 , which is essential considering that using 𝜆 as isisstandard practice. Equation also aarational of initial value isvalue standard practice. Equation

(3) also(3)

represents a rational form ofform the inflow asan aninitial initialvalue standard practice. Equation (3) alsorepresents represents rational form ofthe the inflow equation, but the “superfluous roots” problem was not detected. equation, but the “superfluous roots” problem was not detected. inflow equation, but the “superfluous roots” problem was not detected.

Figure 12. Percentage of convergence towards superfluous roots for all examined methods. Figure12. 12.Percentage Percentageof ofconvergence convergencetowards towardssuperfluous superfluousroots rootsfor forall allexamined examinedmethods. methods. Figure

4.2.4. One Iteration Solution 4.2.4. 4.2.4.One OneIteration IterationSolution Solution

A

solution obtained in just one iteration can be considered noniterative practi‐ AAsolution can be considered noniterative inin the practical solutionobtained obtainedin injust justone oneiteration iteration can be considered noniterative inthe the practi‐ cal because stopping criteria used. Figures and 14 show maximal relative sense because stopping criteria areare notnot used. Figures 1313 and calsense sense because stopping criteria are not used. Figures 13 and14 14show showmaximal maximalrelative relative error depending on the advanced ratio errorwith withrespect respectto tothe thebenchmark benchmark solution, solution, depending depending on on the the advanced advanced ratio ratio 𝜇µ𝜇 for for (5), (6) and (2), (3), respectively. After analyzing these charts, several Equations (5), (6) and (2), (3), respectively. After analyzing these charts, several Equations (5), (6) and (2), (3), respectively. After analyzing these charts, severalfindings findings can canbe behighlighted: highlighted:

•

• • •  • 

Convergence towards an extraneous root may occur value Convergence may occur ififλ used as as an initial value to Convergencetowards towardsan anextraneous extraneousroot root may occur ifh𝜆𝜆is is isused used asan aninitial initial value to solve Equation (6), regardless of chosen method; solve Equation (6), regardless of chosen method; to solve Equation (6), regardless of chosen method; The 𝜆𝜆o = 𝜆λ𝜆h has Thesolutions solutionsof ofEquation Equation(3) (3)obtained obtainedin inone oneiteration iterationwith with λ hasno nopractical practical significance regardless of the chosen method due to the very large maximal significance regardless of the chosen method due to the very large maximalrelative relative error concerning the benchmark solution (𝐸 errorconcerning concerningthe thebenchmark benchmarksolution solution(E (𝐸tmax ););); Failed

IIIIII

method and λmd value areare used to convergence may occur the

BWR

method and 𝜆𝜆 initial initial value used Failedconvergence convergencemay mayoccur occurifififthe theBWR

BWR

method and initial value are used solve Equation (6) with standard arithmetic precision; to solve Equation (6) with standard arithmetic precision; to solve Equation (6) with standard arithmetic precision; Some anomalies in E𝐸tmax magnitude may appear if multipoint methods and λmd 𝜆are magnitude Some anomalies anomalies in in 𝐸 magnitude may may appear appear ifif multipoint multipoint methods methods and and 𝜆 used together to solve all forms of

GIF

except Equation (2), where no anomalies; are all forms of

GIF

except Equation (2), where no anomalies; areused usedtogether togetherto to−solve solve all forms of

GIF

except Equation (2), where no anomalies; Et 𝑀𝑎𝑥| Max | < 5.3 × 10

10 16 is certainly reached

over the entire range of advance ratio if ||𝐸 |𝐸 𝑀𝑎𝑥| 5.3

5.3 10 isiscertainly

certainlyreached reachedover overthe theentire entirerange rangeof ofadvance advanceratio ratioifif the Sharma or KT KT Diff Diff method method is is used used in in conjunction conjunctionwith with 𝜆λmd to solve Equation (2) the Sharma or to solve Equation the Sharma or KT Diff method is used in conjunction with 𝜆 to solve Equation(2) (2) which applies to standard arithmetic precision architecture. Figure

S13

shows that the which applies to standard arithmetic precision architecture. Figure

S13

shows that which applies to standard arithmetic precision architecture. Figure S13 shows that

Aerospace 2023, 10, x FOR PEER REVIEW

• 

above conclusion applies only to the APR zone, i.e., that the numerical methods have the above conclusion applies only to the APR zone, i.e., that the numerical methods worse performance outside the above zone; have worse performance outside the above zone; E5 one-point method used with λo = λmd to solve Equations (2) and (3) provides high 𝜆 to solve Equations (2) and (3) provides E5 one‐point method used with 𝜆 accuracy of obtained solution where | Et Max | < 9.7 × 10−10 for low advance ratios |𝐸 high accuracy of obtained solution where 𝑀𝑎𝑥| 9.7 10 for low advance and | Et Max | < 2.5 × 10−14 for µ > 0.23. ratios and |𝐸 𝑀𝑎𝑥| 2.5 10 for 𝜇 0.23.

Figure 13. 𝐸 𝑀𝑎𝑥 (maximum relative error related to the benchmark solution) for rational GIF Figure 13. Et Max (maximum relative error related to the benchmark solution) for rational GIF forms forms solved by: (a) one‐point methods, (b) two‐point methods; (c) three‐point methods. solved by: (a) one-point methods, (b) two-point methods; (c) three-point methods.

Thepreviously previouslymentioned mentionedfailed failedconvergence, convergence, which occurs if the

BWR

II/𝜆 pair pair The which occurs if the

BWR

II/λ is md is engaged in solving Equation (6) in standard precision, can be eliminated by using arbi‐ engaged in solving Equation (6) in standard precision, can be eliminated by using arbitrary trary precision withsignificant

200 significant

digits, as shown in Figure Thesame sameapplies appliesto tothe the precision with 200 digits, as shown in Figure 15.15. The anomalies in 𝐸 magnitude that arise when some analyzed multipoint methods are anomalies in Etmax magnitude that arise when some analyzed multipoint methods are used with with λ𝜆o = 𝜆λmd for for numerical numerical solving shown in used solving of of all allGIF GIFforms formsexcept exceptEquation Equation(2), (2), shown Figure 15. Furthermore, it can be concluded that the accumulation of computational error in Figure 15. Furthermore, it can be concluded that the accumulation of computational at standard precision affects the final accuracy of the obtained solution even after one it‐ error at standard precision affects the final accuracy of the obtained solution even after one eration. The situation is even more unfavorable if multipoint methods are used in several iteration. The situation is even more unfavorable if multipoint methods are used in several iterations, which which could could already already be be seen seen in in the theprevious previoussubsection. subsection. The The reason reason lies lies in in the the iterations, large difference difference between E𝐸o and andE𝐸 after thefirst firstiteration iteration(Figure (Figure16a), 16a),which whichpractically practically large the t after unnecessarilyactivates activatesthe thesecond secondone. one. As As aa result, result, calculation calculation error error accumulates, accumulates,and andthe the unnecessarily finalsolution solutionhas hasless lessprecision precisionthan thanone oneobtained obtainedafter afterthe thefirst firstiteration. iteration. final

Aerospace 2023, 2023, 10, 10, 238 x FOR PEER REVIEW Aerospace

2020of of 30 31

Figure

14. E

𝐸 𝑀𝑎𝑥 (maximum relative error related to the benchmark solution) for irrational GIF Figure 14. t Max (maximum relative error related to the benchmark solution) for irrational GIF forms solved by: (a) one‐point methods, (b) two‐point methods; (c) three‐point methods.. forms solved by: (a) one-point methods, (b) two-point methods; (c) three-point methods.

For Sharma method For example, example, the the numerical numerical solving solving of of Equation Equation (5)

(5) by

by the the Sharma method with with an an

8 always

initial value value λ𝜆o = λ𝜆h and andstopping stopping criteria always ITNumb criteria ε ≤𝜀5 ×510−10 givesgives ITNumb = 1.67 1.67 and and occurrence of NaN in 100% of cases, the stopping criteria are not occurrence of NaN in 100% of cases, which which means means that thethat stopping criteria are not fulfilled, fulfilled, as is seen in Figure 8. as is seen in Figure 8.

Figure Figure 15.

15. The

The effects effects of of an an arbitrary arbitrary precision precision on on maximum maximum relative relative error error reduction reduction for for three‐point three-point . methods used in solving of Equations (3), (5) and (6) with an initial value 𝜆 methods used in solving of Equations (3), (5) and (6) with an initial value λ . md

Computational during thethe firstfirst iteration, especially if theifhigh Computational error erroraccumulates accumulateseven even during iteration, especially the order of convergence methods is used with the most precise initial value (𝜆 ). In that high order of convergence methods is used with the most precise initial value (λmd ). In regard, Figure 16b gives a comparative view ofview the maximal relative relative errors after theafter first that regard, Figure 16b gives a comparative of the maximal errors

Aerospace 2023, 10,10, x FOR

PEER

REVIEW

Aerospace 2023, x FOR

PEER

REVIEW

Aerospace 2023, 10, 238

21 3030 2121of of of 31

iteration and 2/3 ofofthe first duo ininsolving Equations the first iteration and after 2/3 ofiteration the first for iteration for Sharma/λ used in solving iteration andafter after 2/3 the first iteration forSharma/𝜆 Sharma/𝜆 duoused used solving Equations md duo (3), can bebe seen, better precision Equations (3),(6). (5), and (6). As can be seen, betterisprecision isafter obtained after thesteps first two (3),(5), (5),and and (6).AsAs can seen, better precision isobtained obtained afterthe thefirst firsttwo two stepscom‐ com‐ pared toto the whole iteration. steps compared to the whole iteration. pared the whole iteration.

Figure

16. (a)(a)

The example of discrepancy between output relative error and actual relative error Figure The example discrepancy between output relative error and actual relative error Figure 16.16. (a) The example of of discrepancy between output relative error and actual relative error after one iteration (three‐point methods, 𝜆 𝜆 , Equation (2)); (b) Three‐point Sharma–Sharma after one iteration (three‐point methods, 𝜆 𝜆 , Equation (2)); (b) Three‐point Sharma–Sharma after one iteration (three-point methods, λo = λmd , Equation (2)); (b) Three-point Sharma–Sharma numerical method—the error accumulation inin the third step ofof the first iteration. numerical method—the error accumulation the third step the first iteration. numerical method—the error accumulation in the third step of the first iteration.

4.2.5. Newton–Raphson Method 4.2.5. Newton–Raphson Method 4.2.5. Newton–Raphson Method The

NR

method, as the most commonly used tool for solving nonlinear equations, The

NR

method, most commonly used tool for solving nonlinear equations, The

NR

method, as as thethe most commonly used tool for solving nonlinear equations, isis is analyzed separately due to its outstanding historical significance. It has been applied toto analyzed separately due outstanding historical significance. has been applied analyzed separately due to to itsits outstanding historical significance. It It has been applied to solve allall

GIF

forms with three initial values 𝜆,md ), ), with oror without the use ofof the solve

GIF

forms with three initial values 𝜆 ), with without the use the solve all

GIF

forms with three initial values (λ(𝜆h(𝜆 , , λ𝜆,wj𝜆, , λ with or without the use of the convergence acceleration factor (𝑓 0.5) recommended by

W.

Johnson in [21]. Regarding convergence acceleration factor ( f = 0.5) recommended by

W.

Johnson in [21]. Regarding convergence acceleration factor (𝑓 0.5) recommended by W. Johnson in [21]. Regarding failed convergence, the

NR

method behaves similarly toto the discussed one‐ failed convergence, the

NR

method behaves similarly to the previously discussed one-point failed convergence, the

NR

method behaves similarly thepreviously previously discussed one‐ point methods: it only occurs with Equation (6) in about two percent of examined cases. methods: it only occurs Equation

(6) in about

percent of examined cases. The point methods: it only with occurs with Equation

(6) intwo

about two percent of examined cases. the ofofconvergence failures bybyabout 7.57.5times, asas The relaxation factorfactor reduces the number of convergence failures by about 7.5 times, as seen thenumber number convergence failures about times, Therelaxation relaxation factorreduces reduces seen ininTable other side, using the in‐in‐ in Table

S1.

OnS1. the other side, using relaxation factor factor f factor = 0.5𝑓significantly increases seen Table S1.On Onthe the other side,the using therelaxation relaxation 𝑓 0.5 0.5significantly significantly creases the number ofof iterations, which is is presented inin Figure 17.17. The maximum values ofof the number ofnumber iterations, which is presented in Figure 17. The maximum values of the creases the iterations, which presented Figure The maximum values the required number of iterations are higher than the average and are shown in Figure required number of iterations are higher than the average and are shown in Figure

S14.

the required number of iterations are higher than the average and are shown in Figure

S14.

The maximum number ofof iterations the

NR

method is is at the level ofof examined one‐ The maximum number of iterations of theof

NR

method is at the level of examined one-point

S14.

The maximum number iterations of the

NR

method at the level examined one‐ methods or evenorslightly higher. At the At same time, the introduction of the relaxation factor point methods even slightly higher. the same time, the introduction ofof the relaxation point methods or even slightly higher. At the same time, the introduction the relaxation significantly increases the number of required iterations. It does It not apply toapply the case ofthe factor increases the ofofrequired iterations. totothe factorsignificantly significantly increases thenumber number required iterations. Itdoes doesnot not apply solving Equation (6) with λ , where the maximum number of iterations is large for all case of solving Equation (6) with 𝜆 , where the maximum number of iterations is large h case of solving Equation (6) with 𝜆 , where the maximum number of iterations is large methods. for allall methods. for methods.

Figure 17.17. Average number iterations forfor

GIF

forms with 3by initial values: One‐point methods; Figure Average number of iterations

4 GIF

forms with

3 initial

values: One‐point methods; Figure 17. Average number ofofiterations for

44GIF

forms solved Newton–Raphson method

(NR)

Newton–Raphson method

(NR)

and Newton–Raphson method with relaxation factor (NRf). Newton–Raphson method

(NR)

and Newton–Raphson method with relaxation factor (NRf). and Newton–Raphson method with relaxation factor (NRf).

Figure 1818 shows the change inin onon the stopping criteria 𝜀 𝜀for allall

GIF

Figure shows the change depending the stopping criteria

GIF

Figure 18 shows the change in𝐸 𝐸 Et Maxdepending depending on the stopping criteria εfor for all forms. As in previous cases, convergence towards extraneous roots is noticeable for Equa‐ forms. AsAs in previous cases, convergence towards extraneous roots is noticeable for Equa‐

GIF

forms. in previous cases, convergence towards extraneous roots is noticeable for tion value 𝜆 𝜆. The positive influence ofofthe factor 𝑓 𝑓 f0.5 ininin‐ Equation (6) and initial value λ positive influence of the relaxation factor =0.5 0.5 inin‐ tion(6)(6)and andinitial initial value . The positive influence therelaxation relaxation factor h . The creasing the accuracy ofof the numerical solution is is not detected. The relative error is is higher creasing the accuracy the numerical solution not detected. The relative error higher

Aerospace2023, 2023,10, 10,xxFOR

FORPEER

PEERREVIEW

REVIEW

Aerospace

22 of

of 30 30 22

increasing the accuracy of the numerical solution is not detected. The relative error is higher or orat atleast leastthe thesame sameifififthe thementioned mentionedfactor factoris isused. used.In Ingeneral, general,the therelative relativeaccuracy accuracyof ofthe the or at least the same the mentioned factor is used. In general, the relative accuracy of the numerical solution achieved by the

NR

method is at the level of the previously analyzed numerical solution solution achieved achieved by by the the NR NR method method is is at at the the level level of of the the previously previously analyzed analyzed numerical one-point methods. one‐point methods. one‐point methods.

Figure18. 18.Maximum Maximumrelative relativeerror errorrelated relatedtoto to the benchmark solution (𝐸 for

NR

and NRf meth‐ Figure 18. Maximum relative error related the benchmark solution NRf meth‐ Figure the benchmark solution

(E(𝐸

NRNR

andand NRf methods t Max ) forfor ods in solving: (a) Equations (5) and (6); (b) Equations (2) and (3). ods in solving: (a) Equations

(5) (6);

and(b) (6);Equations (b) Equations

(2) (3).

and (3). in solving: (a) Equations

(5) and

(2) and

Thesolution solutionobtained obtained in one iteration is shown shown in Figure Figure 19. the results results are com‐ com‐ The inin one iteration is shown in Figure

19. If

the are compared The solution obtained one iteration is in 19. IfIfresults the are pared with the diagrams given in Figures 13a and 14a, it can be concluded that the

NR

with diagrams given ingiven Figures 13a and13a 14a,and it can beitconcluded that the NR paredthe with the diagrams in Figures 14a, can be concluded thatmethod the NR method does not perform better than other one‐point methods in case of use in one itera‐ does notdoes perform better than other one-point methodsmethods in case of In method not perform better than other one‐point in use casein ofone use iteration. in one itera‐ tion. In In addition, addition, Figure 19 shows the negative negative influence influence of the the relaxation relaxation factor on the the addition, Figure 19Figure shows19 the negative influence of the relaxation factor on the achieved tion. shows the of factor on achieved accuracy. accuracy. accuracy. achieved

Figure 19. 19.Maximum Maximumrelative relativeerror errorrelated related tothe the benchmark benchmarksolution solution(𝐸 (𝐸 forNR NR andNRf NRfmeth‐ meth‐ Figure Figure

19. Maximum

relative error related totothe benchmark solution (Et Max ) forfor NR andand NRf methods ods obtained in one iteration. ods obtained in iteration. one iteration. obtained in one

4.3.GIF

GIFAnalytical AnalyticalSolution Solution 4.3. 4.3. Solution Accordingto to[18], [18],an ananalytical analyticalsolution solutiongives givesfour fourroots: roots:one onecomplex complexconjugate conjugatepair pair According According to [18], an analytical solution gives four roots: one complex conjugate pair and two real roots. Unlike [18], where only one case (point) was considered, in study, and two real roots. Unlike [18], where only one case (point) was considered, in this study, and two real roots. Unlike [18], where only one case (point) was considered, in this study, closed‐formsolution solution is istested testedover overthe theentire entireAPR APRzone zonein in436,067,402 436,067,402points, points,and andthe the aaaclosed-form closed‐form solution is tested over the entire

APR

zone in 436,067,402 points, and the observationsare areas asfollows: follows: observations observations are as follows:

• • • • • 

The Thefirst firsttwo tworoots rootsform formaaacomplex complexconjugate conjugatepair pairin in100% 100%of oftested testedcases casesand andcan canbe be The first two roots form complex conjugate pair in 100% of tested cases and can be completely rejected; completely rejected; completely rejected; The Thethird thirdand andfourth fourthroots rootsform formaaacomplex complexconjugate conjugatepair pairin in2.8% 2.8%of oftested testedpoints; points; The third and fourth roots form complex conjugate pair in 2.8% of tested points; The third and fourth roots are a NaN in 1.25% of cases; The third and fourth roots are a NaN in 1.25% of cases; The third and fourth roots are a NaN in 1.25% of cases; The Thethird thirdroot rootis isaaanegative negativereal realnumber numberin in49.6% 49.6%of ofcases; cases; The third root is negative real number in 49.6% of cases; If it is not a NaN or complex number, the fourth root is not not aa NaN NaN or or complex complex number, number, the the fourth fourth root rootrepresents representsaaaphysical physicalsolution; solution; IfIf itit is represents physical solution; The analytical solution is found in about 95% of the tested cases. The analytical solution is found in about 95% of the tested cases.

Aerospace 2023, 10, x FOR PEER REVIEW

•

The analytical solution is found in about 95% of the tested cases. The accuracy of the analytical solution is shown in Figure 20. As can be seen, it has a The accuracy of the analytical solution is shown in Figure 20. As can be seen, it has a much larger relative error, minimum and maximum, compared to very precise, previ‐ much larger relative error, minimum and maximum, compared to very precise, previously ously analyzed numerical solutions obtained in only one iteration using the Sharma and analyzed numerical solutions obtained in only one iteration using the Sharma and E5 E5 methods with an initial value 𝜆 𝜆 for solving Equation (2). The above, as well as methods with an initial value λ0 = λmd for solving Equation (2). The above, as well as the the fact that a solution cannot be found in about five percent of cases, leads to the conclu‐ fact that a solution cannot be found in about five percent of cases, leads to the conclusion sion that closed‐form solution does not show sufficient quality in comparison with nu‐ that closed-form solution does not show sufficient quality in comparison with numerical merical methods. methods.

Figure error related to the benchmark solution (𝐸 ) for: Analytical form) solu‐ Figure 20.

20. Relative

Maximum relative error (Etmax ) related to the benchmark solution for(closed analytical (closed tion; Numerical solution obtained in one iteration by high order of convergence methods with initial form) solution and the numerical solutions obtained in one iteration by a high order of convergence 𝜆 an . initial value λ = λ . value 𝜆 with methods 0

md

4.4.

4.4. GIF

GIF Computational Computational Cost Cost In order order to to complete completethe thecomparison comparisonofofmultipoint multipointwith withsingle‐point single-point methods, it In methods, it is is necessary to evaluate computational of applied the applied methods. Furthermore, necessary to evaluate the the computational costcost of the methods. Furthermore, veri‐ verifying whether the multipoint method will achieve its computational efficiency in fying whether the multipoint method will achieve its computational efficiency in practice practice is necessary. In a limited numerical experiment to the rational forms is necessary. In a limited numerical experiment applied to applied the rational forms of GIF, by of GIF, by comparing the time identical calculations sample of over comparing the time required forrequired identicalfor calculations on a sample on of a over

21.4 million

21.4 million different combinations of

GIF

parameters, the comparative advantage of the different combinations of GIF parameters, the comparative advantage of the three‐point three-point methods was Introducing confirmed. Introducing anfour‐point additional method four-point method (order methods was confirmed. an additional (order of conver‐ of convergence equal to 16)that showed thesehave methods have the weakest performance gence equal to 16) showed these that methods the weakest performance of all ana‐ of all methods. analyzed Therefore, methods. three‐point Therefore, three-point methods the optimal choice. lyzed methods represent therepresent optimal choice. In addition, In addition, introducing the relaxation factor in the

NR

method increases the required introducing the relaxation factor in the NR method increases the required calculation time calculationtotime compared to theby standard about

50 percent.

Theand selected compared the standard method about 50method percent.by The selected initial guess stop‐ initial guess and stopping criteria have an impact on the required calculation time, but ping criteria have an impact on the required calculation time, but to a much lesser extent to a much lesser extent than the choice of the method itself. The

KT

Diff method stands than the choice of the method itself. The KT Diff method stands out for its calculation out forThe its calculation speed.are The obtained results in the Table S2 and given mate‐ in the speed. obtained results shown in Table S2 are andshown given in supplementary Supplementary Material (https://github.com/marjanvtsl/GIF-Numerical-Analysis.git, rial (https://github.com/marjanvtsl/GIF‐Numerical‐Analysis.git, accessed on 23 February accessed on 23 February 2023). 2023).

5. Conclusions and Recommendations

5. Conclusions and Recommendations

Symbolic algebra, arbitrary-precision arithmetics, and multipoint numerical methods Symbolic algebra, arbitrary‐precision arithmetics, and multipoint numerical meth‐ are proficient mathematical tools used in engineering practice. They allow old, entrenched ods are proficient mathematical tools used in engineering practice. They allow old, en‐ methods to be replaced by more modern, faster, and efficient ones. The low-fidelity longitutrenched methods to be replaced by more modern, faster, and efficient ones. The low‐ dinal force equilibrium model of a single-rotor helicopter in forward flight is modified (no fidelity longitudinal force equilibrium model of a single‐rotor helicopter in forward flight small angle assumption) and represented by one nonlinear variable equation instead of a is modified (no smallequations, angle assumption) and represented by one nonlinear variable system of nonlinear which significantly simplifies the numerical solution equa‐ procetion instead of a system of nonlinear equations, which significantly simplifies theiterative numer‐ dure. The used algorithm allows changing the variant of the equation during the ical solution procedure. The used algorithm allows varianttrim of the equation cycle. Further practical implementation of the LFEMchanging requires the complete equations to during the iterative cycle. Further practical implementation of the

LFEM

requires com‐ be included in the model. plete Regarding trim equations to be included in the model. the numerical analysis of the GIF solution, the conclusions are as follows: Regarding the numerical analysis of the GIF solution, the conclusions are as follows:

•

•

•

•

• • •

The use of λwj for the numerical solution of the Glauert inflow equation results in a smaller number of iterations and a higher accuracy of the obtained solution compared to the case when the induced ratio in hover λh is used as an initial guess; Modifying the previously mentioned initial value (λmd ) gives an even better result: the smallest average number of iterations and the highest precision of the solution regardless of the used method and stopping criteria; The chosen form of the Glauert inflow equation (rational or irrational, with unknown λ or λi ) directly affects the performance of the numerical solution where the best convergence failure-free performance gives Equation (2); Optimal three-point methods used along with λmd and high accuracy requested often lead to anomalies that significantly degrade the performance of the used method, and in this regard, it is much better to use them in only one iteration procedure for standard precision arithmetic; Multi-precision arithmetic is computationally expensive, but it completely frees the solutions from the accompanying anomalies; Analytical solutions in the standard arithmetic precision environment perform much worse than numerical solutions; The Sharma–Sharma, KT Diff, and optionally E5 methods, used in only one iteration, with λmd as an initial value for induced inflow ratio in solving Equation (2), have all attributes to become methods of choice for solving the GIF for a single-rotor helicopter in steady-level flight conditions below dynamic stall limit.

Supplementary Materials: The following supporting information can be downloaded at: https: //www.mdpi.com/article/10.3390/aerospace10030238/s1, Figure S1: Working Group and Control Group results of No limit model; Figure S2: ITNumb percentage distribution of Equation (6): (a) Onepoint methods Halley and Kiss; (b) One-point methods Chebishev and E5; (c) Two-point methods Ostrowski and Jarrat; (d) Two-point methods Chun and Cordero; (e) Three-point methods Sharma and BWR II; (f) Three-point methods KT Diff and BWR.; Figure S3: Average ITNumb distribution of Equation (5): (a) One-point methods; (b) Two-point methods; (c) Three-point methods; Figure S4: ITNumb percentage distribution of Equation (5): (a) One-point methods; (b) Two-point methods; (c) Three-point methods; Figure S5: Average ITNumb distribution of Equation (3): (a) One-point methods; (b) Two-point methods; (c) Three-point methods; Figure S6:ITNumb percentage distribution of Equation (3): (a) One-point methods Halley and Kiss; (b) One-point methods Chebishev and E5; (c) Two-point methods Ostrowski and Jarrat; (d) Two-point methods Chun and Cordero; (e) Three-point methods.; Figure S7: Average ITNumb distribution of Equation (2): (a) One-point methods; (b) Twopoint methods; (c) Three-point methods; Figure S8:ITNumb percentage distribution of Equation (2): (a) One-point methods; (b) Two-point methods; (c) Three-point methods; Figure S9: The maximum output errors Eo max vs advance ratio for Equation (6): (a) One-point methods; (b) Two-point methods; (c) Three-point methods; Figure S10: The maximum output errors Eo max vs advance ratio for Equation (5): (a) One-point methods; (b) Two-point methods; (c) Three-point methods; Figure S11: The maximum output errors Eo max vs advance ratio for Equation (3): (a) One-point methods; (b) Two-point methods; (c) Three-point methods; Figure S12: The maximum output errors Eo max vs advance ratio for Equation (2): (a) One-point methods; (b) Two-point methods; (c) Three-point methods; Figure S13: Comparative view of Et max for solutions of Equation (2) obtained in one iteration by three-point methods with λmd initial value in the APR zone and out of the APR zone (0.15 ≤ CT ≤ 0.2; 20◦ ≤ ∝D ≤ 35◦ ). Figure S14: Maximum number of iterations for 4 GIF forms with

3 initial values: Newton-Raphson method (NR) and Newton-Raphson method with relaxation factor

(NRf); Table S1: Percentage of failed convergence, PoF (%) generated by Newton-Raphson method used with (NRf) and without (NR) relaxation factor in solving of Equation (6), ε = 5∆10−n . Symbolic Equations (S1)–(S16) represent 16 variants of the LFE, and they are given in the HTML file: Code No.1, which is uploaded along with the article. The complete codes with obtained results developed and used for NCP_1 execution can be downloaded at: https://github.com/marjanvtsl/LFEM.git, accessed on 30 December 2022. The code for NCP_2 with computational cost tests is given at: https://github.com/marjanvtsl/GIF-Numerical-Analysis.git, accessed on 23 February 2023.

Author Contributions: All authors discussed and agreed upon the idea, and made scientific contributions: Conceptualization, M.D. (Marjan Dodic), B.K., M.D. (Mirko Dinulovic) and A.B.; Data curation, M.D. (Marjan Dodic) and B.K.; Formal analysis, M.D. (Marjan Dodic), B.K., B.R., M.D. (Mirko Dinulovic) and A.B.; Investigation, M.D. (Marjan Dodic) and B.K.; Methodology, M.D. (Marjan Dodic), B.K., B.R. and A.B.; Resources, M.D. (Marjan Dodic) and B.K.; Software, M.D. (Marjan Dodic), B.K. and M.D. (Mirko Dinulovic); Supervision, B.K.; Validation, M.D. (Marjan Dodic), B.K. and B.R.; Visualization, M.D. (Marjan Dodic) and B.K.; Writing—original draft, M.D. (Marjan Dodic) and B.K.; Writing—review and editing, M.D. (Marjan Dodic) and B.K. All authors have read and agreed to the published version of the manuscript. Funding: This research received no external funding. Data Availability Statement: Not applicable. Acknowledgments: This research was supported by the University of Defence in Belgrade. Conflicts of Interest: The authors declare no conflict of interest.

Nomenclature a amin amax al a l0 as a1s b bmin bmax b1s c cd cdTR fe

rotor tip speed statistical factor minimum value of rotor tip speed statistical factor maximum value of rotor tip speed statistical factor blade lift-curve slope with compressibility effect blade lift-curve slope without compressibility effect speed of sound first harmonic coefficient of longitudinal blade flapping with respect to shaft rotor diameter statistical factor minimum value of rotor diameter statistical factor maximum value of rotor diameter statistical factor first harmonic coefficient of lateral blade flapping with respect to shaft blade chord main drag coefficient of rotor blades main drag coefficient of tail rotor blades equivalent helicopter drag area, including fuselage, gear, rotor hub and tail,  

f ec f el h hmin hmax ∆h k k min k max ki ∆m rf Clav

equivalent helicopter drag area at α F = 0 equivalent helicopter lift area altitude minimum altitude maximum altitude altitude increment equivalent helicopter drag area empirical factor minimum value of equivalent helicopter drag area empirical factor maximum value of equivalent helicopter drag area empirical factor induced power empirical factor mass increment convergence relaxation factor mean lift coefficient, 6 Cσt 1+13 µ2

Cp ∆C p

CT

CTmin CTmax

D

Df

power coefficient, PR /ρAΩ3 R3 profile power increase due to the compressibility effect thrust coefficient, T/ρAΩ2 R2 minimum thrust coefficient maximum thrust coefficient parasite drag of rotorcraft fuselage parasite drag force

D/ 0.5ρV2

2

Eo Eomax Et Etmax Fp

H

HTR

ITNumb Lf

M

Mr Mat Mtip

PR

PTO

R

Rc Re Reloc

T

vi

V, V∞

Vmin Vmax

∆V

Vf Vt

WMTO

WTO

αl αn αD α Dmin α Dmax αF αM ∆α D δ δo δ0 θ θ FP ε ε av ε MAX λ λ0 λe λwj λmd λi λh λf λ f −1 λ HPS

local relative error during iterative process maximum local relative error during iterative process real relative error maximum real relative error profile power factor rotor drag force or H-force tail rotor drag force number of iterations fuselage lift force, Mach number, 0.75Vt /as blade advancing Mach number at 0.75R, (Vcosα D + 0.75Vt )/as blade advancing-tip Mach number, (Vcosα D + Vt )/as blade advancing-tip Mach number at hover, Vt /as required power for forward flight maximum (available) take-off power rotor radius correlation coefficient Reynolds number, 0.75Vt ∆c/ν blade retreating Reynolds number at 0.75R main rotor thrust rotor induced velocity rotor velocity with respect to the air or flight speed or true airspeed minimum flight speed maximum flight speed flight speed increment p local air velocity with respect to the fuselage, V 2 + 2vi V sin α D + vi 2 rotor tip speed, ΩR maximum take-off mass take-off mass

C

airfoil angle of attack, alavl + αn airfoil zero-lift angle of attack rotor disc plane incidence angle or angle of attack, positive down for forward tilt minimum rotor disc plane incidence angle maximum rotor disc plane incidence angle fuselage angle of attack, positive down for forward tilt mast tilt, positive forward downwash angle produced at fuselage by the main rotor atmospheric (static) pressure ratio, p/po , po = 101325 [ Pa ] blade average profile drag coefficient without compressibility effect blade average profile drag coefficient with compressibility effect atmospheric (static) temperature ratio, T/To , To = 288.15 [K ] flight path angle, positive down relative error, stopping criteria average error [%] maximum error [%] rotor inflow ratio, (Vsinα D + vi )/ΩR or λi + µz , positive down through disc initial value of rotor (induced) inflow ratio for numerical solution analytical solution of rotor inflow ratio λ for edgewise flight (α D = 0) initial value of rotor inflow ratio proposed by Wayne Johnson initial value of rotor inflow ratio proposed by Wayne Johnson—modified induced inflow ratio, vi /ΩR rotor inflow ratio in hover rotor inflow ratio or induced inflow ratio numerically obtained by standard precision value from previous iteration high precision solution of rotor inflow ratio (benchmark solution)

µ µz µc ρ ρ ISA ν σ σmin σmax σmean Ω Ωn

APR

BERP

CPU

CFD

CSD

CG

GIF

GPU

ISA

MTOM

MTOMmin MTOMmax NaN

NCP

NR

NoF PoF

RW

TOM

UAV

WG

1P2R

1P3R

1P4R

1P4R

2P4R

3P8R

rotor advance ratio, Vcosα D /ΩR rotor axial velocity ratio, Vsinα D /ΩR or µ tan α D q rotor velocity (with respect to the air) ratio, V/ΩR or µ2 + µ2z air density air density according to the International Standard Atmosphere kinematic viscosity rotor solidity, Nc/(πR) minimum rotor solidity maximum rotor solidity mean rotor solidity rotor rotational speed nominal rotor rotational speed acceptable parameter range British Experimental Rotor Programme central processing unit computational fluid dynamics computational structural dynamics control group Glauert inflow formula graphics processing unit International Standard Atmosphere maximum take-off mass minimum value of maximum take-off mass maximum value of maximum take-off mass not a number numerical computational plan Newton–Raphson number of failed convergence percentage of failed convergence rotary wing take-off mass unmanned aerial vehicle working group one-point two-order of convergence one-point three-order of convergence one-point four-order of convergence one-point five-order of convergence two-point five-order of convergence three-point eight-order of convergence

Appendix A Longitudinal force equilibrium model (LFEM) The main two equations of the model are previously given in the study, which is Equations (14) and (15). Impossibility of using a small angle assumption leads to the more complex longitudinal equilibrium model, given below. Airframe drag is usually written in terms of an equivalent parasite drag area as: D = 0.5ρVf2 f e ,

(A1)

where f e is equivalent parasite drag area, which depends on the fuselage angle of attack. For this study, the following model is adopted: f e = f ec (A + B(sinn α F )),

(A2)

where A = 1, B = 10, and n = 3. Constant f ec is a value of the equivalent parasite drag area at α F = 0, usually represented as a function of maximum take-off weight as: f ec = k(WMTO /1000)2/3

(A3)

According to [24,34], the constant k is based on historical data and varies from kmax =

1.3533 for high airframe drag to kmin = 0.3934 for the very best industrial solutions. The

average value is kav = 0.66. Fuselage lift force is calculated as L f = −0.5ρVf2 f el .

(A4)

Equivalent lift area f el is also scalable, based on the helicopter model from [25] (p. 679), as in Table A1. Table A1. Equivalent lift area, f el .

1 f

αF

fel 1

α F ≤ 15◦ 15◦ < α F ≤ 22.5◦ 22.5◦ < α F ≤ 30◦ α F > 30◦

0.113 f ecm (−α F ) − 0.282 f ecm

0.0133 f ecm (−α F ) − 1.7 f ecm

−2 f ecm 0

ecm is f ec calculated for kav = 0.66.

As is shown in Figure 2b, the fuselage angle of attack can be evaluated as αF =

180 (α D − α M + ∆α D ), π

(A5)

where ∆α D is a downwash angle caused by the main rotor stream and, according to Figure 2b, calculated as   λi cosα D ,

(A6)

∆α D = arctan µc + λi sinα D where λi as modeled with an assumption that CT ≈ Cw , as Cw λi = q ,

2 (λe + µtan α D )2 + µ2

(A7)

The main rotor drag force H is roughly estimated as follows:

H=

3.65 πρσcd Vt VR2 cos α D , 8

(A8)

and the tail rotor drag force HTR as

HTR =

3.65 πρσTR cd TR Vt TR VR TR 2 cos α D , 8

(A9)

where cdTR = 0.01 and σTR = 0.1. Vt TR and R TR are obtained by statistics, given in [23]. The most used expression for cd in low-fidelity rotor models is the quadratic equation for NACA 23012 airfoil at Re = 2 × 106 , developed by Bailey [35]: cd = 0.0087 − 0.0216αl + 0.645α2l .

(A10)

The biggest disadvantage of Bailey’s expression is a significant drop in accuracy for αl > 11.8◦ [21]. Therefore, based on experimental test data for 0 ≤ αl ≤ 20◦ for NPL9618

airfoil given in [36] and with the use of the GraphPad PRISM® regression analysis tool, a cubic expression is developed: cd = δ0 ( Re , M) + 0.02274αl − 0.6453α2l + 5.001α3l ,

(A11)

where δ0 ( Re , M) represents a blade profile drag, and is calculated as given in Table A2. Table A2. Rotor blade profile drag estimation. Mr

106 ≤Re ≤107

Mr ≤ 0.8

δ0 = 0.015058 − 0.0010138 log Re

0.8 < Mr ≤ 1

Mr > 1

Re <106 , Re >107 1  6 8 δ0 = 0.00867 2×R10 e

δ0 = δMr ≤0.8 + 0.4( Mr − 0.8) 1

For Re = 2 × 106 and Mr < 0.8, the Equation (A11) becomes: cd = 0.00867 + 0.02274αl − 0.6453α2l + 5.001α3l

(A12)

NPL9618 airfoil has a superior drag characteristic over NACA 23012 and is used for the Lynx helicopter main rotor blades. The modeling of Reynolds and Mach numbers effects on the blade average profile drag coefficient is based on [37,38] and is shown in Table A2. For the determination of an average blade angle of attack, well-known relations for αl , Clav and Ct have been used (see nomenclature). Generally, an airfoil lift-curve slope al primarily depends on the Mach number and significantly less on the Reynolds number [30,37,39,40]. Therefore, the Reynolds number influence is neglected. As shown in Table A3, the lift-curve slope modeling is based on a simple Prandtl–Glauert correction for Mr ≤ 0.9. For 0.9 < Mr ≤ 1.1, the McCroskey expression has been used (Equation (10) from [37], where t/c = 0.12). For higher Mr , the lift-curve slope is practically equal to zero. Table A3. Main rotor blade lift-curve slope determination. Mr Mr ≤ 0.9

0.9 < Mr ≤ 1.1

al a lo q

1−(0.75Mr )2

3.1515 0.26Mr 2

− 13

The system of nonlinear equations within the longitudinal force equilibrium model (LFEM) is transformed into a single nonlinear algebraic longitudinal force equilibrium (LFE) equation with α D as a variable using symbolic algebra. Furthermore, depending on the values of Mr and α F , there are 16 variants of LFE, represented by Equations (S1)–(S16). LFE is successively solved by the NR method about three times faster than the classic Newton-Kantorovich method for solving systems of nonlinear equations. References 1. 2. 3. 4. 5.

Glauert, H. The Theory of the Autogyro. J. R. Aeronaut. Soc. 1927, 31, 483–508. [CrossRef] Peters, D.A. How Dynamic Inflow Survives in the Competitive World of Rotorcraft Aerodynamics. J. Am. Helicopter Soc. 2009, 54,

011001. [CrossRef]

Smith, M.J.; Moushegian, A. Dual-Solver Hybrid Computational Approaches for Design and Analysis of Vertical Lift Vehicles. Aeronaut. J. 2022, 126, 187–208. [CrossRef] Chen, R.T. A survey of nonuniform inflow models for rotorcraft flight dynamics and control applications. In Proceedings of the Fifteenth European Rotorcraft Forum, Amsterdam, The Netherland, 12–15 September 1989. Peters, D.A.; Modarres, M.; Howard, A.B.; Rahming, B. A Third Approximation to Glauert’s Momentum Theory. J. Am. Helicopter Soc. 2016, 61, 042007. [CrossRef]

6. 7. 8. 9. 10. 11. 12. 13. 14. 15. 16. 17. 18. 19. 20. 21. 22. 23. 24. 25. 26. 27. 28. 29. 30. 31. 32. 33. 34. 35. 36. 37. 38.

Rubin, R.L.; Zhao, D. New Development of Classical Actuator Disk Model for Propellers at Incidence. AIAA J. 2021, 59, 1040–1054. [CrossRef] Petrovic, Z.; Stupar, S.; Kostic, I.; Simonovic, A. Determination of a Light Helicopter Flight Performance at the Preliminary Design Stage. Stroj. Vestn. J. Mech. Eng. 2010, 56, 535–543. Traub, J.F. Iterative Methods for the Solution of Equations, 2nd ed.; Chelsea Publishing Company: New York, NY, USA, 1982. Petkovic, M.S.; Neta, B.; Petkovic, L.; Dzunic, J. Multipoint Methods for Solving Nonlinear Equations, 1st ed.; Elsevier Science Publishing Co., Inc.: San Diego, CA, USA, 2014; ISBN 9780123972989. Dzunic, J. Multipoint Methods for Solving Nonlinear Equations. Ph.D. Thesis, University of Nis, Nis, Serbia, 2012. Petkovic, M.S.; Neta, B.; Petkovic, L.; Dzunic, J. Multipoint methods for solving nonlinear equations: A survey. Appl. Math. Comput. 2014, 226, 635–660. [CrossRef] Berahas, A.S.; Jahani, M.; Richtárik, P.; Takáč, M. Quasi-Newton methods for machine learning: Forget the past, just sample. Optim. Methods Softw. 2022, 37, 1668–1704. [CrossRef] Goldfarb, D.; Ren, Y.; Bahamou, A. Practical quasi-newton methods for training deep neural networks. Adv. Neural Inf. Process. Syst. 2020, 33, 2386–2396. [CrossRef] Davis, K.; Schulte, M.; Uekermann, B. Enhancing Quasi-Newton Acceleration for Fluid-Structure Interaction. Math. Comput. Appl. 2022, 27, 40. [CrossRef] Bogaers, A.E.J.; Kok, S.; Reddy, B.D.; Franz, T. Quasi-Newton methods for implicit black-box FSI coupling. Comput. Methods Appl. Mech. Eng. 2014, 279, 113–132. [CrossRef] Akbari, A.; Giannacopoulos, D. An efficient multi-threaded Newton–Raphson algorithm for strong coupling modeling of multi-physics problems. Comput. Phys. Commun. 2021, 258, 107563. [CrossRef] Liu, L.; Zhang, H.; Wu, Y.; Liu, B.; Guo, J.; Li, F. A modified JFNK with line search method for solving k-eigenvalue neutronics problems with thermal-hydraulics feedback. Nucl. Eng. Technol. 2023, 55, 310–323. [CrossRef] Ganguli, R.A. Pedagogical Example for STEM Using the Glauert Inflow Equation, Mathematica and Python. In Proceedings of the AIAA SciTech Forum, San Diego, CA, USA, 7–11 January 2019. [CrossRef] Bronshtein, I.N.; Semendyayev, K.A.; Musiol, G.; Muehlig, H. Handbook of Mathematics, 5th ed.; Springer: Berlin/Heidelberg, Germany, 2007; ISBN 978-3-540-72121-5. Leishman, G.J. Principles of Helicopter Aerodynamics, 2nd ed.; Cambridge University Press: Cambridge, UK, 2006. Johnson, W. Rotorcraft Aeromechanics; Cambridge University Press: New York, NY, USA, 2013. Johnson, W. NDARC—NASA Design and Analysis of Rotorcraft: Theory; Appendix 3, Release 1.11; TP–2015-218751; National Aeronautics and Space Administration: Washington, DC, USA, 2016. Rand, O.; Khromov, V. Helicopter Sizing by Statistics. J. Am. Helicopter Soc. 2004, 49, 300–317. [CrossRef] Harris, F.D. Introduction to Autogyros, Helicopters, and Other V/STOL Aircraft, Volume II: Helicopters; National Aeronautics and Space Administration: Washington, DC, USA, 2012. Prouty, R. Helicopter Performance, Stability and Control, 2nd ed.; Krieger Publishing Company: Malabar, FL, USA, 2002. Keys, C.N.; Wiesner, R. Guidelines for Reducing Helicopter Parasite Drag; Boeing Vertol Company: Ridley Park, PA, USA, 1974. Bramwell, A.R.S.; Done, G.; Balmford, D. Bramwell’s Helicopter Dynamics, 2nd ed.; Butterworth-Heinemann: Oxford, UK, 2000. Harris, F.D. Rotor Performance at High Advance Ratio: Theory versus Test; NASA/CR-2008-215370; National Aeronautics and Space Administration: Washington, DC, USA, 2008. Harris, F.D. Rotary Wing Aerodynamics—Historical Perspective and Important Issues. In Proceedings of the American Helicopter Society National Specialist’s Meeting on Aerodynamics and Aeroacoustics, Arlington, TX, USA, 25–27 February 1987. McCroskey, W.J. A Critical Assessment of Wind Tunnel Results for the NACA 0012 Airfoil; NASA-TM-100019; National Aeronautics and Space Administration: Washington, DC, USA, 1987. Bezanson, J.; Edelman, A.; Karpinski, S.; Shah, V.B. Julia: A Fresh Approach to Numerical Computing. SIAM Rev. 2017, 59, 65–98. [CrossRef] Sharma, J.R.; Sharma, R. A new family of modified Ostrowski’s methods with accelerated eighth order convergence. Numer. Algor. 2010, 54, 445–458. [CrossRef] Lau, B.H.; Louie, A.W.; Griffiths, N.; Sotiriou, C.P. Performance and Rotor Loads Measurements of the Lynx XZ170 Helicopter with Rectangular Blades; NASA-TM-104000; National Aeronautics and Space Administration: Washington, DC, USA, 1993. Dodic, M.; Krstic, B.B.; Rasuo, B.P. Single Rotor Helicopter Parasite Drag Estimation in the Preliminary Design Stage. Tehnika 2018, 73, 65–77. [CrossRef] Bailey, F.J., Jr. A Simplified Theoretical Method of Determining the Characteristics of a Lifting Rotor in Forward Flight; NACA Report 716; US Government Printing Office: Washington, DC, USA, 1941. Lau, B.H.; Louie, A.W.; Griffiths, N.; Sotiriou, C.P. Correlation of the Lynx-XZ170 Flight-Test Results Up to and Beyond the Stall Boundary. In Proceedings of the AHS Forum 49, St. Louis, MO, USA, 19–21 May 1993. Totah, J. A Critical Assessment of UH-60 Main Rotor Blade Airfoil Data; NASA-TM-103985; National Aeronautics and Space Administration: Washington, DC, USA, 1993. Yamauchi, G.K.; Johnson, W. Trends of Reynolds Number Effects on Two-Dimensional Airfoil Characteristics for Helicopter Rotor Analyses; NASA-TM-84363; National Aeronautics and Space Administration: Washington, DC, USA, 1983.

39. 40.

Rasuo, B. The influence of Reynolds and Mach numbers on two-dimensional wind-tunnel testing: An experience. Aeronaut. J. 2011, 115, 1166. [CrossRef] Rasuo, B. Scaling between Wind Tunnels–Results Accuracy in Two-Dimensional Testing. Trans. Jpn. Soc. Aeronaut. Space Sci. 2012, 55, 109–115. [CrossRef]

Disclaimer/Publisher’s Note: The statements, opinions and data contained in all publications are solely those of the individual author(s) and contributor(s) and not of MDPI and/or the editor(s). MDPI and/or the editor(s) disclaim responsibility for any injury to people or property resulting from any ideas, methods, instructions or products referred to in the content.

