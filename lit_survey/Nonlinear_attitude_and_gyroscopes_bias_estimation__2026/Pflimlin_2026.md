# Nonlinear attitude and gyroscope's bias estimation for a VTOL UAV

**Authors:** Jean Michel Pflimlin, Tarek Hamel, Philippe Souères
**Year:** 2026
**Source URL:** Local File: nonlinear-attitude-and-gyroscope-s-bias-estimation-for-a-3a8wlgf80p.pdf

## Abstract
for a VTOL UAV. We describe an observation strategy to restitute the complete attitude matrix of the vehicle starting from Inertial Measurement Unit (IMU) and magnetometers. This study is a part of the development of the ducted fan UAV designed by Bertin Technologies. First, a measured orientation matrix is calculated from both inertial vectors which are the gravity and the earth magnetic field. Then, an estimated orientation is built by integrating gyroscopic readings, and corrected by the measured one. Nonlinear observation techniques are used to design a nonlinear estimator of the orientation matrix and an adaptive filter of the gyroscope’s bias which ensure the convergence of the observer. Such an observer is as efficient as classical Extended Kalman Filtering based observers, and easier to implement in real time. Simulations are proposed to illustrate the concept. c Copyright ° 2005 IFAC Keywords: VTOL UAV, Attitude and Heading Restitution Sensor, SO(3) group 1. INTRODUCTION The design of autonomous navigation strategy for Micro Air Vehicles (MAV) has now become a very challenging research area. Those small and

## My Notes
- Start typing your research notes here...

---

## Full Paper Text
                        NONLINEAR ATTITUDE AND GYROSCOPE’S  
                          BIAS ESTIMATION FOR A VTOL UAV  
  
                                Jean Michel Pflimlin ∗,1 Tarek Hamel ∗∗  
                                  Philippe Souères ∗ Najib Metni ∗∗∗  
  
  
                       ∗  
                           LAAS - CNRS, 7, Av. du colonel Roche, 31077 Toulouse  
                           ∗∗  
                            I3S - CNRS, Les Algorithmes, Bât. Euclide, 2000, Rte  
                                des Lucioles B.P.121, 06903 Sophia Antipolis  
                            ∗∗∗  
                                LCPC, 58, Bvd. Lefebvre, 75732 Paris Cedex 15  
  
  
  
           Abstract: This paper addresses the problem of attitude and heading restitution  
           for a VTOL UAV. We describe an observation strategy to restitute the complete  
           attitude matrix of the vehicle starting from Inertial Measurement Unit (IMU)  
           and magnetometers. This study is a part of the development of the ducted fan  
           UAV designed by Bertin Technologies. First, a measured orientation matrix is  
           calculated from both inertial vectors which are the gravity and the earth magnetic  
           field. Then, an estimated orientation is built by integrating gyroscopic readings,  
           and corrected by the measured one. Nonlinear observation techniques are used to  
           design a nonlinear estimator of the orientation matrix and an adaptive filter of the  
           gyroscope’s bias which ensure the convergence of the observer. Such an observer  
           is as efficient as classical Extended Kalman Filtering based observers, and easier  
           to implement in real time. Simulations are proposed to illustrate the concept.  
                        c  
           Copyright °    2005 IFAC  
  
           Keywords: VTOL UAV, Attitude and Heading Restitution Sensor, SO(3) group  
  
  
  
  
              1. INTRODUCTION                           little has been done to the development of secure  
                                                        platforms (Hamel et al., 2002). Such platforms  
The design of autonomous navigation strategy for        have a considerable potential for surveillance and  
Micro Air Vehicles (MAV) has now become a               inspection roles in dangerous or awkward environ-  
very challenging research area. Those small and         ments. One of the projects of the French National  
discreet flying vehicles, able to perform vertical      Direction of Armament, DGA, is to supply the  
takeoff and landing (VTOL), are of great interest       infantry with such micro-drones by 2007. As a  
for civil and military operations in urban environ-     part of this project, the group Bertin Technology  
ment. The design of autonomous VTOL vehicle             has in charge the development of a ducted fan  
capable of stationary flight has recently motivated     UAV which is represented in figure 1. The work  
many research projects (Pflimlin et al., 2004). Sig-    presented in this paper has been developed in  
nificant interest has been directed towards the de-     the framework of a collaboration between, Bertin  
velopment of autonomous scale model helicopter          Technologies, the LAAS-CNRS and I3S laborato-  
due to their high payload to power ratio. He-           ries, both devoted to modelling and elaboration of  
licopters, however, are extremely dangerous in          autonomous control strategies. In a first step, the  
practice due to the exposed rotor blades. Very          automatic control design must allow the vehicle to  
                                                        be easily operated by an inexperienced user, de-  
1 Partially supported by Bertin Technologies, and the   spite wind perturbations. Designing such a remote  
French ministry of Armament  
                                                       xb points forward, zb downward, and to complete  
                                                       the base, yb points to the right.  
                                                       The transformation matrix between the inertial  
                                                       frame and the body fixed frame is called attitude  
                                                       matrix R. By definition, each column of R is made  
                                                       of the components of the corresponding body fixed  
                                                       base vector expressed in the inertial frame 2 :  
                                                                          R = [xb , yb , zb ]I  
                                                       R being the transformation matrix between two  
                                                       orthogonal frames, it is an element of SO(3). With  
                                                       the usual matrix product, SO(3) is a group whose  
                                                       elements satisfy (Murray et al., 1994):  
Fig. 1. A view of Bertin VTOL UAV.                           ∀P ∈ SO(3), P −1 = P T et det(P ) = 1  
control requires an efficient attitude control, able   Introducing the canonical base {e1 , e2 , e3 } of IR3 :  
to deal with internal coupling of the system and       e1 = [1, 0, 0]T , e2 = [0, 1, 0]T , e3 = [0, 0, 1]T ,  
to maintain the vehicle in a secure flight envelope.   the components of the inertial base vectors with  
This is strongly dependent on a reliable attitude      respect to the body fixed frame are given by:  
estimation. Much has been done in attitude and  
heading restitution using Extended Kalman filters          [x0 ]A = RT e1 [y0 ]A = RT e2 [z0 ]A = RT e3  
and a representation of the orientation based on  
quaternions (Marins et al., 2001). However, im-        Denoting by Ω = [p, q, r]T the angular veloc-  
plementing a Kalman filter pose serious problem        ity vector of the body fixed frame relative to  
due in particular to the high sensitivity of the       the inertial frame, expressed in body frame, the  
covariance matrix to numerical errors. Engineer-       kinematic equation of attitude that expresses the  
ing techniques such as [U, D]-decomposition are        evolution of R through time is given by (Murray  
indispensable in practice to increase robustness       et al., 1994):  
of the filter (Grewal et al., 2001). Using rotation                        Ṙ = Rsk(Ω)               (1)  
matrices to describe the orientation and designing     Where sk(Ω) denotes the skew-symmetric ma-  
a nonlinear observer directly in the rotation set      trix 3 associated to Ω.  
SO(3) allows to overcome the linearization step  
and to provide a more physical view of what the  
observer is doing.                                     2.2 IMU and magnetometers modelling  
In this paper we describe an observation strategy  
                                                       The VTOL UAV is equipped with a low cost strap  
to restitute the complete attitude matrix of the  
                                                       down IMU, that provides the angular velocity Ω  
micro-drone starting from Inertial Measurement  
                                                       components as well as the linear accelerations,  
Unit (IMU) and magnetometers. The proposed  
                                                       and a 3-axis magnetometer. Angular rates allow  
observer is as efficient as an Extended Kalman  
                                                       to predict the evolution of the attitude matrix,  
filter and much easier to implement. The paper  
                                                       while the measure provided by accelerometers and  
is organized as follows: section 2 provides a de-  
                                                       magnetometers is used to correct this prediction.  
scription of the geometric and kinematic model of  
                                                       When designing an observer, one has to make a  
the attitude and a model of the sensors used to  
                                                       trade-off between the drift resulting from integra-  
provide the attitude measurements. The observer  
                                                       tion of the gyroscopic measurements and the relia-  
design is given in section 3. Simulation results are  
                                                       bility of noisy readings provided by accelerometers  
then proposed to illustrate the method.  
                                                       and magnetometers.  
           2. SYSTEM MODELLING  
                                                       Gyroscope modelling The 3-axis gyroscope pro-  
2.1 Attitude representation and kinematics             vides the angular velocities of the body frame  
                                                       relative to the inertial frame, expressed in the  
Two reference frames are considered to model the       body fixed frame. If the measured angular rates  
system (Boiffier, 1998):                               were perfect, it would be sufficient to integrate  
• I is an inertial frame attached to the earth. It     these measurements using equation (1) to provide  
is assumed to be Galilean. It is associated to the     the orientation matrix R. However, due to open-  
vector base {x0 , y0 , z0 }, where x0 points to the    loop integrations, any bias on gyroscopic measure-  
magnetic North, z0 points to the center of earth,      ments lead to an error on the attitude prediction  
and y0 points to the East to complete the base.  
                                                       2 To describe the attitude of a vehicle, many authors use  
• A is body-fixed frame attached to the vehicle.       the Direct Cosine Matrix DCM . By definition one gets  
It is associated to the vector base {xb , yb , zb }.   DCM = RT  
The conventional roll, pitch and yaw axis used in      3 ∀u ∈ <3 , sk(u) is the skew matrix associated to u :  
  
mechanics of flight are considered in this paper:      ∀v ∈ <3 , u × v = sk(u)v  
which grows to infinity. This phenomena is known       2.3 Measured orientation matrix construction  
as gyroscopic drift. To take into account on-line  
estimation of gyroscope’s bias, we consider the        Both inertial vectors measured by the IMU com-  
model of gyroscopic measurements given by:             pleted with magnetometers are represented in fig-  
                                                       ure 2. Let u be the normalized components of the  
                      Ωm = Ω + b                (2)  
where b depends on the temperature. Many low  
cost IMU are now internally compensated in  
temperature, and as a result, b oscillates slowly  
around a constant average value. In the observer  
design, b will be supposed constant. We will pro-  
pose an adaptive filter to estimate b on-line. In  
practice, the estimator will succeed in estimating  
gyroscope’s bias even though they vary slowly.  
  
  
Accelerometers modelling The 3-axis accelerom-         Fig. 2. Representation of the gravity and the earth  
eter provides the non gravitational linear accel-           magnetic field in the inertial frame.  
erations applied to the vehicle relative to iner-  
tial frame (Radix, 1980). Accelerometers readings      accelerometers readings in body frame:  
gather all forces applied to the vehicle, except                            Am  
gravity. In case of ”strap-down” IMU, readings are                    u=         ≈ −RT e3  
                                                                           kAm k  
expressed in body fixed frame. When the absolute  
acceleration of the vehicle in the inertial frame is   We are going to present a method to build the  
weak (kγ abs k ¿ g), accelerometers’ readings Am       complete attitude matrix R by using both inertial  
provide the components of gravity vector in A:         vectors m and g expressed in body frame. The  
                                                       main idea is to use Gram-Schmidt’s orthonor-  
   −→  
   Am = γ abs −gz0 =⇒ Am ≈ −gRT e3                     malization method to construct RT e1 and RT e3  
             |{z}                               (3)    starting from h and u and to complete the base  
         kγ abs k¿g  
                                                       by the cross product of both vectors.  
If measurements were absolutely reliable, pitch  
and roll of the vehicle could be directly deduced      Proposition 1. Consider the following matrix R  
from the accelerometers, working as inclinometers      built from h and u:  
in that case. However, accelerometers’ readings                 ·                                ¸T  
are very noisy and very sensitive to the vibratory                 h − (uT h)u     −u × h  
                                                          R=                                  −u  
environment surrounding the IMU. Furthermore,                     kh − (uT h)uk kh − (uT h)uk  
if for any reason (wind gust, or transition from       Then we have R = R.  
hovering to forward flight), the vehicle acquires a  
non negligible absolute acceleration in the inertial   Proof . First we have:  
frame, the attitude provided by accelerometers         h − (uT h)u = RT m − (−(RT e3 )T RT m)(−RT e3 )  
will no more be reliable. Therefore, it is important               = RT (m − (eT3 m)e3 )  
in practice to correlate the attitude measured                     = (eT1 m)RT e1  
by the accelerometers and the attitude predicted  
by integrating the gyroscopic measurements to          By convention e1 points to the North, so eT1 m > 0.  
provide a correct attitude estimation (Suh, 2003).     It yields:  
                                                                    eT1 m = kh − (uT h)uk  
                                                       So the first column of RT expresses as RT e1 . The  
Magnetometers modelling To achieve complete            second column of RT can be written:  
attitude estimation, we add to inertial measure-  
ments the information provided by a 3-axis mag-           −u × h             h − (uT h)u  
                                                              T  
                                                                    = (−u)×               = (RT e3 )×(RT e1 )  
netometer. Such a device is able to give the yaw       kh − (u h)uk         kh − (uT h)uk  
angle ψ by measuring the earth magnetic field in        As (RT e3 ) × (RT e1 ) = RT (e3 × e1 ), the second  
the body fixed frame. Let m be the direct cosine       column of RT equals RT e2 . Finally, we get:  
vector of the earth magnetic field. Let m and h be  
the components of m respectively in the inertial                   RT = RT [e1 , e2 , e3 ] = RT  
frame I and in the body fixed frame A. Then we         ¤  
have the following equation:  
                      h = RT m                  (4)  
                                                                   3. OBSERVER DESIGN  
By definition of the inertial frame, one gets m ∈  
span{e1 , e3 }, that is to say:                        In this section, a nonlinear estimator of the ori-  
                                                       entation matrix and an adaptive filter of the gy-  
             m = (eT1 m)e1 + (eT3 m)e3                 roscope’s bias are designed. Convergence of the  
resulting estimator is achieved by means of non-              Developing the term (I3×3 − R̃)(I3×3 − R̃)T , one  
linear observation techniques (Krstic et al., 1995).          gets  
Let us denote by R̂ ∈ SO(3) and b̂ the estimations  
                                                              (I3×3 − R̃)(I3×3 − R̃)T = I3×3 − R̃ − R̃T + R̃R̃T  
of R and b respectively. Estimations errors R̃ and  
b̃ are defined by:                                            As a product of two matrices in SO(3), R̃ is  
                        T  
             R̃ = R̂ R and b̃ = b − b̂                        naturally in SO(3). This means that R̃R̃T = I3×3 .  
The convergence of the observer is ensured if and             So, as trace(R̃) = trace(R̃T ), the final expression  
only if the following two conditions are verified:            of V is given by:  
                                                                                               1  
             R̃ −→ I3×3 and b̃ −→ 0                                      V = 3 − trace(R̃) + Γ−1 b̃T b̃       (10)  
                                                                                               2  
The observer design consists in building filters on  
 ˙     ˙                                                      Now, let us calculate the time derivative of R̃.  
R̂ and b̂. To keep R̂ within SO(3), the following  
                                                              Using (5), one gets:  
structure on the estimator is imposed:  
                                                                      ˙  
             ˙                  ˙  
            R̂ = R̂sk(Ω̂) and b̂ = Γτ           (5)            R̃˙ = R̂T R + R̂T Ṙ = −sk(Ω̂)R̂T R + R̂T Rsk(Ω)  
                                                                   = −sk(Ω̂)R̃ + R̃sk(Ω)  
Where Γ is a positive gain which allows to tune  
up the adaption of the gyroscope’s bias.                      There, let us recall one of the main properties of  
                                                              skew-symmetric matrix (Murray et al., 1994):  
Let us introduce the following direct cosine vec-  
tors of IR3 , which represent the components of                 ∀R ∈ SO(3), ∀ω ∈ IR3 , Rsk(ω)RT = sk(Rω)  
the inertial base vectors in the body frame and in  
the estimated body frame respectively:                        So R̃˙ is given by:  
           x = RT e1 ; y = RT e2 ; z = RT e3                  R̃˙ = −sk(Ω̂)R̃ + R̃sk(Ω) I3×3 = −sk(Ω̂ − R̃Ω)R̃  
           x̂ = R̂T e1 ; ŷ = R̂T e2 ; ẑ = R̂T e3                                      |{z}  
                                                                                            =R̃T R̃  
Let X, Y , Z denote the cross product between the  
estimated direct cosine vectors and the real ones:            Now, using (2), equation (6) can be written:  
X = x̂ × x, Y = ŷ × y, Z = ẑ × z.                                 Ω̂ = R̃( Ωm −b̂) + $ = R̃(Ω + b̃) + $  
                                                                            |{z}  
                                                                              Ω+b  
Theorem 1. With the above notations, the estima-  
tion law on attitude Ω̂ given by:                                                                ˙  
                                                              It yields the final expression of R̃:  
                    Ω̂ = R̃(Ωm − b̂) + $               (6)  
                                                                                R̃˙ = −sk(R̃b̃ + $)R̃                 (11)  
with $, defined as:  
                                                              Now, let us go back to the calculation of V̇ . Using  
              $ = −k(X + Y + Z), k > 0                 (7)  
                                                              (10), one gets:  
and the adaptive filter τ defined by:  
                                                                                         ˙ + Γ−1 b̃T b̃˙  
                                                                            V̇ = −trace(R̃)  
               τ = R̃T (X + Y + Z)            (8)  
ensure convergence of the estimator given in (5)              As stated before, the gyroscope’s bias is supposed  
for any initial rotation angle α0 between the es-                                   ˙           ˙     ˙  
                                                              constant. Therefore, b̃ is simply b̃ = −b̂. Using the  
timated orientation matrix and the real one and                              ˙  
any initial estimation error on gyroscope bias b̃0            expression of R̃ given by (11), V̇ can be written:  
verifying:                                                                                                 ˙  
    |b̃0 | ≤ bmax  
                                 √  
                    with bmax ≤ 2 2Γ  
                                                                      V̇ = trace(sk(R̃b̃ + $)R̃) − Γ−1 b̃T b̂  
                                           1     b2           At this stage, one can notice that R̃ can be  
    |α0 | < αmax with αmax = arccos(1 −      (4 − max ))  
                                           2      2Γ          expressed by:  
                                                        (9)  
                                                              R̃ = R̂T R = R̂T (e1 eT1 + e2 eT2 + e3 eT3 )R  
                                                                 = (R̂T e1 )(eT1 R) + (R̂T e2 )(eT2 R) + (R̂T e3 )(eT3 R)  
Proof . Let us define the Lyapunov function:                  That is to say:  
             1                  1  
        V = kI3×3 − R̃k2F rob + Γ−1 b̃T b̃                                    R̃ = x̂xT + ŷy T + ẑz T               (12)  
             2                  2  
                                                              So V̇ is finally given by:  
In the expression of V , kI3×3 − R̃kF rob denotes the  
Frobenius norm 4 of the matrix I3×3 − R̃. Using                 V̇ = trace(sk(R̃b̃ + $)x̂xT ) + trace(sk(R̃b̃ + $)ŷy T )  
                                                                                                          ˙  
the explicit form of this norm, it yields:                           +trace(sk(R̃b̃ + $)ẑz T ) − Γ−1 b̃T b̂  
      1                                     1                 Let us recall the following property of the trace:  
 V = trace((I3×3 − R̃)(I3×3 − R̃)T ) + Γ−1 b̃T b̃  
      2                                     2  
                                                                         ∀a, b ∈ IR3 trace(abT ) = aT b  
4 Recall that the Frobenius norm of a square matrix M is      This property leads to the following expression:  
given by:                  p                                      V̇ = [sk(R̃b̃ + $)x̂]T x + [sk(R̃b̃ + $)ŷ]T y  
               kM kF rob =   trace(M M T )  
                                                                                                       ˙  
                                                                       +[sk(R̃b̃ + $)ẑ]T z − Γ−1 b̃T b̂  
Keeping in mind the basic properties 5 of skew-           Let us define Da = {(α, b̃) ∈ IR × IR3 /V (α, b̃) ≤  
symmetric matrices, the following expression of V̇        a} the region delimited by the equipotential curve  
is obtained:                                              {V = a}. It is clear that for ∀v, w ∈ IR/0 ≤ v ≤  
                                                     ˙    w, Dv ⊂ Dw and that ∀a ≥ 0, (α, b̃) = (0, 0) ∈  
   V̇ = (R̃b̃)T (x̂ × x + ŷ × y + ẑ × z) − Γ−1 b̃T b̂  
             T  
        +$ (x̂ × x + ŷ × y + ẑ × z)                     Da . As V̇ < 0, we ensure that DV (t) ⊂ DV (0) .  
                                                          To determine the attractive domain of the point  
The adaptive filter given by (8) cancels the in-          (α, b̃) = (0, 0), it is sufficient to consider the  
fluence of b̃ in the expression of the Lyapunov           biggest region Da within the hyperplanes α = ±π.  
function time derivative, which therefore can be  
written as follows:  
                 V̇ = $T (X + Y + Z)  
Finally, with the expression of $ (7), we get:  
                 V̇ = −k|X + Y + Z|2               (13)  
The non positivity of V̇ is then ensured. Using the  
theorem of Lasalle, the observer state tends to the  
invariant set of V̇ :  
      {(M, b̂) ∈ SO(3) × IR3 /X + Y + Z = 0}  
  
The resulting gap R̃ between the estimated orien-  
tation matrix and the real one defines a rotation  
of an angle α about a unit vector ω between x, y, z  
and x̂, ŷ, ẑ. In the base {x, y, z}, this rotation  
                                   4  
expresses as R̃ = exp(sk(ω)α) = [rij ]1≤i,j≤3 . The       Fig. 3. Illustration of the attractive domain  
analytical form of cross products X, Y and Z in               around equilibrium configuration α = 0 .  
base {x, y, z} is given by:  
                                                          This region is tangent to both hyperplanes at  
    X = −x × x̂ = −sk(e1 )R̃e1 = −[0, −r31 , r21 ]T       points (α, b̃) = (±π, 0). At these points, V =  
    Y = −y × ŷ = −sk(e2 )R̃e2 = −[r32 , 0, −r12 ]T       2(1 − cos(±π)) = 4. The attractive domain is the  
    Z = −z × ẑ = −sk(e3 )R̃e3 = −[r13 , r31 , 0]T        interior of D4 . Considering the the upper born on  
                                                          the initial gyroscope’s bias estimation error b̃max ,  
When adding those three terms, the components             the maximal initial angular gap α0 allowed can be  
of the rotation vector ω appear (Murray et al.,           determined by the relation:  
1994):  
                                                                                       1  
                     r32 − r23                                   V (0) = 2(1 − cosα0 ) + Γ−1 |b̃0 |2 ≤ 4  
                                                                                         2  
   X + Y + Z = −  r13 − r31  = −2(sin α)ω  
                                                          This leads to the upper born on α0 6 and b̃0 given  
                     r21 − r12                            by (9). The observer being initialized in this way,  
The condition X+Y +Z = 0 defining the invariant           the convergence of R̂ to R is ensured. R̂ → R  
set becomes:  
                                                          means R̃ → I3×3 and R̃˙ → 0. Furthermore, the  
                   2(sin α)ω = 0  
                                                          convergence to zero of X + Y + Z means that $  
ω being a unit vector, this is only possible if                                           ˙  
                                                          will tend to zero, as well as b̂ (cf. (7) and (8)).  
α = 0 or α = ±π. This means that R̂ can  
                                                          Therefore, the estimation error on gyroscope’s  
converge to the real attitude matrix (case α =  
                                                          bias tends to a constant value denoted by b̃∞ .  
0) or to a matrix deduced from the real one  
by an axial symmetry about axis ω (case α =               When t → ∞ in the expression of R̃˙ given by  
±π). To guarantee the convergence of R̂ to R,             (11), it can be shown that b̃∞ verifies:  
we need to characterize the attractive domain                                   sk(b̃∞ ) = 0  
of the equilibrium point α = 0. Initializing the  
                                                          which implies the convergence of b̃ to zero.¤  
observer state inside this domain will ensure the  
convergence to the real attitude matrix. First, let                  4. SIMULATION RESULTS  
us recall the relation between the rotation angle  
α and R̃ (Murray et al., 1994):                           In our simulation, the observer given by the equa-  
                      trace(R̃) − 1                       tion (5) is sampled using Rodrigue’s formula:  
                 cos α =  
                             2                                                R̂k+1 = R̂k Ak  
The expression of the Lyapunov function given by  
(10) can be written as follows:                           6  Taking an adaption gain very small Γ = 0.01, and  
                               1                          an important initial error on gyroscope’s bias estimation  
          V = 2(1 − cos α) + Γ−1 |b̃|2                    bmax = 5◦ /s, one gets αmax ≈ 145◦ . In practice, initial-  
                               2  
                                                          izing the observer state with the first measured attitude  
                                                          matrix, maximal angular gap α0 will not exceed 30◦ so  
5   ∀u, v ∈ IR3 , sk(u)T = −sk(u) and sk(u)v = −sk(v)u    this upper born is not very constraining  
    where Ak = exp(sk(ωk )τ ), given that Ω(t) = ωk                              matrix and the real one. Behavior obtained in sim-  
    for t ∈ [kτ, (k +1)τ ]. Ak has a closed form solution                        ulation is very encouraging and integration inside  
    (Murray et al., 1994) of the form:                                           the embedded calculator of Bertin Technologies  
                               sin(kωk kτ )           1 − cos(kωk kτ ) 2         VTOL UAV is in progress. At this moment, a  
      Ak = I3×3 +                           sk(ωk ) +                 sk (ωk )   similar but simplified algorithm providing roll and  
                                  kωk k                    kωk k2  
                                                                                 pitch angles has been successfully integrated and  
     This discrete form will preserve the evolution of                           allows remote controlled flight using PIDs. In fur-  
    R̂k within SO(3) because Ak = exp(sk(ωk )τ ) is                              ther work, we will propose a nonlinear control law  
    a rotation matrix. However, to prevent any accu-                             coupled with this estimation law which ensures  
    mulation of numerical errors, columns of R̂k can                             the stability of the closed loop system despite  
    be kept orthogonal by Gram-Schmidt algorithm.                                wind gusts.  
                   0  
  
  
                 −20  
                                                                                           6. ACKNOWLEDGMENTS  
  φest (°)  
  
                 −40  
                                                                                 The authors wish to express their gratitude to  
                 −60  
                                                                                 the French Ministry of Research and Technology  
                       0   5      10     15     20       25   30     35     40  
                                                                                 and the National Direction of Armament DGA for  
                  40                                                             granting this research program. They also thank  
                  30                                                             Olivier Philippe, VTOL UAV project manager at  
        (°)  
            est   20  
                                                                                 Bertin Technologies, who provided funding and  
      θ  
                                                                                 support in this research. They also address a  
                  10  
                                                                                 special thanks to the whole drone team of Bertin  
                   0  
                       0   5      10     15     20       25   30     35     40   Technologies for their assistance and advices.  
                   0  
  
                                                                                                  REFERENCES  
                 −50  
ψest (°)                                                                         Boiffier, J.L. (1998). The Dynamics Of Flight.  
             −100  
                                                                                      Jhon Wiley & Sons.  
             −150  
                                                                                 Grewal, M.S., L. Weill and A.P Andrews (2001).  
                                                                                      Global Positioning Systems, Inertial Naviga-  
                       0   5      10     15      20      25   30     35     40  
                                              time (s)  
                  10  
                                                                                      tion, and Integration. Jhon Wiley & Sons.  
                   5                                                             Hamel, T., R. Mahony, R. Lozano and J. Os-  
    best (°/s)     0                                                                  trowski (2002). Dynamic modelling and con-  
                  −5                                                                  figuration stabilization for an x4-flyer. In:  
                 −10  
                                                                                      Proc. of the 15th International Federation of  
                       0   5      10     15      20      25   30     35     40  
                                              time (s)                                Automatic Control Symposium, IFAC 2002.  
                                                                                      Barcelona, Spain.  
    Fig. 4. Evolution of the estimated Euler angles                              Krstic, M., I. Kanellakopoulos and P. Kokotovic  
         and gyroscope’s bias.                                                        (1995). Nonlinear and Adaptive Control De-  
                                                                                      sign. John Wiley & Sons.  
    Simulations were performed to represent the con-                             Marins, J.L, X. Yun, E.R. Bachmann, R.B.  
    vergence of the three Euler angles estimation  
                                                                                      McGhee and M.J. Zyda (2001). An extended  
    and the gyroscope’s bias estimation. A good be-  
                                                                                      kalman filter for quaternion-based orientation  
    havior of the estimator can be observed on fig-  
    ure 4 in spite of large gaps between initial val-                                 estimation using marg sensors. In: Proc. of  
    ues of estimated Euler angles and the real ones                                   the IEEE Int. Conf. on Intelligent Robots and  
    ([−50◦ , 30◦ , −100◦ ]) and an important gyroscopic                               Systems. Maui, Hawaii, USA. pp. 2003–2011.  
    drift (b = [0.02, −0.05, 0.07]T rd/s).                                       Murray, R.M., Z. Li and S. Sastry (1994). A Math-  
                                                                                      ematical Introduction to Robotic Manipula-  
                                                                                      tion. CRC Press.  
                                   5. CONCLUSION                                 Pflimlin, J.M., P. Souères and T. Hamel (2004).  
                                                                                      Hovering flight stabilization in wind gusts for  
    In this draft, we have proposed an observation                                    a ducted fan uav. In: Proc. of the 43rd IEEE  
    strategy to estimate the orientation of a VTOL                                    Conference on Decision and Control, CDC04.  
    UAV taking advantage from both short term pre-  
                                                                                      Atlantis, Bahamas.  
    cision given by integration of measured angular  
                                                                                 Radix, J.C. (1980). Systèmes inertiels à com-  
    rates and long term precision provided by ac-  
    celerometers and magnetometers. A nonlinear es-                                   posants liés ”strap-down”. CEPADUES Edi-  
    timation law defined on SO(3) and an adaptive                                     tions.  
    filter on gyroscope’s bias have been designed and                            Suh, Y. Soo (2003). attitude estimation using low  
    a theorem both proves the convergence of the                                      cost accelerometer and gyroscope. In: Proc.  
    method and provides an upper born on the initial                                  of the 7th Korea Russia International Sympo-  
    angular gap between the estimated orientation                                     sium, KORUS03. Ulsan, Korea. pp. 423–427.  
  
