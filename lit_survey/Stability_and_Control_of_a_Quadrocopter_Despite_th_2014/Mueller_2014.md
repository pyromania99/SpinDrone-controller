# Stability and Control of a Quadrocopter Despite the Complete Loss of One, Two, or Three Propellers

**Authors:** Mark Wilfried Mueller, Raffaello D'Andrea
**Year:** 2014
**Source URL:** https://hiperlab.berkeley.edu/wp-content/uploads/2018/05/2014_StabilityAndControlOfAQuadrocopterDespiteTheCompleteLossOfOneTwoOrThreePropellers.pdf

## Abstract
quadrocopter maintaining a height around a position in space despite having lost a single, two opposing, or three propellers. In each case the control strategy consists of the quadrocopter spinning about a primary axis, fixed with respect to the vehicle, and tilting this axis for translational control. A linear, timeinvariant description of deviations from the attitude equilibrium is derived, allowing for a convenient cascaded control design. The results for the cases of losing one and two propellers are validated in experiment, while the case of losing three propellers is validated in a nonlinear simulation. These results have application in multicopter fault-tolerant control design, and also point to possible design directions for novel flying vehicles. I. I NTRODUCTION Multicopters have found broad use as research platforms, used e.g. for vision based pose estimation with quadrocopters [1] and hexacopters [2], and also as platforms allowing for new capabilities. For example, the use of both quadrocopters and hexacopters for whale monitoring is investigated in [3], and hexacopters are used in [4] for weed research; a team of quadrocopters is used to carry a

## My Notes
- Start typing your research notes here...

---

## Full Paper Text
2014 IEEE International Conference on Robotics & Automation (ICRA)  
Hong Kong Convention and Exhibition Center  
May 31 - June 7, 2014. Hong Kong, China  
  
  
  
Stability and control of a quadrocopter despite the complete loss of one,  
                         two, or three propellers  
                                            Mark W. Mueller and Raffaello D’Andrea  
  
  
   Abstract— This paper presents periodic solutions for a                    is used, allowing for straight-forward analysis of the system’s  
quadrocopter maintaining a height around a position in space                 controllability characteristics, and for controller synthesis.  
despite having lost a single, two opposing, or three propellers.                The control strategy presented here for a single failed  
In each case the control strategy consists of the quadrocopter  
spinning about a primary axis, fixed with respect to the vehicle,            propeller is broadly similar to that of [14] in that the vehicle  
and tilting this axis for translational control. A linear, time-             rotates freely about an axis, but differs in that the designer  
invariant description of deviations from the attitude equilibrium            has an additional degree of freedom, to choose a ratio of the  
is derived, allowing for a convenient cascaded control design.               forces produced in equilibrium.  
The results for the cases of losing one and two propellers                      The methods presented in this paper could also be applied  
are validated in experiment, while the case of losing three  
propellers is validated in a nonlinear simulation. These results             to design novel, rotating body vehicles. Such vehicles could  
have application in multicopter fault-tolerant control design,               be designed using as few as one propeller, and would thus  
and also point to possible design directions for novel flying                be cheaper to produce than a quadrocopter, at the cost of not  
vehicles.                                                                    being able to control the full vehicle attitude. When equipped  
                                                                             with a camera, such a vehicle could be used as a low-cost  
                       I. I NTRODUCTION                                      omnidirectional flying camera, similar to e.g. [15] or [16] .  
                                                                                This paper is organised as follows: Section II presents the  
   Multicopters have found broad use as research platforms,  
                                                                             equations governing the quadrocopter dynamics. Section III  
used e.g. for vision based pose estimation with quadro-  
                                                                             then presents periodic solutions to the equations of motion  
copters [1] and hexacopters [2], and also as platforms  
                                                                             for the three different loss cases, while Section IV investi-  
allowing for new capabilities. For example, the use of  
                                                                             gates under which conditions the system is controllable about  
both quadrocopters and hexacopters for whale monitoring  
                                                                             these solutions. These results are then validated in Section V,  
is investigated in [3], and hexacopters are used in [4] for  
                                                                             and the paper concludes with an outlook in Section VI.  
weed research; a team of quadrocopters is used to carry a  
slung load in [5] and an octocopter is used to calibrate radio                                   II. E QUATIONS OF MOTION  
telescope antennae in [6].  
                                                                                The equations governing the motion of a quadrocopter are  
   Amongst others, a motivation for using a multicopter with  
                                                                             derived in this section. First, the translational and rotational  
six or more propellers, instead of a four propeller quadro-  
                                                                             dynamics are presented, followed by some simplifications  
copter, is that the vehicle is able to maintain normal flight if  
                                                                             for the sake of tractability. The kinematics of the reduced  
one of the propellers fails (see e.g. [7] for a hexacopter design  
                                                                             attitude, which is used later to describe the equilibria of  
and [8] for an octocopter rotor failure strategy). A survey on  
                                                                             the system and to design the controllers, are presented next.  
research on fault detection and diagnosis and fault-tolerant  
                                                                             Boldface symbols like g are used throughout this paper to de-  
control strategies for unmanned rotary wing vehicles is given  
                                                                             note three-dimensional vectors, while non-boldface symbols  
in [9], and an example of currently available commercial  
                                                                             like m will generally be used for scalars, with exceptions  
solutions are the emergency parachutes of [10].  
                                                                             made explicit. The short-hand notation ω B = (p, q, r) will  
   Partial failure of a quadrocopter actuator is investigated for            be used to denote the elements p, q, and r of the vector ω B .  
example in [11], [12] and [13]. A complete propeller failure  
for a quadrocopter is investigated in [14], where the strategy  
is to give up controlling the vehicle’s yaw angle, and use the  
remaining propellers to achieve a horizontal spin.  
   This paper presents periodic solutions for a quadrocopter  
experiencing one, two opposing, or three complete rotor  
failures. The strategy employed is to define an axis, fixed  
with respect to the vehicle body, and have the vehicle rotate  
freely about this axis. By tilting this axis, and varying the  
total amount of thrust produced, the vehicle’s position can be  
controlled. A linear, time invariant description of the attitude  
  
  The authors are with the Institute for Dynamic Systems and Control,  
ETH Zurich, Sonneggstrasse 3, 8092 Zurich, Switzerland.                      Fig. 1. A quadrocopter in controlled flight despite having lost one complete  
{mullerm, rdandrea}@ethz.ch                                                  propeller.  
  
978-1-4799-3684-7/14/$31.00 ©2014 IEEE                                  45  
                                                                                        The differential equation governing the evolution of the  
                                                                                      body’s angular velocity is now [17]  
                                                                                                                                        4  
                                                                                                                                        X  
                                                                                                                           I B ω̇ B +         I P ω̇ Pi +  
                                                                                                                                        i=1  
                                                                                                                 4  
                                                                                                                                                 !             (4)  
                                                                                                                 X  
                                                                                               B        B   B              P   B        Pi  
                                                                                                                                               
                                                                                            Jω ×K I ω +                I       ω +ω                  = τres  
                                                                                                                 i=1  
  
                                                                                      with τres the resultant torque acting on the body. The first two  
Fig. 2. (A) A dynamic model of a quadrocopter with four propellers  
                                                                                      terms are the time derivative of the body rates and propeller  
arranged symmetrically about the vehicle centre of mass, showing a body-              speeds, respectively, and evaluate to  
fixed reference frame consisting of the directions x, y and z. Propellers 1  
                                                                                                        I B ω̇ B = Ixx  
                                                                                                                     B      B       B  
                                                                                                                                            
and 3 rotate in the opposite sense of propellers 2 and 4. A drag torque τd                                             ṗ, Iyy q̇, Izz ṙ           (5)  
acts to oppose the vehicle’s angular velocity ω B expressed in the body-                                 P Pi              P  
                                                                                                                                   
fixed frame as ω B = (p, q, r). The vehicle has a weight force mg. (B)                                 I ω̇ = 0, 0, Izz ω̇i .                       (6)  
shows a detail of a propeller i rotating at angular velocity ωi with respect  
to the body. Each propeller produces a thrust force fi and torque τi , both              The second term of (4) expresses the cross-coupling of the  
in the direction of the propeller’s axis of rotation. As drawn in (B), ωi > 0,        angular momentum in the system, due to taking the derivative  
and τi < 0.  
                                                                                      in a non-inertial frame. Multiplying out the term yields  
                                                                                                                   4  
                                                                                                                                       !  
                                                                                                                  X  
                                                                                              Jω B ×K I B ω B +       I P ω B + ω Pi  
                                                                                                                                       
A. Dynamics                                                                                                                              =  
                                                                                                                 i=1  
   Fig. 2 shows a quadrocopter, with four propellers, and a                                                    T     T  
                                                                                                                                  P  
                                                                                                                                                              (7)  
total mass m. Five forces act on the vehicle: the weight mg,                                                   Izz − Iyy   qr + Izz  
                                                                                                                                    qωΣ  
                                                                                                                 T     T            P  
and the four propeller forces of magnitude fi which act in the  
                                                                                                             − Izz − Ixx    pr − Izz pωΣ   
                                                                                                                     T       T  
body-fixed direction z = (0, 0, 1) as defined in the figure.                                                        Iyy − Ixx pq  
Additionally, five torques act on the vehicle: one for each                           having introduced the total inertias Ixx     T     B  
                                                                                                                                      = Ixx      P  
                                                                                                                                            + 4Ixx ,  
propeller (captured by the scalar τi ) and a drag torque τd .                          T        B      P    T     B       P  
                                                                                      Iyy = Iyy + 4Ixx , Izz = Izz + 4Izz , and the sum of motor  
The propeller torques oppose the propellers’ rotation, and the                        speeds ωΣ = ω1 + ω2 + ω3 + ω4 .  
vehicle drag torque τd opposes the vehicle’s angular velocity.                           τres , on the right hand side of (4), represents all the  
Expressed in a body-fixed reference frame, the vehicle’s                              moments acting upon the body, which consist of the torques  
angular velocity is ω B = (p, q, r). The rotation of the body-                        produced by the motors, and the moments due to the rotor  
fixed frame with respect to some inertial frame is described                          forces. The centre of each rotor, through which the forces are  
by the rotation matrix R.                                                             assumed to act, is taken to lie at distance l from the centre  
   The position of the quadrocopter’s centre of mass, ex-                             of mass, such that  
pressed in the inertial frame, is denoted d = (d1 , d2 , d3 ).                                                                        
Then, the quadrocopter’s translational dynamics are:                                                            (f2 − f4 ) l + τdx  
                                                                                                     τres =  (f3 − f1 ) l + τdy                (8)  
                                     4  
                                     X                                                                       τ1 + τ2 + τ3 + τ4 + τdz  
                       md¨ = Rz             fi + mg                       (1)                                    
                                      i=1                                             with τd = τdx , τdy , τdz the components of the drag torque.  
with g = (0, 0, −9.81) m s−2 the acceleration due to gravity.                         B. Further simplifying assumptions  
   The vehicle is assumed to consist of five rigid bodies:  
the vehicle body as such, and the four propellers (which                                 There is a strong linear relationship between a propeller’s  
are taken to include any rotating part of the motors). The                            reaction torque and thrust force (characterised by the coeffi-  
vehicle body inertia tensor I B is assumed to be diagonal                             cient κτ with the sign given by the sense of rotation). The  
when expressed in the body-fixed frame:                                               thrust force of a stationary propeller is proportional to the  
                        B                                                           angular velocity squared with coefficient κf , such that [18]  
                         Ixx 0       0  
                 I B =  0 Iyy  B  
                                     0 .                (2)                                                τi = (−1)i+1 κτ fi                                 (9)  
                                      B  
                          0     0 Izz                                                                       fi = κf ωi2 .                                     (10)  
   It is assumed that each propeller can be treated as a                                For simplicity, the aerodynamic drag acting on the quadro-  
disk that is symmetric about its axis of rotation. This                               copter is assumed to act only to oppose the yaw rate r, with  
means that the propeller’s rotational inertia expressed in the                        proportionality constant γ > 0 such that  
quadrocopter-fixed frame is independent of the orientation of  
the propeller, and equals                                                                                    τd = (0, 0, −γr)                                 (11)  
                         P                 
                          Ixx 0        0                                              with the assumption of a linear drag term supported by  
                  I P =  0 Ixx  P  
                                       0 .                (3)                        experimental data. The fact that a term linear in the speed  
                                        P                                             dominates, instead of a quadratic term, can possibly be  
                           0     0 Izz  
                                                                                 46  
explained by the asymmetric relative air velocity over the               with the norm constraint on n expressed by  
advancing and retreating propeller blades (an effect exploited  
                                                                                                kn̄k =  ω̄ B = 1                        (17)  
in a different context in [19]).  
   It is assumed that the propeller speeds are controlled                where k·k is the Euclidean norm.  
by high bandwidth motors, such that the motor speed is                      The primary axis is taken to point opposite to gravity  
unaffected by the vehicle motion. Furthermore, it is assumed             along the periodic solution, and the vehicle is required to  
that I P  I B , such that the term I P ω̇ can be neglected.             not accelerate in the direction of gravity. The fraction of the  
Note that the angular momentum of the rotors might still be              total thrust force f¯Σ = f¯1 + f¯2 + f¯3 + f¯4 pointing opposite  
comparable to that of the body, such that the term I P ω Pi              to gravity is thus n̄z , such that  
may not be negligible compared to I B ω B . The quadrocopter                                      f¯Σ n̄z = m kgk .                 (18)  
                                                 B      B  
body is assumed to be symmetric, such that Ixx       = Iyy .  
   Expanding (4) with (2)-(3), (5)-(8) and applying the above            If n̄z < 1, a part of the total thrust force will point  
assumptions yields the following three differential equations            perpendicular to gravity, imparting an acceleration of the  
for the vehicle’s body rates:                                            vehicle in this direction. In this case, the vehicle will move  
                                                                         along a horizontal circular trajectory with a period of  
                              B  
                                ṗ = κf ω22 − ω42 l−  
                                                   
                            Ixx  
         T      T  
                         P  
                                                             (12)                                          2π  
        Izz − Ixx   qr − Izz q (ω1 + ω2 + ω3 + ω4 ) ,                                             Tps =                             (19)  
                                                                                                          kω̄ B k  
                              B  
                                q̇ = κf ω32 − ω12 l+  
                                                   
                            Ixx  
                                                             (13)        and a radius of  
         T      T         P  
                    
       Izz  − Ixx   pr + Izz p (ω1 + ω2 + ω3 + ω4 ) ,                                               p  
                                                                                                      1 − n̄2z kgk  
         B  
       Izz ṙ = −γr + κτ κf ω12 − ω22 + ω32 − ω42 .  
                                                     
                                                             (14)                          R̄ps =                     2.            (20)  
                                                                                                       n̄z     kω̄ B k  
C. Reduced attitude kinematics                                              The solutions will now be presented for each case of a  
   A typical quadrocopter controller (see e.g. [20]) allows              quadrocopter losing one propeller, two opposing propellers,  
for control of the quadrocopter’s full attitude R to some                and three propellers. This involves solving for the eleven  
desired attitude as part of the control strategy. The strategy           unknowns n̄x , n̄y , n̄z , p̄, q̄, r̄, , ω̄1 , ω̄2 , ω̄3 and ω̄4 by  
adopted herein is to give up control of the full attitude once           utilising the eight algebraic equations (12) - (14) (with the  
a propeller has failed, and instead control only a single                angular accelerations set to zero) and (16) - (18).  
direction of the attitude (or, the attitude to within one degree            The symmetry properties of the quadrocopter mean that  
of freedom), often referred to as a reduced attitude [21],               the equilibrium yaw rate is independent of the pitch and roll  
which can be described by a unit vector.                                 rates from (14), and can be solved independently as  
   The differential equation governing the evolution of a unit                              κτ κf  
                                                                                                     ω̄12 − ω̄22 + ω̄32 − ω̄42 .  
                                                                                                                                 
                                                                                       r̄ =                                               (21)  
vector stationary in the inertial frame but expressed in the                                  γ  
body-fixed frame as n = (nx , ny , nz ) is given by the cross               Each lost motor/propeller will add a constraint of the form  
product                                                                  ωi = 0, and the solutions for each of the loss cases will be  
                       ṅ = −ω B × n.                       (15)         considered in more detail below.  
        III. P ERIODIC SOLUTIONS AND EQUILIBRIA                          A. Solution with one lost propeller  
    This section presents periodic solutions for the position               Without loss of generality it will be assumed that propeller  
and attitude of a quadrocopter experiencing the loss of one,             4 has failed, such that f4 = τ4 = 0, and specifically that  
two (opposing) or three propellers. For the case of one                  ω̄4 = 0, leaving two degrees of freedom for solving for the  
and three lost propellers, the resultant forces and torques              periodic solution. An intuitive way of specifying them is for  
acting on the vehicle will not be zero, such that no static              the two opposing propellers to produce equal thrust (thus  
equilibrium exists for the vehicle’s centre of mass. However,            f¯1 = f¯3 ), and choosing a ratio ρ = f¯2 /f¯1 between the thrust  
in each case the reduced attitude of the vehicle is constant,            of propellers 1 and 2, such that ρ becomes a tuning factor.  
such that the controllability of the system’s attitude can be            This leaves eleven nonlinear equations to solve for eleven  
investigated with methods well-established for linear, time-             unknowns.  
invariant, systems in Section IV. An overbar will be used to                From (13), one solution is p̄ = 0 and thus n̄x = 0, for all  
express values constant along the periodic solution (e.g. p̄).           choices of ρ. For small ρ, as ρ grows, n̄z decreases and thus  
    The goal is to find periodic solutions in which a constant           the total force required increases by (18). The radius R̄ps  
primary axis n̄ exists about which the vehicle rotates with              of the horizontal orbit (20) will be zero at ρ = 0, but the  
constant angular velocity ω̄ B . The primary axis is fixed with          relationship between the angular velocity and ρ is harder  
respect to the vehicle body, and is expressed in the body fixed          to predict, and numerical results are given for a specific  
frame as n̄ = (n̄x , n̄y , n̄z ). From (15), the requirement that        quadrocopter in Section V, specifically Fig. 3. Note that for  
n̄˙ = 0, and the properties of the cross product, it follows             ρ = 0 the two-propeller solution of Section III-B, below,  
that                                                                     is recovered, while as ρ tends to infinity the single propeller  
                                                                         solution of Section III-A is recovered (with instead propellers  
                           n̄ =  ω̄ B                      (16)         1, 3 and 4 taken as failed).  
  
                                                                    47  
B. Solution with two lost propellers                                     and introducing an input u, which enters the system through  
   It is assumed that two opposing propellers have failed,               the matrix B. The definition of B and u will be deferred to  
taken without loss of generality to be 2 and 4. Note that the            the sections below, depending on the number of remaining  
case of two adjacent propellers failing will not be addressed            propellers.  
here, and is a topic to be investigated in future work.                  A. Control with one lost propeller  
   Setting ω̄2 = ω̄4 = 0 leaves one degree of freedom,  
which can be resolved by requiring that the remaining motors               With three propellers remaining, the input vector u =  
produce equal thrust, i.e. ω̄1 = ω̄3 . The equilibrium can now           (u1 , u2 ) is introduced as a function of the deviations of the  
be solved for as follows:                                                actual motor forces from the equilibrium, with units of force  
                                                                                            u1 = f3 − f¯3 − f1 − f¯1  
                                                                                                                        
                                 1                                                                                                  (29)  
                    f¯1 = f¯3 = m kgk                      (22)  
                                 2s                                                         u2 = f2 − f¯2 .  
                                                                                                            
                                                                                                                                    (30)  
                                       m kgk  
                   ω̄1 = ω̄3 = −                           (23)          The remaining degree of freedom is resolved by specifying  
                                        2κf  
                                                                       that the total thrust matches the desired thrust:  
                                  κτ m kgk  
                  ω̄ B = 0, 0,                             (24)                           f1 + f2 + f3 = f¯1 + f¯2 + f¯3 .               (31)  
                                       γ  
                     n̄ = (0, 0, 1) .                      (25)          The system (26) is expanded to include these inputs as  
For the two-propeller case, R̄ps = 0 and the vehicle will                                         s̃˙ = As̃ + B(3) u                     (32)  
be stationary at a point in space with its z axis pointing                                                    
                                                                                                              0 1  
                                                                                                                     
vertically.                                                                                              l 1 0  
                                                                                               B(3) = B           .                    (33)  
C. Solution with three lost propellers                                                                  Ixx 0 0  
                                                                                                              0 0  
   Propellers 2, 3 and 4 are taken to have failed, again without  
loss of generality, such that the system is fully constrained.           Examining    the rank of the controllability           matrix  
                                                                         C(3) = B(3) AB(3) A2 B(3) A3 B(3) it is easy to  
                                                                                                                     
From (12) follows that one solution is for q̄ = n̄y = 0, and  
then p̄ 6= 0 and thus n̄x 6= 0. The solution for a specific              show that (32) is controllable if l 6= 0 and n̄z 6= 0.  
vehicle is given in Section V.  
                                                                         B. Control with two lost propellers  
                    IV. C ONTROLLABILITY                                    Having lost two opposing propellers, the system has a  
   It is shown below that the vehicle’s reduced attitude is              single input u1 , defined as in (29), which is now added to  
controllable near the equilibrium solutions of the preceding             (26) to give  
section. This is done by exploiting the time invariant nature  
                                                                                                   s̃˙ = As̃ + B(2) u                    (34)  
of the attitude equilbria and linearising about them, and                                                       
examining the rank of the controllability matrix [22]. The                                                     0  
state vector s = (p, q, nx , ny ) is introduced to describe the                                           l   1  
                                                                                                                .  
                                                                                                B(2) = B                                (35)  
vehicle’s reduced attitude, and the conditions under which                                               Ixx 0  
                                                                                                               
this reduced attitude is controllable are derived below for                                                    0  
the three different propeller loss cases discussed in this paper.        Again, the total thrust produced must match the commanded  
The actual design and implementation of a controller for a               thrust  
specific vehicle is deferred until Section V. Given that the  
total vehicle thrust can be specified, and that simultaneously                                  f1 + f3 = f¯1 + f¯3 .                    (36)  
the direction of the vehicle’s thrust can be controlled, the  
                                                                           The  two propeller system (34) is                controllable if  
vehicle’s acceleration can be controlled (at least quasi-  
                                                                         C(2) = B(2) AB(2) A2 B(2) A3 B(2) has full rank, or  
statically) and thus also its position.  
                                                                         equivalently if the determinant of C(2) is non-zero, i.e.  
   The attitude deviation from the equilibrium is written as  
                                                                                                                       4  
s̃ = s − s̄, and will evolve to first order as  
                                                                                                                   
                                                                                                               2    l  
                                                                                          ā r̄ n̄2z (ā + r̄)      B  
                                                                                                                          6= 0.           (37)  
              s̃˙ = As̃ + Bu                                (26)                                                   Ixx  
                                                                         Combining this with (24), and assuming l 6= 0 this leaves  
                                                      
                           0           ā    0     0  
                ∂ ṡ    −ā           0     0     0                                                          2  
             A=        =                                  (27)                                ā n̄2z (ā + r̄) 6= 0.                  (38)  
                ∂s s=s̄  0           −n̄z   0     r̄   
                          n̄z          0     −r̄   0                     Note that for the two-propeller case, by (25), nz = 1. Thus  
defining the coupling constant ā as                                     the system is uncontrollable if ā = 0, or  
                                                                                             T      T          P  
                                                                                                        
              T  
             Ixx    T  
                 − Izz       P  
                            Izz                                                             Ixx − Izz   r̄ = 2Izz ω̄1            (39)  
      ā =       B  
                       r̄ −  B  
                                (ω̄1 + ω̄2 + ω̄3 + ω̄4 )    (28)  
                Ixx         Ixx                                          in which case the vehicle roll rate p is uncontrollable.  
  
                                                                    48  
  If instead ā + r̄ = 0, or                                            B. Translational controller  
                  T  
                 Ixx    B  
                     + Ixx      T  
                                       P  
                           − Izz r̄ = 2Izz ω̄1             (40)            When using fewer than four propellers, only the direction  
                                                                        of the primary axis n is controlled, in contrast to the nominal  
the linearised system has two uncontrollable modes corre-               quadrocopter case where the full attitude R is controlled. The  
sponding to p + ānx and ny .                                           direction of the primary axis, and the total thrust that the  
C. Control with three lost propellers                                   vehicle produces, affect the vehicle acceleration through (1).  
                                                                        A cascaded control strategy is designed, with a fast inner  
   The description of the linearised attitude system when               loop controlling the vehicle’s reduced attitude, and a slow  
using only a single propeller is the same as that for two               outer loop controlling the vehicle’s position.  
propellers (see Section IV-B) except that now f3 = 0                       The translational deviation of the quadrocopter from a  
and ω3 = 0. The controllability requirement is then the                 desired point in space is written as d. The goal of the  
same as (37). Noting that n̄z = 0 is not a physically                   translational controller is to make this deviation behave like  
meaningful possibility as the total force given by (18) would           a second-order system, with damping ratio ζ and natural  
then be undefined, the linearised quadrocopter attitude is not          frequency ωn , by introducing the desired acceleration d¨des  
controllable with a single propeller if either of the two below         such that  
equalities hold:                                                                         d¨des + 2ζωn d˙ + ωn2 d = 0.              (44)  
                         T     T           P  
                                    
                        Izz − Ixx   r̄ = Izz ω̄1            (41)  
                                                                        The damping ratio was set to ζ = 0.7, and ωn was chosen for  
                  T      T     B           P  
                                    
                 Izz − Ixx − Ixx r̄ = Izz ω̄1 .             (42)        each of the cases such that the translational system responds  
                                                                        much slower than the attitude system. The direction of the  
  Note that in the one-propeller case the total thrust will vary  
                                                                        instantaneously desired direction of the primary axis ndes ,  
with u1 as this can not be specified as in (31) and (36).  
                                                                        and the total thrust produced fΣ , are defined as  
                       V. VALIDATION                                                                                    
                                                                                       ndes n̄z fΣ = mR−1 d¨des − g              (45)  
  The preceding analysis is validated in experiment and  
simulation in this section. A cascaded control design is                                      fΣ = f1 + f2 + f3 + f4 .             (46)  
implemented for each case of a quadrocopter losing one,  
                                                                        This can be easily adapted, so that the horizontal and  
two opposing, or three propellers, with a slow outer loop  
                                                                        vertical degrees of freedom have different natural frequen-  
controlling the vehicle’s position, and a fast inner loop  
                                                                        cies/damping ratios.  
controlling the reduced attitude. An LQR controller [23] is  
                                                                           The fast inner controller, as described in Section IV, must  
implemented in each case for the inner loop control.  
                                                                        then make the vehicle’s primary axis align with ndes while  
A. Experimental platform                                                producing a total thrust fΣ . Note that for d¨des = 0, the  
   The work was validated using quadrocopters based on the              solutions of Section III are recovered.  
Ascending Technologies Hummingbird [24], in the Flying                  C. Motor time constants  
Machine Arena [25], which will be referred to here simply  
as “the quadrocopter”. The attitude controller of Section IV               It was found in experiment that the motor dynamics have a  
is executed at 1000 Hz on board the vehicle, while the                  large influence on the system behaviour. For this reason, the  
translational controller presented below is executed at 50 Hz.          reduced attitude system (26) of Section IV was extended as  
   The quadrocopter’s inertia was measured by measuring its             follows. The currently produced force deviations were added  
period of oscillation when suspended around three different             as states, each tracking its respective command as a first order  
axes, and was found to be Ixx  T  
                                  = 3.2 × 10−3 kg m2 , Izz  
                                                         T  
                                                            =           system with time constant σmot . Thus the linearised system  
          −3     2  
5.5 × 10 kg m . The propeller inertia was estimated by                  for i propellers now becomes  
approximating the propeller and motor rotor as disks and                                    ṡe = Ae se + B(i)e ue                 (47)  
cylinders, respectively, to get IzzP  
                                      = 1.5 × 10−5 kg mm2 ,                                                      
                                                  P                                                A     B(i)  
while the other propeller inertia was neglected Ixx  = 0. The                               Ae =           −1                      (48)  
                                                                                                    0 −σmot   1  
vehicle’s mass was measured to be 0.50 kg, and the distance                                               
from the centre of mass to the centre of the propellers is                                            0  
                                                                                            Be = −1                                (49)  
l = 0.17 m.                                                                                        σmot 1  
   The propellers used were characterised using a force-                with 0 and 1 being zero and identity matrices of the  
torque sensor, and the thrust and reaction torque coeffi-               appropriate dimension, respectively.  
cients were estimated as κf = 6.41 × 10−6 Ns2 /rad2 and                   The time constant for the quadrocopter was estimated to  
κτ = 1.69 × 10−2 Nm/ N, respectively. The propellers are                be σmot = 0.015 s.  
able to produce thrust forces in the range  
                                                                        D. Implementation for one lost propeller  
                       fi ∈ [0.2, 3.8] N.                  (43)  
                                                                          The equilibrium for the quadrocopter with three pro-  
  The quadrocopter’s drag coefficient was estimated to be               pellers for different values of ρ = f¯2 /f¯1 is investigated  
γ = 2.75 × 10−3 N m s rad−1 .                                           in Fig. 3. The absolute body rate has a minimum of  
  
                                                                   49  
Fig. 3.     The equilibrium state for the quadrocopter when using three                 Fig. 4. Experimental results for a quadrocopter at hover, with propeller 4  
propellers, as a function of the ratio ρ = f¯2 /f¯1 , showing from top to bottom        disabled at time 0. The first three plots show the two horizontal directions,  
the angular velocity, the direction of the primary axis, the thrust forces and          and the vertical. The desired position setpoint is shifted hoizontally by 1 m  
the radius of the horizontal motion. Note that by design f¯3 = f¯1 . As the             after 7.7 s, and the height setpoint is set to zero after 19.8 s, and the vehicle  
force f¯2 increases, it can be seen that the primary axis n̄ of the vehicle             lands at 21.5 s. Initially, the vehicle only uses two opposing propellers to  
initially moves farther away from the body z axis, while the resultant pitch            build up angular momentum, and the controller is enabled when the vehicle  
rate q̄ increases. Note the discontinuity at ρ = 2, when r̄ = n̄z = 0, and              angular velocity exceeds 10 rad s−1 (at 0.62 s). The oscillatory motion of  
after which the sense of rotation reverses. At ρ = 0 the solution is that of            the vehicle about the setpoint can be seen clearly on the top plot.  
the two-propeller case, while for large ρ the solution approaches that of the  
single propeller case.  
                                                                                        position, also when the position is shifted 1 m horizontally,  
                                                                                        and also allows the vehicle to perform a soft landing.  
  ω̄ B = 19.0 rad s−1 at ρ = 0.655, while the propeller force                             When maintaining a position, the vehicle’s state is approx-  
f1 has a minimum in the region ρ ∈ [0, 2] of f¯1 = 2.04 N  
 ¯  
                                                                                        imately as below:  
at ρ = 0.563, with both minima being shallow. A value  
of ρ = 0.5 was chosen for the implementation, placing                                                    f1 = 2.2 N, f2 = 2.1 N, f3 = 0.8 N                        (54)  
all steady-state thrust forces some distance away from the                                                 B                               −1  
                                                                                                       ω       = (2.1, 3.7, 13.7) rad s                            (55)  
saturation values (43). In this case, the attitude equilibrium  
                                                                                                           n = (0.14, 0.26, 0.96) .                                (56)  
values are  
                                                                                           Fig. 1 shows a quadrocopter in flight with one lost  
                    f¯1 = f¯3 = 2.05 N, f¯3 = 1.02 N                      (50)  
                                                                                        propeller, and the video accompanying this paper shows a  
                  ω̄ B = (0, 5.69, 18.89) rad s−1                         (51)          quadrocopter taking off despite the loss of one propeller,  
                    n̄ = (0, 0.289, 0.958)                                (52)          translating 2 m horizontally, and then landing.  
                 R̄ps = 8 mm.                                             (53)  
                                                                                        E. Implementation for two lost propellers  
   An LQR controller was designed on the extended system,                                 The equilibrium condition for the quadrocopter flying with  
with a diagonal state and input cost matrices, with the cost                            two opposing propellers with equal equilibrium thrusts is  
value of 1 s2 rad−2 on the angular rates, 20 on the deviation                           calculated by (22) - (25) as  
from the primary axis, zero on the extended motor states,  
and 1 N−2 on the inputs. The translational controller natural                                                    f¯1 = f¯3 = 2.45 N                                (57)  
frequency was set to ωn = 1 rad s−1 .                                                                            ω̄1 = ω̄3 = −619 rad s       −1  
                                                                                                                                                                   (58)  
   The resulting controller was implemented on the system,                                                      ω̄ B = (0, 0, 30.1) rad s−1                        (59)  
and results are shown in Fig. 4 for a quadrocopter starting at  
hover at a height of 2 m. At time 0, the fourth propeller                                  Again, an LQR controller was designed on the extended  
is disabled, and the vehicle initially uses only the two                                two-propeller system, with a diagonal state and input cost  
opposing propellers to produce an angular velocity about                                matrices, with the cost value of 0 on the angular rates,  
the z axis. Once this angular velocity exceeds 10 rad s−1 ,                             1000 on the deviation from the primary axis along x, 2  
the LQR controller is used to control the vehicle (at 0.62 s).                          on the deviation from the primary axis along y, zero on  
The control strategy stabilises the vehicle around the desired                          the extended motor states, and 0.75 N−2 on the inputs. The  
  
                                                                                   50  
Fig. 5.    Experimental results for a quadrocopter flying with only two  
propellers. The quadrocopters starts at rest, on the ground, at time zero.          Fig. 6. Nonlinear simulation results for a quadrocopter flying with only one  
After 5.6 s the horizontal setpoint is shifted by 1 m. Note that the vehicle        propeller. The simulation is started with the vehicle at the reduced attitude  
does not reach the desired height, and has a height error of approximately          solution of Section III-C, but with a 1 m offset both in the horizontal and  
2.4 m.                                                                              vertical directions. The controller successfully reduces the horizontal error,  
                                                                                    but a vertical offset of 0.75 m remains. Note that the maximum producible  
                                                                                    thrust in this simulation was set to double that of the true system.  
translational controller was left as for the single propeller  
loss case: ωn = 1 rad s−1 .  
   The resulting controller was implemented on the system,                          quadrocopter. Furthermore, when starting from a hover, a  
and results are shown in Fig. 5 for a quadrocopter taking                           vehicle losing propellers 2, 3 and 4 will have a much stronger  
off from the ground with only two propellers. After 5.6 s,                          tendency to accelerate about the y axis than the z axis  
the horizontal setpoint is shifted by 1 m. Notable is that                          (because l  κτ ). This implies that it is non-trivial to bring  
the vehicle has a large steady-state offset in the height,                          a vehicle from a stationary state to a state sufficiently close  
indicating that the propellers produce significantly less thrust                    to equilibrium for a linear control strategy to be effective.  
than expected when the body is rotating at high angular                                A nonlinear simulation was used to validate the single  
velocities. In practise, this could be solved by implementing                       propeller case. The simulator implemented the dynamic  
an integral controller on the vehicle height, or by more                            equations of Section II with the properties of the quadro-  
accurately modelling the behaviour of the propellers when                           copters as given above, except that the maximum thrust force  
rotating. When maintaining a position, the following states                         limit was doubled to 7.6 N. In the simulation, the simulated  
were measured:                                                                      motor speeds track the commanded speeds as first order  
                                                                                    systems with time constant 0.015 s.  
                  f1 = 2.98 N, f3 = 3.10 N                            (60)             The LQR controller was designed with a cost of 1 s2 rad−2  
                    B                                  −1  
                ω        = (0.22, −0.72, 27.1) rad s        .         (61)          on the angular rates, 100 on the attitude deviations, zero on  
                                                                                    the extended motor state, and 4 N−2 on the input. The trans-  
  A quadrocopter taking off with two propellers, translating  
                                                                                    lational controller was parametrised with ωn = 0.5 rad s−1 .  
two metres horizontally, and then landing can be seen in the  
                                                                                       The simulation was started with the vehicle’s attitude at  
accompanying video.  
                                                                                    the periodic solution, but with a 1 m horizontal error, and  
F. Implementation for three lost propellers                                         a 1 m vertical error. The results are shown in Fig. 6 – the  
                                                                                    vehicle is brought to the correct horizontal position, but the  
  When utilising only a single propeller, the equilibrium  
                                                                                    vehicle height shows a steady state offset of approximately  
condition was calculated as  
                                                                                    0.75 m.  
                  f¯1 = 5.37 N, ω̄1 = −915 rad s−1                    (62)  
                     B                           −1                                 G. Discussion  
                ω̄       = (14.7, 0, 33.0) rad s                      (63)  
                     n̄ = (0.41, 0, 0.91) .                           (64)            The implemented two-propeller solution uses only slightly  
                                                                                    more thrust per propeller than the three propeller solution,  
  Because the required force f¯1 exceeds the thrust limits                          but has significantly less total thrust at the periodic solution  
(43) this equilibrium could not be implemented on the                               (due to using only two propellers, and having n̄z = 1) and  
  
                                                                               51  
could thus be expected to be more energy efficient. However,                        [5] J. Fink, N. Michael, S. Kim, and V. Kumar, “Planning and control  
the three propeller case has two independent inputs on the                              for cooperative manipulation and transportation with aerial robots,”  
                                                                                        The International Journal of Robotics Research, vol. 30, no. 3, pp.  
attitude system compared to one independent input for the                               324–334, 2011.  
two-propeller case – the attached video shows a close-up                            [6] J. R. Horandel, S. Buitink, A. Corstanje, J. E. Enriquez, and H. Falcke,  
of the two cases, and it can be clearly seen that the three-                            “The LOFAR radio telescope as a cosmic ray detector,” in Interna-  
                                                                                        tional Cosmic Ray Conference, 2013.  
propeller solution remains closer to the setpoint than the                          [7] D. Scaramuzza, M. Achtelik, L. Doitsidis, F. Fraundorfer, E. Kos-  
two-propeller solution. This effect can also be seen when                               matopoulos, A. Martinelli, M. Achtelik, M. Chli, S. Chatzichristofis,  
comparing Fig. 4 and Fig. 5. A future topic of research                                 L. Kneip et al. (2013) Vision-controlled micro flying robots: from  
                                                                                        system design to autonomous navigation and mapping in gps-  
will be to quantify the sensitivity of the system to noise                              denied environments. [online] http://robotics.ethz.ch/∼scaramuzza/  
for different values of ρ.                                                              Davide Scaramuzza files/publications/pdf/IEEE RAM submitted.pdf.  
                                                                                    [8] A. Marks, J. F. Whidborne, and I. Yamamoto, “Control allocation for  
                         VI. O UTLOOK                                                   fault tolerant control of a VTOL octorotor,” in UKACC International  
                                                                                        Conference on Control. IEEE, 2012, pp. 357–362.  
   This paper presents equilibrium states and controllers that                      [9] Y. Zhang, A. Chamseddine, C. Rabbath, B. Gordon, C.-Y. Su,  
allow a quadrocopter to maintain a position in space after                              S. Rakheja, C. Fulford, J. Apkarian, and P. Gosselin, “Development of  
losing one, two (opposing), or three propellers. The strategy                           advanced FDD and FTC, techniques with application to an unmanned  
                                                                                        quadrotor helicopter testbed,” Journal of the Franklin Institute, 2013.  
in each case is to have the vehicle rotate freely about an axis,                   [10] (2013,       September)       Multicopter,       quadcopter,      drone,  
fixed with respect to the body. The remaining motor forces                              RC       aircraft     recovery     and       rescue     chutes.     [on-  
are then used to rotate this axis in inertial space, which allows                       line] http://www.fruitychutes.com/uav rpv drone recovery parachutes/  
                                                                                        multicopter quadcopter-rc aircraft recovery and rescue chutes.htm.  
the vehicle to translate in space when combined with the                           [11] M. Ranjbaran and K. Khorasani, “Fault recovery of an under-actuated  
total thrust produced. A cascaded control scheme exploiting                             quadrotor aerial vehicle,” in IEEE Conference on Decision and Con-  
time scale separation was used to control the translation. The                          trol. IEEE, 2010, pp. 4385–4392.  
                                                                                   [12] H. A. Izadi, Y. Zhang, and B. W. Gordon, “Fault tolerant model predic-  
strategy is implemented and validated with data gathered by                             tive control of quad-rotor helicopters with actuator fault estimation,”  
experiment for the cases of losing a single and two opposing                            in IFAC World Congress, vol. 18, no. 1, 2011, pp. 6343–6348.  
propellers, while the case of losing three propellers was                          [13] A. Chamseddine, Y. Zhang, C. A. Rabbath, C. Join, and D. Theil-  
                                                                                        liol, “Flatness-based trajectory planning/replanning for a quadrotor  
validated in a nonlinear simulation.                                                    unmanned aerial vehicle,” IEEE Transactions on Aerospace and Elec-  
   In future work, the authors intend to more accurately                                tronic Systems, vol. 48, no. 4, pp. 2832–2848, 2012.  
characterise the vehicle, specifically the aerodynamic effects                     [14] A. Freddi, A. Lanzon, and S. Longhi, “A feedback linearization  
                                                                                        approach to fault tolerance in quadrotor vehicles,” in IFAC World  
affecting the propellers, and to construct a flying single                              Congress, 2011, pp. 5413–5418.  
propeller vehicle. The authors also intend to implement                            [15] T. Haus, M. Orsag, and S. Bogdan, “Omnidirectional vision based  
the presented results with a fault detection scheme, and                                surveillance with spincopter,” in International Conference on Un-  
                                                                                        manned Aircraft Systems, 2013, pp. 326–332.  
investigate different switching strategies for transitioning                       [16] H. Youngren, S. Jameson, and B. Satterfield, “Design of the samarai  
from e.g. four propellers to three.                                                     monowing rotorcraft nano air vehicle,” in Proceedings of the American  
                                                                                        Helicopter Society AHS 65th Annual Forum and Technology Display,  
                  ACKNOWLEDGEMENTS                                                      2009.  
                                                                                   [17] P. H. Zipfel, Modeling and Simulation of Aerospace Vehicle Dynamics  
  The Flying Machine Arena is the result of                                             Second Edition. AIAA, 2007.  
contributions of many people, a full list of which                                 [18] P. Pounds, R. Mahony, P. Hynes, and J. Roberts, “Design of a  
can be found at http://www.idsc.ethz.ch/                                                four-rotor aerial robot,” in Australasian Conference on Robotics and  
                                                                                        Automation, vol. 27, 2002, p. 29.  
Research DAndrea/FMA/participants.                      The                        [19] P. Martin and E. Salaün, “The true role of accelerometer feedback in  
authors thank Raymond Oung for making available the                                     quadrotor control,” in IEEE International Conference on Robotics and  
propeller characterisation data.                                                        Automation, 2010, pp. 1623–1629.  
                                                                                   [20] R. Mahony, V. Kumar, and P. Corke, “Aerial vehicles: Modeling,  
  This research was supported by the Swiss National Science                             estimation, and control of quadrotor,” IEEE robotics & automation  
Foundation through grant agreement number 138112.                                       magazine, vol. 19, no. 3, pp. 20–32, 2012.  
                                                                                   [21] N. A. Chaturvedi, A. K. Sanyal, and N. H. McClamroch, “Rigid-body  
                            R EFERENCES                                                 attitude control,” Control Systems, IEEE, vol. 31, no. 3, pp. 30–51,  
                                                                                        2011.  
 [1] F. Fraundorfer, L. Heng, D. Honegger, G. H. Lee, L. Meier, P. Tanska-  
                                                                                   [22] F. Callier and C. Desoer, Linear System Theory, ser. Springer Texts  
     nen, and M. Pollefeys, “Vision-based autonomous mapping and explo-  
                                                                                        in Electrical Engineering. Springer, 1994.  
     ration using a quadrotor MAV,” in IEEE/RSJ International Conference  
                                                                                   [23] D. P. Bertsekas, Dynamic Programming and Optimal Control, Vol. II.  
     on Intelligent Robots and Systems. IEEE, 2012, pp. 4557–4564.  
                                                                                        Athena Scientific, 2007.  
 [2] S. Weiss, M. W. Achtelik, S. Lynen, M. C. Achtelik, L. Kneip, M. Chli,  
                                                                                   [24] D. Gurdan, J. Stumpf, M. Achtelik, K.-M. Doth, G. Hirzinger, and  
     and R. Siegwart, “Monocular vision for long-term micro aerial vehicle  
                                                                                        D. Rus, “Energy-efficient autonomous four-rotor flying robot con-  
     state estimation: A compendium,” Journal of Field Robotics, vol. 30,  
                                                                                        trolled at 1 kHz,” in IEEE International Conference on Robotics and  
     no. 5, pp. 803–831, 2013.  
                                                                                        Automation, April 2007, pp. 361–366.  
 [3] W. Selby, P. Corke, and D. Rus, “Autonomous aerial navigation and  
                                                                                   [25] S. Lupashin, M. Hehn, M. W. Mueller, A. P. Schoellig, M. Sherback,  
     tracking of marine animals,” in Proceedings of the 2011 Australasian  
                                                                                        and R. D’Andrea, “A platform for aerial robotics research and  
     Conference on Robotics and Automation. Australian Robotics &  
                                                                                        demonstration: The flying machine arena,” Mechatronics, no. 0, pp. –,  
     Automation Association, 2011, pp. 1–7.  
                                                                                        2014.  
 [4] J. Rasmussen, J. Nielsen, F. Garcia-Ruiz, S. Christensen, and J. C.  
     Streibig, “Potential uses of small unmanned aircraft systems (UAS) in  
     weed research,” Weed Research, 2013.  
  
  
  
  
                                                                              52  
  
