# ©The Author(s) 2015

**Authors:** Unknown_Author
**Year:** 2015
**Source URL:** https://hiperlab.berkeley.edu/wp-content/uploads/2018/05/2015_RelaxedHoverSolutionsForMulticoptersApplicationToAlgorithmicRedundancyAndNovelVehicles.pdf

## Abstract
This paper presents a relaxed definition of hover for multicopters with propellers pointing in a common direction. These solutions are found by requiring that the multicopter remain substantially in one position, and that the solutions be constant when expressed in a coordinate system attached to the vehicle. The vehicle’s angular velocity is then shown to be either zero or parallel to gravity. The controllability of a vehicle’s attitude about these solutions is then investigated. These relaxed hover solutions may be applied as an algorithmic failsafe, allowing for example a quadrocopter to fly despite the complete loss of one, two, or three of its propellers. Experimental results validate the quadrocopter failsafe for two types of failure (a single propeller, and two opposing propellers failing), and a nonlinear simulation validates the remaining two types of failure (two adjacent, and three propellers failing). The relaxed hover solutions are also shown to allow a multicopter to maintain flight in spite of extreme centre of mass offsets. Finally, the design and experimental validation of three novel vehicles is presented. 1. Introduction Multicopters have found broad use as research platforms, used e.g. for vision based pose estimation with quadrocopters (Fraundorfer et al. 2012) and hexacopters (Weiss et al. 2013), and also as platforms allowing for new capabilities. For example, the use of both quadrocopters and hexacopters for whale monitoring is investigated by Selby et al. (2011), and hexacopters are used by Rasmussen et al. (2013) for weed research; a team of quadrocopters is used to carry a slung load by Fink et al. (2011) and an octocopter is used to calibrate radio telescope antennae by Hörandel et al. (2013). Multicopters also hold promise as goods delivery vehicles (D’Andrea 2014). These vehicles typically consist of an even number of propellers all pointing in a common direction, with half of the propellers having the opposite handedness from the remainder, and thus also rotating in the opposite direction. The propellers are then mechanically arranged such that the torques they produce can be made to sum to zero while the propeller

## My Notes
- Start typing your research notes here...

---

## Full Paper Text
                                                                                                    ©The Author(s) 2015  
  
Relaxed hover solutions for multicopters:  
application to algorithmic redundancy and  
novel vehicles  
  
Mark W. Mueller∗ and Raffaello D’Andrea  
Institute for Dynamic Systems and Control, ETH Zurich, Switzerland  
  
  
  
Abstract  
  
  
    This paper presents a relaxed definition of hover for multicopters with propellers pointing in a common direction.  
These solutions are found by requiring that the multicopter remain substantially in one position, and that the solutions be  
constant when expressed in a coordinate system attached to the vehicle. The vehicle’s angular velocity is then shown to be  
either zero or parallel to gravity. The controllability of a vehicle’s attitude about these solutions is then investigated. These  
relaxed hover solutions may be applied as an algorithmic failsafe, allowing for example a quadrocopter to fly despite the  
complete loss of one, two, or three of its propellers. Experimental results validate the quadrocopter failsafe for two types  
of failure (a single propeller, and two opposing propellers failing), and a nonlinear simulation validates the remaining  
two types of failure (two adjacent, and three propellers failing). The relaxed hover solutions are also shown to allow a  
multicopter to maintain flight in spite of extreme centre of mass offsets. Finally, the design and experimental validation of  
three novel vehicles is presented.  
  
  
  
1. Introduction  
Multicopters have found broad use as research platforms, used e.g. for vision based pose estimation with quadro-  
copters (Fraundorfer et al. 2012) and hexacopters (Weiss et al. 2013), and also as platforms allowing for new capabilities.  
For example, the use of both quadrocopters and hexacopters for whale monitoring is investigated by Selby et al. (2011), and  
hexacopters are used by Rasmussen et al. (2013) for weed research; a team of quadrocopters is used to carry a slung load  
by Fink et al. (2011) and an octocopter is used to calibrate radio telescope antennae by Hörandel et al. (2013). Multicopters  
also hold promise as goods delivery vehicles (D’Andrea 2014).  
    These vehicles typically consist of an even number of propellers all pointing in a common direction, with half of  
the propellers having the opposite handedness from the remainder, and thus also rotating in the opposite direction. The  
propellers are then mechanically arranged such that the torques they produce can be made to sum to zero while the propeller  
thrusts support the vehicle’s weight. Differences between propeller thrusts allow the vehicle’s attitude to be changed, and  
the sum of the thrusts is used to accelerate the vehicle. This paper will only consider multicopters where all propellers have  
  
  
∗ Corresponding author, email:mwm@ethz.ch  
2  
  
  
parallel axes of rotation (note however, that alternative designs exist where this is not the case, e.g. Voyles & Jiang (2012)  
or Efraim et al. (2015)).  
    Amongst others, a motivation for using a multicopter with six or more propellers, instead of a four propeller quadrocopter,  
is that the vehicle is able to maintain normal flight if one of the propellers fails (see e.g. Scaramuzza et al. (2014) for a  
hexacopter design and Marks et al. (2012) for an octocopter rotor failure strategy). The need for having more than four  
propellers follows from the requirement that the vehicle be able to hover with zero angular velocity even after the failure  
of one of the actuators.  
    In this paper, this requirement of having zero angular velocity is relaxed, and instead hover solutions are searched  
for during which the position remains approximately constant, and where the solutions may be described with constant  
parameters: these relaxed hover solutions form a superset of those typically used for multicopters. This means that the  
vehicle may rotate at a constant velocity in hover. They allow for the design of novel hover-capable vehicles, and may also  
be employed to offer redundancy for multicopters experiencing an actuator failure. Since the solutions are constant in the  
body frame the powerful techniques of linear time invariant systems theory may be applied for analysis and control design.  
    An approach is presented to formulate and solve for such relaxed hover solutions, and for designing linear, time invariant  
controllers to control a hovering vehicle. This approach is used to compute solutions for a quadrocopter experiencing any  
combination of up to three complete propeller failures. For two of the cases, experimental validation is provided, while the  
approach for the remaining two types of failure are validated in a nonlinear simulation. The case of losing two opposing  
propellers is investigated in detail, with a specific focus on vehicle mass and aerodynamic properties that would render the  
vehicle uncontrollable.  
    The approach may also be used to control a multicopter where the centre of mass is located far from the propellers’  
geometric centre. A specific example is given for a quadrocopter with an eccentric centre of mass, where the conventional  
hover solution of zero angular velocity requires infeasible motor forces. However, by allowing the vehicle to rotate, a  
feasible solution is found, even if the centre of mass lies outside of the propellers’ convex hull.  
    The methods presented are also applied to design novel, rotating body vehicles. A family of such vehicles, called  
“spinners”, is presented, where each vehicle comprises a number of propellers arranged in a rotationally symmetric pattern  
about the vehicle’s centre of mass, and all propellers have the same handedness and rotate in the same direction. At hover  
these vehicles rotate at a high angular velocity, in the opposite direction of their propellers. A two, three, and four propeller  
spinner is presented. In addition to their dynamic properties, when equipped with a camera, such a vehicle could be used  
as a low-cost omnidirectional flying camera, similar to e.g. Haus et al. (2013) or Youngren et al. (2009) – note however  
that the spinners tend to rotate at higher velocity than the vehicles in these references, such that motion blur might present  
a problem.  
  
  
  
  
1.1. Related work  
A survey on fault detection and diagnosis and fault-tolerant control strategies for unmanned rotary wing vehicles is given  
by Zhang et al. (2013), and examples of commercial solutions are the emergency parachutes of Fruity chutes (2014) or  
those described by Dronologista (2014). Partial failure of a quadrocopter actuator is investigated for example in Ranjbaran  
& Khorasani (2010), Izadi et al. (2011) and Chamseddine et al. (2012). Complete propeller failure of a single propeller of  
a quadrocopter is investigated in Freddi et al. (2011), Akhtar et al. (2013), and Lanzon et al. (2014), where the strategy is to  
give up controlling the vehicle’s yaw angle, and use the remaining propellers to achieve a horizontal spin. A flight strategy  
to cope with an actuator failure on an octocopter is presented in Marks et al. (2012), and a hexacopter design with a focus  
on actuator redundancy is presented in Achtelik et al. (2012).  
                                                                                                                                3  
  
  
    The failsafe method presented in this paper extends those above by finding solutions when using fewer than three  
propellers. Furthermore, this paper presents a family of solutions when using only three propellers (such as a quadrocopter  
having lost one propeller) – the user may thus select the solution most appropriate to the vehicle and the situation.  
    The controllability of a flying, rotating vehicle with three mutually orthogonal propellers is analysed in Kataoka et al.  
(2011), and a circular flight path for this vehicle is designed for it in Kataoka et al. (2013) – the vehicle concept uses forward  
tilted propellers to induce rotation of the vehicle, so that the vehicle then also spins about an axis similarly to the spinners  
proposed in this work.  
    This paper will focus on vehicles that do not rely on aerodynamic effects (apart from drag and the propellers) for  
their stability – this is in contrast to maple-seed-like vehicles such as the Samara (Ulrich et al. 2010), or vehicles like the  
Spincopter (Orsag et al. 2013).  
    This work follows on a conference paper published previously (Mueller & D’Andrea 2014), and extends the results by  
  
•   presenting a more detailed derivation of the salient effects acting on a multicopter,  
•   deriving general conditions of the hover solutions without assumptions on the number of propellers, the locations of  
    the propellers, or the mass distribution of the vehicle,  
•   likewise deriving a general framework for establishing attitude controllability,  
•   considering additionally the failure case of a quadrocopter losing two adjacent propellers,  
•   showing how the relaxed hover solutions may be applied to multicopters with large centre of mass offsets, and  
•   presenting the design of three novel multicopters.  
  
  
1.2. Notation  
Boldface symbols like g are used throughout this paper to denote vectors in three-dimensional space, while non-boldface  
symbols like m will generally be used for scalars, with exceptions made explicit. The coordinate system in which a vector  
is expressed will be denoted by a superscript, for example g E expresses g in coordinate system E. Where possible, vectors  
will be left without expressing them in any coordinate system. A subscript will be used to express a relationship or attribute,  
for example ωBE represents the angular velocity of B with respect to E. The short-hand notation (p, q, r) will be used to  
compactly denote a column vector.  
  
  
1.3. Organisation  
This paper is organised as follows: in Section 2 the dynamic model of a generic multicopter is presented. Hover solutions  
are derived in Section 3 and the controllability of the vehicle about these solutions is investigated in Section 4. The approach  
is applied as an actuator failsafe for quadrocopters in Section 5, and applied to a quadrocopter with a large centre of mass  
offset in Section 6. A novel family of multicopters is presented in Section 7, and the paper concludes in Section 8.  
  
  
2. Multicopter modelling  
This section derives the equations of motion for a multicopter, with a special focus on the attitude dynamics. A model for  
the thrust force, reaction torque, and mechanical power consumption of a propeller is also presented. The equations are  
used in later sections to solve for hover solutions, and the power consumption of the propellers is used to compare different  
solutions.  
    Fig. 1 shows a multicopter with Np = 3 propellers. The multicopter has a total mass m (including propellers), and its  
position relative to some point fixed in an inertial frame is expressed as d. The mass moment of inertia of the multicopter  
excluding the propellers is IB , and the multicopter is rotating at an angular velocity ωBE with respect to the inertial frame.  
4  
  
  
  
  
Fig. 1. A multicopter with three propellers, showing the definition of the symbols used to derive its dynamics equations. The axes of  
rotation of the propellers are parallel, and are fixed with respect to the body.  
  
  
    The scalar thrust force fPi of each propeller i points in the body-fixed direction zB , and a body-fixed coordinate system B  
                    B  
is defined so that zB = (0, 0, 1). A perpendicular vector xB  
                                                           B = (1, 0, 0) is also defined, pointing towards propeller 1. Each  
thrust force i acts at a displacement rPi from the multicopter’s centre of mass. The angular velocity of propeller i with  
respect to the inertial frame is written as ωPi E .  
    In addition to thrust, each propeller produces a scalar reaction torque τPi opposing the propeller’s angular velocity and  
parallel to zB , as a result of aerodynamic drag acting on the propeller blade. The mass moment of inertia of propeller i  
is IPi . As a simplification, because the propellers rotate much faster than the vehicle’s body, the propellers will be treated  
as symmetric about their axes of rotation; their moment of inertia is then constant when expressed in a body-fixed frame,  
independent of the propeller’s instantaneous rotation angle.  
   It will be assumed that the vehicle travels at low translational velocities, such that translational drag forces may be  
neglected. For a discussion of such effects see e.g. Martin & Salaün (2010). As a result of the multicopter’s angular velocity,  
a drag torque τd acts on the multicopter. Gravity acts on the vehicle as a force mg.  
    The translational dynamics of the vehicle, expressed in the inertial coordinate system E, may now be composed  
as (Zipfel 2007)  
  
                                                                Np  
                                                                X  
                                                  md¨E = zB  
                                                          E  
                                                                      fPi + mg E .                                               (1)  
                                                                i=1  
  
  
    The orientation of the body with respect to the inertial reference frame is captured by the coordinate transformation  
matrix C EB , such that a vector transforms as v E = C EB v B . The differential equation for this matrix is (Zipfel 2007)  
  
                                                      Ċ EB = C EB JωBE  
                                                                     B  
                                                                        K                                                        (2)  
  
where JaK represents the skew-symmetric matrix form of the cross product, so that JaKb = a × b for any 3D vectors a  
                    B  
and b. Recall that ωBE represents the vehicle’s angular velocity with respect to the inertial frame ωBE , as expressed in the  
body-fixed frame B. The angular dynamics expressed in the body-fixed coordinate system are (Zipfel 2007)  
                                                                                
                      Np                                       Np                       Np  
                      X                                        X                        X  
           B B  
                             IPBi ω̇PBi E + JωBE  
                                              B      B B  
                                                                      IPBi ωPBi E  =         JrPBi KzB  
                                                                                                      B        B  
                                                                                                                 τPi + τdB .  
                                                                                                                      
          IB ω̇BE +                              K IB ωBE +                                            fPi + zB                 (3)  
                       i=1                                     i=1                      i=1  
  
  
The first two terms are the time derivative of respectively the vehicle’s and the propellers’ angular velocities with respect  
to the inertial frame, as expressed in the body-fixed frame. The third term expresses the cross-coupling of the angular  
                                                                                                                             5  
  
  
momentum in the system, due to taking the derivative in a non-inertial frame. The right-hand side of the equation represents  
all the moments acting upon the body, consisting of the propeller forces acting at a distance from the centre of mass, the  
propeller reaction torques, and the vehicle drag torque.  
  
  
  
2.1. Propeller model  
It is assumed that the pitch of each of the propellers is fixed and that a given propeller’s thrust and torque are functions  
only of the propeller’s angular velocity with respect to the air. Specifically, these functions are taken to be proportional to  
the square of the angular velocity (Pounds et al. 2002), and the air is assumed to be stationary with respect to the inertial  
frame. The thrust and torque are then characterised by the propeller coefficients κfi and κτi as follows:  
  
                                              fPi = κfi (ωPi E · zB ) |ωPi E · zB |                                        (4)  
                                              τPi = −κτi (ωPi E · zB ) |ωPi E · zB |                                       (5)  
  
with ωPi E = ωPi B + ωBE the rotational velocity of the propeller with respect to the air, and · denoting the vector inner  
product. The torque coefficient κτi is positive, and the sign of κfi is a function of the propeller’s handedness.  
       The propeller’s scalar speed ΩPi with respect to the body is typically controlled by an electronic speed controller, so  
that  
  
                                                        ωPi B = ΩPi zB .                                                   (6)  
  
       Note that this is a very simple propeller model, neglecting for example the translation of the propeller’s centre. An  
example of a more complicated model is given by Martin & Salaün (2010), where a relationship between a quadrocopter’s  
translational motion and a horizontal propeller force is given. Such effects are here assumed to be small disturbances, which  
will be compensated by feedback control.  
  
  
  
Power consumption The mechanical power consumed by a propeller may be computed as the product of the torque it  
produces and the angular velocity of the propeller with respect to the body, as given below.  
  
                                                        PPi = −τPi ΩPi .                                                   (7)  
  
The mechanical power is used in later sections to compare the efficiency of different hover solutions. This model may be  
extended, for example, with information about the electrical efficiency of the vehicle’s powertrain to compute the power  
required for the vehicle to hover.  
  
  
  
2.2. Rotational drag model  
The aerodynamic drag torque τd will in general be very hard to model, and will be a function of the geometry of the  
vehicle, the angular velocity of the vehicle, and the angular velocities of the propellers. The model used here is based on the  
form drag of a translating object, where the magnitude of the drag force is quadratic in the magnitude of the translational  
velocity (McCormick 1995).  
       Here it is assumed that the magnitude of the torque is quadratic in the vehicle’s angular velocity:  
  
                                                    τd = − kωBE k Kd ωBE                                                   (8)  
6  
  
  
where Kd is expressed as a constant 3 × 3 matrix in a body-fixed coordinate system, and the Euclidean norm of a vector  
is written as k·k.  
  
  
3. Hover solutions  
The conventional hover condition for a multicopter is at zero acceleration and zero angular velocity, with the thrust vector  
pointing opposite to gravity. For a symmetric quadrocopter in hover, for example, this implies that each motor produces a  
force equal to one quarter of the vehicle’s weight. The propellers’ angular momenta then sum to zero, so that the vehicle  
also has zero total angular momentum in hover. In this situation, the cross coupling in (3) disappears and the vehicle’s  
dynamics in the three translational directions decouple to first order at the equilibrium. An example of a control design for  
a quadrocopter can be found in Mahony et al. (2012).  
    The definition of hover is relaxed to include flight conditions where the vehicle remains substantially at one point in  
space, where the vehicle may have non-zero angular velocity. The translational acceleration may likewise be non-zero,  
but must average to zero for the vehicle to remain substantially in the same position. Only solutions which are constant  
when described in a body-fixed frame will be considered, with specifically the propellers’ angular velocity and the vehicle’s  
angular velocity being constant. These solutions are then time-invariant, allowing for the design of time invariant controllers.  
The rich literature and powerful tools available for the design and analysis of linear time invariant systems may then be  
applied. Such a hover solution can be fully described by Np + 6 variables: the Np propeller speeds, three components of  
the vehicle’s angular velocity, and three components of the vehicle’s orientation.  
  
  
3.1. Non-zero angular velocity during hover  
                                                                                         E  
If a multicopter’s angular velocity with respect to the inertial frame ωBE is zero then zB is constant, and by (1) the thrust  
direction zB must point opposite to gravity. The total thrust force must then equal the weight of the vehicle, so that the  
vehicle has zero acceleration in hover. Furthermore, by (3) the moments produced by the propellers must also sum to zero  
– this is the conventional hover solution.  
    For a constant non-zero angular velocity the orientation of the body with respect to an inertial frame at time t may be  
solved for in closed form from (2) as  
  
                                             C EB (t) = C EB (0) exp Jω̄BE  
                                                                        B  
                                                                                   
                                                                           tK                                               (9)  
  
where exp(·) is the matrix exponential. An overbar will be used to denote variables that are constant at the hover solution,  
          B  
such as ω̄BE .  
                                                               B  
    Using Rodrigues’ formula (Shuster 1993) and interpreting ω̄BE t as a rotation vector of constant direction and increasing  
magnitude, (9) can be solved for explicitly as  
  
                    B                  1             B  
                                                          B                1                     B              B  
                                                                                                                    K2  
                                                                                                          
             exp Jω̄BE tK = I +        B  
                                          sin      ω̄BE t Jω̄BE K+             2     1 − cos    ω̄BE t        Jω̄BE        (10)  
                                     ω̄BE                                   B  
                                                                          ω̄BE  
  
with I the identity matrix. This is periodic, with period Thvr = 2π/ kω̄BE k.  
    Since all other quantities are constant, the acceleration of the vehicle in hover must also be periodic, with period Thvr .  
The average acceleration of the vehicle can be calculated by averaging (1) over one period, where the time varying quantities  
have been made explicit below:  
  
                                      ZThvr                  ZThvr  
                                                                                          
                                                                         Np  
                                  1                      1               X  
                                          md¨E (t) dt =            E  
                                                                 zB (t)     f¯Pi + mg E  dt                              (11)  
                                 Thvr                   Thvr             i=1  
                                      0                      0  
                                                                                                                              7  
  
              E  
Transforming zB (t) into the body-fixed frame, setting the above equal to zero (so that the average acceleration is zero) and  
simplifying yields  
  
                                                             ZThvr              Np  
                                                   1                            X  
                                                E  
                                            −mg =                C EB (t) dt zB  
                                                                              B  
                                                                                   f¯Pi .                                  (12)  
                                                  Thvr                              i=1  
                                                              0  
  
  
    The integral on the right hand side may be thought of as the average coordinate transformation of the vehicle, and can  
be solved for by substituting (9)-(10):  
  
                                        ZThvr                                                           !  
                                  1                                                1  
                                            C EB (t) dt = C EB (0) I +                         B  
                                                                                          2 Jω̄BE K  
                                                                                                    2  
                                                                                                            .              (13)  
                                 Thvr                                              B  
                                                                                 ω̄BE  
                                        0  
  
  
   Substituting and simplifying yields  
                                                                     
                                                            Np  
                                                  zB · ω̄BE  
                                                            X  
                                        −mg E =          2    f¯Pi  C EB (0)ω̄BE  
                                                                                B  
                                                                                   .                                       (14)  
                                                  kω̄BE k i=1  
  
   A requirement on the total thrust produced can be derived from the above by applying the Euclidean norm:  
  
                                                    Np  
                                                    X          m kgk kω̄BE k  
                                                        f¯Pi =               .                                             (15)  
                                                    i=1  
                                                                |zB · ω̄BE |  
  
Note that this assumes that the sum of the propeller forces is positive.  
   Substituting (15) into (14) yields the condition that the angular velocity must be parallel to gravity in hover at time 0:  
  
                                            sgn (zB · ω̄BE ) EB    B      1 E  
                                                            C (0)ω̄BE =−     g                                             (16)  
                                                kω̄BE k                  kgk  
  
where sgn(·) is the signum function returning the sign of its argument. As it is constant, for any hover solution with constant  
angular velocity and constant motor forces, the angular velocity always remains parallel to gravity. This also implies that  
the vehicle’s vertical acceleration is always zero in hover.  
                                 B  
   Given some angular velocity ω̄BE , the total thrust force produced is constrained by (15), and (16) constrains the  
vehicle’s attitude so that the angular velocity is parallel to gravity. Additionally, the angular acceleration is required to be  
zero at hover, so that three additional constraints appear from (3):  
                                                                   
                                                Np                       Np  
                                                X                        X  
                           B      B B                                                  B ¯  
                                                      IPBi ω̄PBi E  =         JrPBi KzB        B  
                                                                                                  τ̄Pi + τdB  
                                                                                                        
                        Jω̄BE K IB ω̄BE +                                               fPi + zB                          (17)  
                                                i=1                      i=1  
  
  
with f¯Pi and τ̄Pi solved from Section 2.1, and τdB from Section 2.2.  
   There are then seven constraints for the Np + 6 unknowns, leaving Np − 1 degrees of freedom for the hover solution.  
These degrees of freedom may be determined, for example, by finding the hover solution that requires a minimum of input  
power, by symmetry considerations, or by considerations related to the controllability of the system.  
  
  
3.2. Position trajectory  
The vehicle’s acceleration in hover will either be constant, or be periodic with period Thvr . Specifically, if the vehicle’s  
thrust axis zB is not parallel to the hover angular velocity ω̄BE , a component of the total thrust must point perpendicular to  
8  
  
  
  
  
Fig. 2. An example cascaded control strategy, where an outer position controller defines a desired acceleration, and an inner controller  
controls the vehicle’s attitude so that the acceleration is achieved.  
  
  
gravity. Because the vehicle has zero vertical acceleration, the component of the thrust in the direction of the weight must  
equal the vehicle’s weight. The horizontal acceleration component āh may then be calculated as:  
                                                                           2  
                                                                    Np  
                                                     2         2  
                                                                   X  
                                              (māh ) + (m kgk) =     f¯Pi  .                                                    (18)  
                                                                                i=1  
  
  
    This can be rewritten as a centripetal acceleration of radius r̄hvr at vehicle’s angular velocity. Substituting the hover  
forces from (15), and simplifying yields:  
                                                                      s               2  
                                                          kgk               kω̄BE k  
                                              r̄hvr =             2                         − 1.                                   (19)  
                                                        kω̄BE k            zB · ω̄BE  
  
    The vehicle will thus move along a horizontal circle of this radius at the hover solution.  
  
  
4. Position and attitude control  
A given hover solution is only useful if the vehicle can enter the solution, and maintain it in the face of disturbances. In  
this section an approach is given to investigate the controllability of the vehicle about a hover solution. The problem is  
broken into two sections for ease of analysis: first it is argued that attitude control under constant total thrust is sufficient for  
position control. An approach is then given for determining the controllability of the attitude system, where the problem is  
reduced to investigating the controllability of a linear time invariant system with 5 states and NP − 1 inputs. An example  
cascaded controller is shown in Fig. 2.  
  
  
4.1. Position control  
If both the angular velocity and the total thrust force are constant over one rotation, the average acceleration ã(t) of the  
vehicle may be defined, similar to what was done in Section 3.1, by averaging d¨ over one rotation. The goal is then to  
control this average acceleration, and thereby the position of the vehicle.  
    Rather than directly finding an equation for ã(t), the exposition can be simplified by introducing two unit vectors. A  
constant body-fixed vector n is introduced, which lies parallel to the angular velocity of the vehicle in hover, and is oriented  
so that it has a positive component in the direction zB . This vector may be thought of as the vehicle’s thrust direction,  
averaged over one period of rotation.  
    The second unit vector ndes is defined through the vehicle’s desired acceleration. Introducing the short-hand fΣ (t)  
for the total thrust force, the vehicle’s acceleration averaged over one rotation ãdes (t) may be written as below, from  
which ndes (t) and fΣ (t) may be solved:  
                                                            z · n         
                                                              B  
                                              ãdes (t) =          fΣ (t) ndes (t) + g.                                            (20)  
                                                               m  
                                                                                                                                                   9  
  
  
  
  
Fig. 3. The relationships of the vectors introduced for the position control, where the desired acceleration vector ãdes is computed by  
the position controller, and from this follows the desired normal ndes through (20). The goal of the attitude controller is then to rotate  
the body-fixed vector n into ndes .  
  
  
  
Note that the factor (zB · n/m) is constant. These vectors are visualised in Fig. 3.  
    Therefore, if the body-fixed vector n can be controlled to point along some given ndes while the vehicle produces a  
total thrust force fΣ , the vehicle’s average acceleration will track the desired, and the vehicle’s position can be controlled.  
  
  
  
4.2. Reduced attitude control  
Only the two attitude degrees of freedom relevant to the vehicle’s translational motion are controlled, i.e. the unit vector n.  
For convenience a control coordinate frame C is introduced which is fixed with respect to the body-fixed frame B and  
where  
  
                                                         nC = C CB nB = (0, 0, 1) .                                                            (21)  
  
The remaining degree of freedom of C CB (that is, a rotation about n) may be chosen arbitrarily. The components αi of  
the angular velocity are introduced, so that  
  
                                                     C  
                                                    ωBE = C CB ωBE  
                                                                B  
                                                                   = (α1 , α2 , α3 ) .                                                         (22)  
  
    The time derivative of nC  
                            des follows from the product rule and (2):  
  
  
                                                     ṅC        C    C  
                                                       des = −JωBE Kndes + C  
                                                                             CE E  
                                                                               ṅdes .                                                         (23)  
  
The change in direction of the desired acceleration vector from (20) is captured by ṅE  
                                                                                      des .  
    The components ηi are introduced, such that nC                                                                  C  
                                                 des = (η1 , η2 , η3 ). The attitude control goal is then to drive ndes to (0, 0, 1),  
and drive ṅC  
            des to zero. This can be formalised by introducing the state deviation  
  
  
                                            ξ = (η1 , η2 , α1 , α2 , α3 ) − (0, 0, 0, 0, n · ω̄BE ) .                                          (24)  
  
    If ξ is stabilisable for some given fΣ , the vehicle is stabilisable about the desired acceleration. 1  
  
  
1 A stabilisable system is one which can be made asymptotically stable by appropriate state feedback (Callier & Desoer 1994), i.e. any uncontrollable  
  
 modes are asymptotically stable.  
10  
  
  
Selecting input variables Because the total thrust is fixed at fΣ , there are Np − 1 free input variables remaining, which  
may be used to control the attitude. These may be specified by introducing the input vector u, and then specifying the motor  
forces as below:  
  
                                            fPi = f¯Pi + ui , for i ∈ {1, . . . , Np − 1}                                 (25)  
                                                            Np −1  
                                                             X  
                                           fPNp = f¯PNp −           ui .                                                  (26)  
                                                             i=1  
  
  
     The propeller speeds (instead of thrusts) may alternatively be used as inputs, through the relationships of Section 2.1.  
The inputs may also be based on symmetry properties of a specific vehicle, so that e.g. independent torques are used as  
inputs, from which the required propeller speeds are recovered (as is commonly done when controlling quadrocopters,  
e.g. Mahony et al. (2012)).  
  
  
Linearised system The time derivative ξ˙ follows from (3) and (23). For the analysis it will be assumed that ṅE  
                                                                                                               des = 0, i.e.  
that the desired average vehicle acceleration in (20) is constant. In practise, a cascaded control strategy as shown in Fig. 2  
may be employed, with an outer position controller computing ndes . If the dynamics for this outer controller are sufficiently  
slow, the term ṅE  
                 des in (23) may be neglected.  
     The effect of the angular acceleration of the propellers with respect to the body, ω̇PBi B , may be included through a  
model of the motor dynamics. Alternatively, if the angular inertia of the propellers IPBi is sufficiently small, this term may  
be neglected.  
     Linearising about the hover solution yields a system as below:  
                                                                                          
                                              0        ± kω̄BE k           0   −1    0  
                                       ∓ kω̄BE k           0              1   0      0   
                                                                                          
  
                                  ξ˙ ≈   
                                                                                          
                                           0               0          a11     a12        ξ + Bu  
                                                                                     a13                                 (27)  
                                            0               0          a21     a22   a23   
                                                                                          
                                         
                                              0             0          a31     a32   a33  
  
where aij , B and u depend on the specific vehicle configuration, and follow in part from (3).  
   The requirement that (27) be stabilisable adds an additional criterion to the search for suitable hover solutions for a  
vehicle, in addition to the seven algebraic constraints derived in Section 3. Furthermore, (27) may serve as a basis for linear  
time invariant controller design: in Section 5 controllers are designed for a quadrocopter experiencing propeller failures,  
and in Section 7 controllers are designed for novel vehicles.  
  
  
5. Quadrocopter actuator failsafe  
Quadrocopters are popular as research testbeds, toys, and sensor platforms (e.g. for aerial photography). Compared to hexa-  
or octocopters, which have six or eight propellers, quadrocopters are mechanically simpler and have to carry less structural  
mass (which in turn may make them more efficient). However, they do not offer an obvious hardware redundancy if a  
propeller fails (that is, they cannot hover at zero angular velocity after losing a propeller).  
     In this section it is shown that quadrocopters can maintain a hover as defined in Section 3, after the complete loss  
of a propeller. In fact, under some restrictions on the vehicle’s mass distribution and aerodynamic properties, the vehicle  
remains controllable after the loss of any number of propellers, as long as a single propeller remains operable.  
     Five different quadrocopter failure scenarios are considered:  
  
•    no failure,  
                                                                                                                                11  
  
  
  
  
Fig. 4. The quadrocopter used for the experiments, maintaining controlled flight despite the complete loss of one propeller.  
  
  
  
•   failure of a single propeller,  
•   failure of two opposing propellers,  
•   failure of two adjacent propellers, and  
•   failure of three propellers.  
  
  
    The hover solutions will first be presented for each, followed by controller design, and then each scenario is validated  
in a non-linear simulation, For losing a single propeller, and losing two opposing propellers, the simulation is validated by  
experiment. Extension 1 shows a video of a quadrocopter in flight after the loss of a single, and two opposing propellers.  
    This section focuses on controlled flight near hover in a controlled environment, rather than discussing practical  
implementation issues related to using these hover solutions on an actual quadrocopter in the field. For fault detection  
strategies, the reader is referred to the references in the introduction. Low-cost and robust sensing/estimation strategies for  
rapidly rotating vehicles and robust transitions from one flight mode to another may be promising areas of future research.  
  
  
  
  
5.1. Platform  
The work is validated using quadrocopters based on the Ascending Technologies Hummingbird (Gurdan et al. 2007), in  
the Flying Machine Arena (Lupashin et al. 2014), which will be referred to here simply as “the quadrocopter”. Such a  
quadrocopter is shown in Fig. 4.  
   The vehicle’s mass was measured to be 0.50 kg, and the distance from the centre of mass to the centre of the propellers is  
0.17 m. The quadrocopter’s inertia was determined from a CAD model, and validated by measuring its period of oscillation  
when suspended around three different axes. The inertia matrix was estimated to be  
  
                                              B  
                                             IB = diag (2.7, 2.7, 5.2) × 10−3 kgm2 .                                           (28)  
  
    The propeller inertia about its axis of rotation was estimated by approximating the propeller and motor rotor as disks  
and cylinders, respectively. The remainder of the inertia matrix is neglected, so that the propeller’s inertia is constant when  
expressed in the body-fixed frame:  
  
                                               IPB = diag (0, 0, 1.5) × 10−5 kgm2 .                                            (29)  
  
    The propellers used were characterised using a force-torque sensor, and the thrust and reaction torque coefficients were  
estimated as |κfi | = 6.41 × 10−6 N s2 rad−2 and κτi = 1.1 × 10−7 N m s2 rad−2 , respectively. On a static test bench, the  
propellers are able to produce thrust forces in the range of 0.2 N to 3.8 N.  
12  
  
  
  
  
Fig. 5. Hover solutions for different failure scenarios, from left to right: failure of a single propeller, failure of two opposing propellers,  
and failure of two adjacent propellers. The graphs depict how the hover solutions vary as a function of the ratio of angular velocities of  
the remaining motors. For the single failure, solutions may also be found where the odd propeller turns faster than the two propellers  
with the same handedness – these are not shown as they consume much more power than the solutions shown. For both scenarios of  
two propellers failing only half of the possible solutions are shown with the remainder following by symmetry – e.g. for two opposing  
propellers failing, hover solutions exist with Ω̄3 /Ω̄1 > 1. The angular velocity is plotted as expressed in the body-fixed coordinate  
                  B  
system, where ω̄BE    = (p, q, r). Note that the propeller speeds shown are absolute.  
  
  
  
     The aerodynamic drag torque acting on the body was estimated to be diagonal in the body-fixed frame, with entries  
experimentally identified as below.  
  
                                            KdB = diag (0.7, 0.7, 1.4) × 10−4 Nms2 /rad2                                                  (30)  
  
  
  
5.2. Hover solutions  
Two short-hand notations will be used in this section: the total mechanical power consumed by the propellers will  
be expressed as PΣ = PP1 + PP2 + PP3 + PP4 . The propeller speeds ΩPi will be collected into the vector Ω =  
{−ΩP1 , ΩP2 , −ΩP3 , ΩP4 }, where the signs of the entries are changed so that all entries are positive if the propellers  
spin in their intended directions.  
     All configurations with more than one remaining functional propeller have at least one degree of freedom in selecting a  
hover solution. In such cases, numerical optimisation is used to find the hover solution that uses least mechanical power, as  
computed in Section 2.1. Depending on the application, minimising other variables may be more useful, e.g. minimising  
the vehicle’s total rotation rate, or maximising the minimum distance from the equilibrium propeller speeds to the actuator  
limits.  
                                                                                                                             13  
  
  
No failure For the quadrocopter with no propellers having failed there are 3 degrees of freedom for finding a hover solution.  
As expected, the minimum power solution is to have all propellers turning at angular velocities of equal magnitude, and  
specifically  
  
                                                Ω̄ = (438, 438, 438, 438) rad s−1                                          (31)  
                                              B  
                                            ω̄BE = (0, 0, 0) rad s−1                                                       (32)  
                                              P̄Σ = 36.9 W                                                                 (33)  
                                              r̄hvr = 0 mm.                                                                (34)  
  
  
Failure of a single propeller Without loss of generality it will be assumed that propeller 4 has failed, that is the propeller  
pointing to the right if looking at the quadrocopter from the top. In this case the constraint Ω̄4 = 0 is added and two degrees  
of freedom remain for the hover solution. Through numerical optimisation the following solution was found to consume  
least power in hover:  
  
                                                 Ω̄ = (585, 362, 585, 0) rad s−1                                           (35)  
                                                B  
                                              ω̄BE = (0.2, 4.3, 19.5) rad s−1                                              (36)  
                                                P̄Σ = 46.8 W                                                               (37)  
                                               r̄hvr = 6 mm.                                                               (38)  
  
    Some intuition for this may be gained by constraining Ω̄1 = Ω̄3 , and performing a line search over the ratio Ω̄2 /Ω̄1 .  
This is illustrated in Fig. 5.  
  
  
Failure of two opposing propellers Here it will be assumed that the left and right hand side propellers have failed, so  
that Ω̄4 = Ω̄2 = 0. Now there is only one degree of freedom for the hover solution, which may be explored with a line  
search over the ratio of the remaining propeller speeds: Ω̄3 /Ω̄1 . This line search is illustrated in Fig. 5. The least power  
solution is for zB to point opposite to gravity, with the two remaining propellers rotating at equal speeds:  
  
                                                  Ω̄ = (643, 0, 643, 0) rad s−1                                            (39)  
                                                 B  
                                               ω̄BE = (0, 0, 24.5) rad s−1                                                 (40)  
                                                 P̄Σ = 54.1 W                                                              (41)  
                                                 r̄hvr = 0 mm.                                                             (42)  
  
  
Failure of two adjacent propellers If both the right hand side and rear propellers have failed, the two added constraints  
are Ω̄4 = Ω̄3 = 0. Again, only one degree of freedom remains in the hover solution, which is also explored with a line  
search in Fig. 5. In this case, no hover solution exists for the two remaining propellers rotating at equal speed. Thus, it will  
be assumed that the magnitude of propeller 1’s velocity is larger than that of propeller 2. The least power solution is then:  
  
                                                  Ω̄ = (1067, 218, 0, 0) rad s−1                                           (43)  
                                                B  
                                              ω̄BE = (26.0, 0, 23.3) rad s−1                                               (44)  
                                                P̄Σ = 129 W                                                                (45)  
                                                r̄hvr = 9 mm.                                                              (46)  
14  
  
  
  
  
Fig. 6. The feedback structure used for the experiments. The position controller computes a desired average acceleration ãE     des , which is  
then converted into a desired total thrust fΣ and direction nE  
                                                             des according  to (20). The attitude controller’s outputs u i are combined   with  
the desired total thrust force to compute the desired individual propeller forces as in (25)-(26).  
  
  
Note that a symmetric solution exists for Ω̄P2 > Ω̄P1 , consuming the same power, but where the vehicle rotates in the  
opposite direction.  
  
Failure of three propellers If all but one propeller have failed, there are no degrees of freedom remaining in the hover  
solution. It will be assumed that only the front propeller remains, and the hover solution is then  
  
                                                       Ω̄ = (1103, 0, 0, 0) rad s−1                                                       (47)  
                                                     B  
                                                   ω̄BE = (28.0, −1.6, 24.5) rad s−1                                                      (48)  
                                                     P̄Σ = 141 W                                                                          (49)  
                                                     r̄hvr = 8 mm.                                                                        (50)  
  
     Note that it is unlikely that any practical quadrocopter has sufficient excess thrust that a single propeller is able to  
produce the force required.  
  
  
5.3. Control and validation  
The controllability of the vehicle about the hover solution is investigated for each of the five different failure cases presented  
above. If at least two propellers remain the method of Section 4 was used directly. For the case of losing three propellers  
(and thus only a single propeller remaining), the method was modified to use the single remaining force as input. In this  
case, the total force cannot be controlled independently of the attitude. The layout of the controller is shown in Fig. 6,  
showing the arrangement of the cascaded position and attitude controllers. The controllers are described in more detail  
below.  
  
Position control A position controller was created which calculates a desired acceleration based on the vehicle’s current  
position and velocity, such that the vehicle’s position behaves as a damped second order system (Dorf & Bishop 2008).  
This desired acceleration can then be substituted for the desired average acceleration in (20), and be transformed into a  
desired vehicle direction ndes and total thrust fΣ . The second order system’s natural frequency and damping ratio were set  
to 1.5 rad s−1 and 0.7, respectively.  
  
Attitude control Given a linearised attitude system of the form (27), a linear quadratic regulator (LQR) controller (Anderson  
& Moore 2007) was designed for the attitude system. The same LQR weights are used for each failure scenario. The cost  
                                                                                                                                              15  
  
  
  
  
Fig. 7. Simulation results for a quadrocopter recovering from a 1 m horizontal position error for different failure scenarios. Each column  
represents a different failure condition, from left to right: no failure (nominal operation), failure of a single propeller, failure of two  
opposing propellers, failure of two adjacent propellers, failure of three propellers. Note that the angular velocity is expressed in the  
control frame C, so that two components must be zero at equilibrium. The position is expressed as dE = (d1 , d2 , d3 ), with d3 pointing  
opposite to gravity. The simulation is validated by experiment in Section 5.4. Each row shares its legend. The steady-state vertical offset  
for the failure of two adjacent propellers and the failure of three propellers is due to a steady state attitude error, as can also be seen in the  
corresponding steady-state errors of the angular velocity. This error could be explained by interaction between the attitude controller and  
the position control, and higher-order dynamic effects not compensated for by the linear controller strategy. The steady-state position  
offsets may be readily compensated for, e.g. by adding an integral term to the position control.  
  
  
  
on deviations from the desired normal was set to 20, and the cost on the angular rates set to zero. The input cost was set to  
the identity matrix of the appropriate size, with units N−2 .  
    The five dimensional attitude subsystem (27) is stabilisable for each scenario, and is controllable in each case except  
for two opposing propellers failing. In this case, for the given hover solution, the vehicle’s angular velocity about zB is  
uncontrollable. Note that this may be avoided by selecting a different hover condition – this would however increase the  
hover power consumption and does not actually affect the vehicle’s hover performance.  
  
  
  
  
Simulation results The control strategy is validated in a nonlinear simulation for each of the different failure scenarios.  
No sensor noise or external disturbances are simulated. The actuators are simulated to have additional dynamics, so that  
the propeller speeds respond as a first order system with time constant 15 ms.  
    A comparison of the flight performance for each failure scenario in simulation is shown in Fig. 7. The data starts with  
the vehicle at the hover attitude, but with a 1 m horizontal position offset. In each scenario, the vehicle is able to stabilise  
its position at the origin.  
16  
  
  
  
  
Fig. 8. Comparison of experimental (left column) and simulated (right column) data for a quadrocopter with a single failed propeller,  
and an initial 1 m horizontal position error. The angular velocity is as expressed in the control frame C, and the position is expressed  
as dE = (d1 , d2 , d3 ). The steady-state vertical position offset of approximately 0.8 m in the experiment is discussed in the text.  
  
  
  
  
5.4. Experimental validation  
The simulation is validated by comparing it to data collected from flight experiments in the Flying Machine Arena. The  
vehicle’s state is estimated using position and orientation measurements from a motion capture system, and inertial sensors  
on the vehicle. A video of such experiments can be found in Extension 1. These experiments are only possible for the  
scenarios of no failure, failure of a single propeller, and failure of two opposing propellers – for the remaining cases (two  
adjacent propellers failing or three propellers failing) the quadrocopter of Section 5.1 can not produce sufficient thrust to  
compensate for the vehicle’s weight. The data for a single propeller failure are compared in Fig. 8, and for the failure of  
two opposing propellers in Fig. 9.  
     For a single failure the experimental data differs somewhat from the simulated data, most notably for the steady-state  
angular velocity. In experiment, the vehicle has an angular velocity component perpendicular to the n axis of approxi-  
mately 4.1 rad s−1 . The angular velocity component about n is also lower in experiment than expected from the simulation  
(15.8 rad s−1 compared to 19.9 rad s−1 ). This discrepancy may be due to additional torques acting on the vehicle, specif-  
ically torques generated by the translation of the centres of the propellers. Further characterisation of the aerodynamic  
effects acting on the vehicle at high rotational velocities represent a possibly interesting area of future research.  
     Furthermore, the vehicle has an approximately 0.8 m vertical offset in experiment for a single failure. This may  
be partially explained by the angular velocity offset of the experiments w.r.t. the simulation – in the experiment, the  
quadrocopter’s thrust axis is tilted farther away from gravity, and thus the feed-forward thrust to compensate for the  
vehicle’s weight from (20) will be too low. This could be readily compensated by e.g. adding an integral control term to  
the position controller, or by refining the model to allow for better prediction of the vehicle’s angular velocity equilibrium.  
Nonetheless, the controller successfully stabilises the vehicle about the hover solution, and the vehicle’s horizontal error is  
controlled to zero as expected.  
     For the case of two failed opposing propellers, the simulation matches the experimental data well. It is notable that  
the two motors do not produce equal force in experiment after the transients have died away – this may be explained by a  
centre of mass offset of the experimental vehicle, or an imperfection in one of the two actuators.  
                                                                                                                                       17  
  
  
  
  
Fig. 9. Comparison of experimental (left column) and simulated (right column) data for a quadrocopter with two opposing failed  
propellers, and an initial 1 m horizontal position error. The angular velocity is as expressed in the control frame C, and the position is  
expressed as dE = (d1 , d2 , d3 ).  
  
  
  
  
5.5. Special cases for controllability  
Some intuition for when it is possible to control a quadrocopter having lost one or more propellers may be gained by  
analysing the hover conditions and controllability requirements of Sections 3 and 4 under some simplifying assumptions,  
especially for the case of losing two opposing propellers.  
    Specifically, it will be assumed that the vehicle’s mass moment of inertia expressed in the body frame is diagonal, with  
only two unique entries. The inertia of the propeller is assumed to be zero except about its axis of rotation:  
  
                                                   B  
                                                  IB = diag (IB,xx , IB,xx , IB,zz )                                                 (51)  
                                                  IPB = diag (0, 0, IP,zz )                                                          (52)  
  
where furthermore IP,zz  IB,zz . The propellers are assumed identical except for their handedness, so that  
  
                                                            κfi = (−1)i κf                                                           (53)  
                                                            κτi = κτ .                                                               (54)  
  
It is assumed that the vehicle’s angular velocity with respect to the inertial frame is much smaller than the propellers’ angular  
velocity with respect to the body, i.e kωBE k  kωPi B k for each remaining propeller i. The propellers are mounted at a  
distance l from the vehicle’s centre of mass.  
    The vehicle’s drag torque coefficient is given below, again with only two unique entries:  
  
                                                KdB = diag (Kd,xx , Kd,xx , Kd,zz ) .                                                (55)  
18  
  
                                                            B  
Defining the components of the vehicle angular velocity as ωBE = (p, q, r), and applying the simplifications, it follows  
that:  
  
                                    ṗIB,xx = (IB,xx − IB,zz ) qr − Kd,xx p kωBE k +  
                                                                                                                              (56)  
                                           lκf Ω22 − Ω24 − qIP,zz (Ω1 + Ω2 + Ω3 + Ω4 )  
                                                          
  
                                    q̇IB,xx = (−IB,xx + IB,zz ) pr − Kd,xx q kωBE k +  
                                                                                                                              (57)  
                                           lκf Ω21 − Ω23 + pIP,zz (Ω1 + Ω2 + Ω3 + Ω4 )  
                                                          
  
                                    ṙIB,zz = −Kd,zz r kωBE k −  
                                                                                                                              (58)  
                                            κτ (|Ω1 | Ω1 + |Ω2 | Ω2 + |Ω3 | Ω3 + |Ω4 | Ω4 ) .  
  
     Considering specifically the loss of two opposing motors, such that Ω2 = Ω4 = 0, and constraining the hover solution  
so that Ω̄1 = Ω̄3 , the hover solution follows as  
  
                                                      p̄ = q̄ = 0                                                             (59)  
                                                           s  
                                                              κτ m kgk  
                                                      r̄ =                                                                    (60)  
                                                               κf Kd,zz  
                                                                   s  
                                                                      m kgk  
                                                    Ω̄1 = Ω̄3 = −           .                                                 (61)  
                                                                        2κf  
  
     The attitude controllability can now be analysed as described in Section 4, by introducing the scalar attitude input u  
and requiring that the total force remain constant; it follows that u = (fP1 − fP3 ) /2. The control frame C is selected to  
coincide with the body frame B. The entries aij of the linearised dynamics matrix (27) and the input matrix B are then as  
follows:  
  
                                                    Kd,xx r̄  
                                     a11 = a22 = −                                                                            (62)  
                                                     IB,xx  
                                             2Kd,zz r̄  
                                     a33 = −                                                                                  (63)  
                                              IB,zz  
                                                    p  
                                                       2m kgkIP,zz   (IB,xx − IB,zz ) r̄  
                                     a12 = −a21 = √                +                                                          (64)  
                                                        κf IB,xx           IB,xx  
                                     a13 = a23 = a31 = a32 = 0                                                                (65)  
                                             2l h              iT  
                                      B=          0 0 0 1 0 .                                                                 (66)  
                                           IB,xx  
  
     It is clear that r is an exponentially stable, uncontrollable mode of the linearised attitude system, and it will be neglected  
for the remainder of the analysis. A reduced, four state system remains with a single input. The controllability of this system  
may be determined by computing the determinant of the 4 × 4 controllability matrix of the system and ensuring that this  
is non-zero (Callier & Desoer 1994).  
     After some tedious algebra it emerges that there are two plausible conditions where this system is uncontrollable. Firstly,  
if (67) holds, the cross-coupling in the attitude dynamics (3) disappears and the vehicle’s roll rate p is uncontrollable:  
                                                    p                            √  
                                            IP,zz       2Kd,zz = (IB,zz − IB,xx ) κτ .                                        (67)  
                                                                                                                           19  
  
  
   If, instead, the following two conditions hold, the system is also uncontrollable:  
  
                                                  Kd,xx = 0 and                                                          (68)  
                                                p                          √  
                                           IP,zz 2Kd,zz = (IB,zz − 2IB,xx ) κτ .                                         (69)  
  
In this case there are two uncontrollable modes, which correspond to p + a12 η1 and η2 .  
   These two conditions may be used to determine whether a quadrocopter’s attitude will be close to uncontrollable with  
two opposing propellers, given its physical parameters. Unfortunately, the aerodynamic drag parameters Kd,xx and Kd,zz  
may be hard to accurately predict.  
   If only a single propeller has failed, the same hover solution may be selected as when two opposing propellers have  
failed with p̄ = q̄ = 0. Then the system has an additional input which directly actuates the roll rate p, and it will be  
controllable for all mass distributions.  
  
Further simplifications The preceding results may be given a more intuitive geometric interpretation by further assuming  
that the propellers have zero inertia, i.e. IP,zz = 0. The first case for being uncontrollable then simplifies from (67) to  
the requirement that IB,xx = IB,zz (recall that by assumption also IB,xx = IB,yy ). This happens if the vehicle’s mass  
distribution is symmetric, i.e. similar to that of a sphere. The second case for being uncontrollable simplifies from (69)  
to IB,zz = 2IB,xx = 2IB,yy – this corresponds to the mass distribution of a two-dimensional object, e.g. a flat plate with  
the propellers’ thrust axes pointing along the plate’s normal.  
  
  
6. Quadrocopter centre of mass offsets  
If a quadrocopter’s centre of mass is not located at its geometric centre the propeller forces will no longer all be equal  
when the vehicle hovers at zero angular velocity. Specifically, for the input torques of (3) to balance, propellers closer to  
the centre of mass must produce larger forces.  
   For large centre of mass offsets the required propeller forces may be close to (or even exceed) the propeller limits,  
leaving little or no control authority for feedback control. In such a situation, the approach of Sections 3-4 may be used to  
find a hover solution where the nominal motor forces are further away from the actuator limits.  
   As an extreme example: for a quadrocopter with propellers distributed on the corners of a square, if the centre of mass is  
located halfway between the geometric centre of the propellers and one propeller, the propeller farthest from the centre of  
mass must produce zero force for the input torques to balance. If the centre of mass moves further away from the geometric  
centre, the far propeller must produce a negative thrust. In this situation, the vehicle will consume significantly more power  
than if the centre of mass coincides with the geometric centre. Furthermore, specific hardware may not be able to produce  
negative thrust.  
   If however the vehicle is allowed to rotate during hover, one may search e.g. for a hover solution that minimizes the  
required mechanical power, or maximizes the distance from the propellers’ thrust in hover to the actuator saturation limits.  
   As an example, consider the quadrocopter of Section 5 carrying an additional load of 0.1 kg at some distance ρ from the  
geometric centre of the propellers. As the mass is shifted farther from the vehicle’s geometric centre, the vehicle’s centre  
of mass also moves farther from the geometric centre. The ratio of vehicle mass to the additional mass implies that the  
centre of mass is located at a distance ρ/6 from the geometric centre.  
   The mass moment of inertia of the vehicle with the added mass may be computed using Huygen’s theorem (Zipfel  
2007). For the simplicity of analysis, it is assumed that the vehicle’s drag matrix Kd remains unchanged with the added  
mass and the resulting shifted centre of mass.  
   Fig. 10 compares the motor speeds and power requirements of the quadrocopter as the mass is moved farther away from  
the geometric centre, for both the conventional hover solution (with zero angular velocity) and the relaxed hover solution of  
20  
  
  
  
  
Fig. 10. Hover solutions for a quadrocopter with an additional mass mounted at varying distances from the vehicle’s geometric centre,  
showing the solution at which the vehicle has zero angular velocity, and a relaxed hover solution of Section 3 that minimizes power  
consumption. Note that the abscissa is the distance of the centre of mass to the geometric centre, normalised by the quadrocopter’s arm  
length. Because of the mass ratio, the additional mass is placed six times farther from the propellers’ geometric centre than the resulting  
centre of mass. The top four graphs plot for each propeller the ratio of each required propeller speed to that speed required if the centre of  
mass coincides with the geometric centre. The bottom graph shows the three components of the vehicle’s angular velocity in the relaxed  
                                                             B  
hover solution expressed in the body-fixed frame, where ωBE       = (p, q, r).  
                                                                                                                                           21  
  
  
  
  
Fig. 11. A novel class of flying vehicles, from left to right: bispinner, trispinner, and quadspinner. For each vehicle, all propellers rotate  
in the same direction, causing the vehicle to rotate in the opposite direction in hover. Extension 2 contains a video showing each of these  
vehicles flying.  
  
  
  
Section 3. The mass is moved from the geometric center along the axis pointing towards propeller 1. For the conventional  
hover solution, the power required increases significantly as the mass is moved outwards, and at distances of ρ > 0.51 m  
the propeller farthest from the added mass must rotate in the direction opposite to the usual.  
    The figure also shows the results if the vehicle is allowed to rotate, and a search is done to find motor speeds that  
minimise the mechanical power in hover. The vehicle’s mechanical power remains within 5% of the nominal power, even  
with displacements up to ρ = 1.2 m. The motor speeds also remain close to the nominal speeds, even at large displacements.  
Note that for ρ > 1.02 m the centre of mass lies outside of the convex hull of the four propellers.  
    Thus, if the propeller’s direction of rotation is fixed, a quadrocopter may be operated under a much wider range of  
centre of mass positions with the relaxed hover conditions compared to the restriction that the vehicle’s angular velocity is  
zero at hover.  
  
  
  
  
7. Novel vehicles  
A novel class of hover-capable vehicles may also be designed using the approach of Section 3. Here, three examples are  
given, with novel vehicles having two, three, or four propellers arranged symmetrically about the vehicle’s centre of mass.  
The propellers are mounted such that their thrust axes are parallel, and all propellers have the same handedness.  
    This family of vehicles is dubbed “spinners”, allowing a prefix for the number of propellers. A quadspinner, trispinner,  
and bispinner are shown in Fig. 11, and Extension 2 contains a video showing each vehicle in flight. Because of their  
symmetries, the hover equilibria are chosen such that all propellers have the same rotational speed.  
    Each vehicle is controlled similarly to the quadrocopter in Fig. 6, where the position and attitude control are separated.  
A desired attitude is generated by the position controller, which makes the position behave like a second order damped  
system. An LQR controller is created for the attitude, based on the first order model of Section 4.  
              n of Section 5.2 will obe used to describe the hover solutions, and again so that all entries in Ω̄ are nominally  
    The notation  
positive: Ω̄ = −ΩP1 , . . . , −ΩPNp . The motors and propellers used are the same as those of Section 5.1.  
  
  
  
  
7.1. Quadspinner  
A normal quadrocopter may easily be converted into a quadspinner, by reversing the direction of rotation of two motors  
(and accordingly replacing the propellers). The quadspinner shown in Fig. 11 has the same physical properties as the  
quadrocopter presented in Section 5.1, except for the propellers’ handedness.  
22  
  
  
     It will then hover at the following condition:  
  
                                                 Ω̄ = (462, 462, 462, 462) rad s−1                                             (70)  
                                                B  
                                              ω̄BE = (0, 0, 24.2) rad s−1                                                      (71)  
                                                P̄Σ = 38.9 W.                                                                  (72)  
  
  
  
  
7.2. Trispinner  
A three propeller vehicle was also created, with the propellers mounted at intervals of 120◦ . Conceptually, this trispinner  
is very similar to the quadspinner, as the propellers can produce a torque in any direction in the plane perpendicular to the  
thrust vector, for some given total thrust value.  
     Visually similar tri-rotor vehicles exist (see for example Salazar-Cruz et al. (2006), Zou et al. (2012)) that use a servo  
motor in addition to the three propellers, where the servo motor can rotate one of the vehicle’s arms and thus vector the  
thrust of one propeller. These vehicles can then fully control their attitudes in addition to their total thrust. The trispinner, in  
comparison, does not attempt to control its orientation about the thrust axis, and has the advantage of being mechanically  
simpler.  
     The vehicle depicted in Fig. 11 has a mass of 0.44 kg, with the propellers mounted 0.17 m from the vehicle’s centre of  
mass. The vehicle’s inertia matrix and drag characteristics are then given below:  
  
                                         B  
                                        IB = diag (2.4, 2.3, 4.4) × 10−3 kg m2                                                 (73)  
                                       KdB = diag (0.55, 0.55, 1.1) × 10−4 Nms2 /rad2                                          (74)  
  
and the hover condition is then  
  
                                                      Ω̄ = (498, 498, 498) rad s−1                                             (75)  
                                                  B  
                                                ω̄BE = (0, 0, 26.1) rad s−1                                                    (76)  
                                                    P̄Σ = 36.6 W.                                                              (77)  
  
  
  
  
7.3. Bispinner  
Finally, a bispinner was created, by removing two complete arms from a quadrocopter. The vehicle has a mass of 0.38 kg,  
and the inertia matrix and drag characteristics are as below:  
                                                                          
                                                270  65              0  
                                           B  
                                          IB =  65 3300             0  × 10−6 kg m2                                          (78)  
                                                                          
  
                                                      0      0      3400  
                                         KdB = diag (0, 3.6, 7.2) × 10−5 Nms2 /rad2 .                                          (79)  
  
Note that the vehicle’s inertia about the axis connecting the motors is an order of magnitude smaller than the other terms  
on the diagonal. The off-diagonal terms follow as a result of the battery being positioned at an angle.  
                                                                                                                                     23  
  
  
  
  
Fig. 12. Experimental data for a bispinner at hover, where the motors (and thus attitude control) are switched off at time 0. The open  
loop exponential instability of the angular velocity is clearly visible, and angular accelerations up to 450 rad s−2 are measured by the  
vehicle’s rate gyroscopes.  
  
  
  
    The hover condition for this vehicle is  
  
                                                      Ω̄ = (567, 567) rad s−1                                                      (80)  
                                                    B  
                                                  ω̄BE = (0, −0.1, 29.7) rad s−1                                                   (81)  
                                                     P̄Σ = 36.1 W.                                                                 (82)  
  
    Because of its mass distribution, this vehicle has some unique dynamic properties compared to the tri- and quadspinner.  
The matrices of the linearised attitude system, computed with the method of Section 4, are as below:  
                                                                                                  
                                                     0       29.7     0      −1              0  
                                             −29.7           0       1       0           0   
                                                                                               
                                                                                               
                                           A=  
                                              0              0      1.6     54.1        −0.2                                    (83)  
                                              0              0      22.8    −2.0         0   
                                                                                               
  
                                                     0        0       0       0          −1.3  
                                             h                                      iT  
                                           B= 0       0   24.9      −103.5    0.3        .                                         (84)  
  
    This linearised system has a single exponentially unstable mode, with eigenvalue at 35.2. This open loop instability  
can be clearly seen in Fig. 12.  
  
  
  
8. Conclusion and outlook  
This paper presents a broadened definition of hover for multicopters, where specifically the vehicle is not constrained to  
have zero angular velocity when hovering. The class of vehicles that can achieve these hover conditions are a superset of  
conventional multicopters.  
    By allowing a conventional multicopter to hover at non-zero angular velocity, it is shown that the vehicle can main-  
tain controlled flight despite the complete loss of all but one propeller. This seems particularly relevant in the case of  
quadrocopters, as this negates one of the main arguments for using hexacopters or octocopters instead of quadrocopters.  
As such, the use of the proposed failsafe solutions may allow for a more widespread use (and greater public acceptance of)  
24  
  
  
quadrocopters. The extended hover solutions are also shown to allow a quadrocopter to maintain flight under large centre  
of mass disturbances.  
     Novel vehicles may also be conceived, such as the spinners presented herein. These vehicles could be used as novel  
hobbyist platforms or toys, or could be used as novel sensing platforms. For example, by mounting a line scanner to  
such a vehicle, and using the vehicle’s angular velocity to sweep the sensor over the environment, a mechanically simple  
omnidirectional sensor can be created from a line scanner.  
  
  
Acknowledgements  
The Flying Machine Arena is the result of contributions of many people, a full list of which can be found at http://www.  
idsc.ethz.ch/Research\_DAndrea/FMA/participants. The authors would like to thank Alex Wilkinson  
for his help designing the trispinner.  
  
  
Funding  
This research was supported by the Swiss National Science Foundation (SNSF), under grant application 138112.  
  
  
References  
  
Achtelik, M. C., Doth, K.-M., Gurdan, D. & Stumpf, J. (2012), Design of a multi rotor MAV with regard to efficiency, dynamics and  
  redundancy, in ‘AIAA Guidance, Navigation, and Control Conference’, pp. 1–17.  
Akhtar, A., Waslander, S. & Nielsen, C. (2013), Fault tolerant path following for a quadrotor, in ‘IEEE Annual Conference on Decision  
  and Control (CDC)’, pp. 847–852.  
Anderson, B. D. & Moore, J. B. (2007), Optimal control: linear quadratic methods, Courier Dover Publications.  
Callier, F. & Desoer, C. (1994), Linear System Theory, Springer Texts in Electrical Engineering, Springer.  
Chamseddine, A., Zhang, Y., Rabbath, C. A., Join, C. & Theilliol, D. (2012), ‘Flatness-based trajectory planning/replanning for a quadrotor  
  unmanned aerial vehicle’, IEEE Transactions on Aerospace and Electronic Systems 48(4), 2832–2848.  
D’Andrea, R. (2014), ‘Can drones deliver?’, IEEE Transactions on Automation Science and Engineering 11(3), 647–648.  
Dorf, R. & Bishop, R. (2008), Modern Control Systems, Eleventh Edition, Prentice-Hall.  
Dronologista (2014), ‘DJI dropsafe system’, (Accessed 29 Nov. 2014) http://robohub.org/dji-dropsafe-system/ .  
Efraim, H., Shapiro, A. & Weiss, G. (2015), ‘Quadrotor with a dihedral angle: on the effects of tilting the rotors inwards’, Journal of  
  Intelligent & Robotic Systems pp. 1–12.  
Fink, J., Michael, N., Kim, S. & Kumar, V. (2011), ‘Planning and control for cooperative manipulation and transportation with aerial  
  robots’, International Journal of Robotics Research 30(3), 324–334.  
Fraundorfer, F., Heng, L., Honegger, D., Lee, G. H., Meier, L., Tanskanen, P. & Pollefeys, M. (2012), Vision-based autonomous mapping  
  and exploration using a quadrotor MAV, in ‘IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)’, IEEE,  
  pp. 4557–4564.  
Freddi, A., Lanzon, A. & Longhi, S. (2011), A feedback linearization approach to fault tolerance in quadrotor vehicles, in ‘Proceedings  
  of the 18th IFAC World Congress’, pp. 5413–5418.  
Fruity chutes (2014), ‘Multicopter, quadcopter, drone, RC aircraft recovery and rescue chutes’, (Accessed 29 Nov. 2014) http:  
  //www.fruitychutes.com/uav_rpv_drone_recovery_parachutes/multicopter_quadcopter-rc_  
  aircraft_recovery_and_rescue_chutes.htm .  
Gurdan, D., Stumpf, J., Achtelik, M., Doth, K.-M., Hirzinger, G. & Rus, D. (2007), Energy-efficient autonomous four-rotor flying robot  
  controlled at 1 kHz, in ‘IEEE International Conference on Robotics and Automation (ICRA)’, pp. 361–366.  
Haus, T., Orsag, M. & Bogdan, S. (2013), Omnidirectional vision based surveillance with spincopter, in ‘IEEE International Conference  
  on Unmanned Aircraft Systems (ICUAS)’, pp. 326–332.  
                                                                                                                                         25  
  
  
Hörandel, J. R., Buitink, S., Corstanje, A., Enriquez, J. E. & Falcke, H. (2013), The LOFAR radio telescope as a cosmic ray detector, in  
  ‘International Cosmic Ray Conference’.  
Izadi, H. A., Zhang, Y. & Gordon, B. W. (2011), Fault tolerant model predictive control of quad-rotor helicopters with actuator fault  
  estimation, in ‘Proceedings of the 18th IFAC World Congress’, pp. 6343–6348.  
Kataoka, Y., Sekiguchi, K. & Sampei, M. (2011), Nonlinear control and model analysis of trirotor uav model, in ‘Proceedings of the 18th  
  IFAC World Congress’, Vol. 18, IFAC, pp. 10391–10396.  
Kataoka, Y., Sekiguchi, K. & Sampei, M. (2013), Circle motion control of trirotor uav via discrete output zeroing, in ‘IEEE Annual  
  Conference on Decision and Control (CDC)’, IEEE, pp. 226–231.  
Lanzon, A., Freddi, A. & Longhi, S. (2014), ‘Flight control of a quadrotor vehicle subsequent to a rotor failure’, Journal of Guidance,  
  Control, and Dynamics 37(2), 580–591.  
Lupashin, S., Hehn, M., Mueller, M. W., Schoellig, A. P., Sherback, M. & D’Andrea, R. (2014), ‘A platform for aerial robotics research  
  and demonstration: The Flying Machine Arena’, Mechatronics 24(1), 41 – 54.  
Mahony, R., Kumar, V. & Corke, P. (2012), ‘Aerial vehicles: Modeling, estimation, and control of quadrotor’, IEEE Robotics & Automation  
  Magazine 19(3), 20–32.  
Marks, A., Whidborne, J. F. & Yamamoto, I. (2012), Control allocation for fault tolerant control of a VTOL octorotor, in ‘UKACC  
  International Conference on Control’, IEEE, pp. 357–362.  
Martin, P. & Salaün, E. (2010), The true role of accelerometer feedback in quadrotor control, in ‘IEEE International Conference on  
  Robotics and Automation (ICRA)’, pp. 1623–1629.  
McCormick, B. W. (1995), Aerodynamics Aeronautics and Flight Mechanics, John Wiley & Sons, Inc.  
Mueller, M. W. & D’Andrea, R. (2014), Stability and control of a quadrocopter despite the complete loss of one, two, or three propellers,  
  in ‘IEEE International Conference on Robotics and Automation (ICRA)’.  
Orsag, M., Cesic, J., Haus, T. & Bogdan, S. (2013), ‘Spincopter wing design and flight control’, Journal of Intelligent & Robotic Systems  
  70(1-4), 165–179.  
Pounds, P., Mahony, R., Hynes, P. & Roberts, J. (2002), Design of a four-rotor aerial robot, in ‘Australasian Conference on Robotics and  
  Automation’, Vol. 27, p. 29.  
Ranjbaran, M. & Khorasani, K. (2010), Fault recovery of an under-actuated quadrotor aerial vehicle, in ‘IEEE Annual Conference on  
  Decision and Control (CDC)’, IEEE, pp. 4385–4392.  
Rasmussen, J., Nielsen, J., Garcia-Ruiz, F., Christensen, S. & Streibig, J. C. (2013), ‘Potential uses of small unmanned aircraft systems  
  (UAS) in weed research’, Weed Research 53(4), 242–248.  
Salazar-Cruz, S., Kendoul, F., Lozano, R. & Fantoni, I. (2006), Real-time control of a small-scale helicopter having three rotors, in  
  ‘IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)’, IEEE, pp. 2924–2929.  
Scaramuzza, D., Achtelik, M., Doitsidis, L., Friedrich, F., Kosmatopoulos, E., Martinelli, A., Achtelik, M., Chli, M., Chatzichristofis, S.,  
  Kneip, L., Gurdan, D., Heng, L., Lee, G. H., Lynen, S., Pollefeys, M., Renzaglia, A., Siegwart, R., Stumpf, J., Tanskanen, P., Troiani,  
  C., Weiss, S. & Meier, L. (2014), ‘Vision-controlled micro flying robots: from system design to autonomous navigation and mapping  
  in gps-denied environments’, IEEE Robotics Automation Magazine 21(3), 26–40.  
Selby, W., Corke, P. & Rus, D. (2011), Autonomous aerial navigation and tracking of marine animals, in ‘Proceedings of the 2011  
  Australasian Conference on Robotics and Automation’, Australian Robotics & Automation Association, pp. 1–7.  
Shuster, M. D. (1993), ‘A survey of attitude representations’, The Journal of the Astronautical Sciencies 41(4), 439–517.  
Ulrich, E. R., Humbert, J. S. & Pines, D. J. (2010), ‘Pitch and heave control of robotic samara micro air vehicles’, Journal of Aircraft  
  47(4), 1290–1299.  
Voyles, R. & Jiang, G. (2012), Hexrotor UAV platform enabling dextrous interaction with structures - preliminary work, in ‘IEEE  
  International Symposium on Safety, Security, and Rescue Robotics (SSRR)’, IEEE, pp. 1–7.  
Weiss, S., Achtelik, M. W., Lynen, S., Achtelik, M. C., Kneip, L., Chli, M. & Siegwart, R. (2013), ‘Monocular vision for long-term micro  
  aerial vehicle state estimation: A compendium’, Journal of Field Robotics 30(5), 803–831.  
26  
  
  
Youngren, H., Jameson, S. & Satterfield, B. (2009), Design of the SAMARAI monowing rotorcraft nano air vehicle, in ‘Proceedings of  
  the American Helicopter Society AHS 65th Annual Forum and Technology Display’.  
Zhang, Y., Chamseddine, A., Rabbath, C., Gordon, B., Su, C.-Y., Rakheja, S., Fulford, C., Apkarian, J. & Gosselin, P. (2013), ‘Development  
  of advanced FDD and FTC, techniques with application to an unmanned quadrotor helicopter testbed’, Journal of the Franklin Institute  
  350(9), 2396–2422.  
Zipfel, P. H. (2007), Modeling and Simulation of Aerospace Vehicle Dynamics Second Edition, American Institute of Aeronautics and  
  Astronautics.  
Zou, J.-T., Su, K.-L. & Tso, H. (2012), ‘The modeling and implementation of tri-rotor flying robot’, Artificial Life and Robotics 17(1), 86–  
  91.  
  
  
A. Index to multimedia extensions  
The multimedia extensions to this article are at: http://www.ijrr.org.  
    Extension Media type Description  
        1            Video            Quadrocopter failsafe flight experiments  
        2            Video            Novel vehicles flight experiments  
  
