# Conference Series

**Authors:** Unknown_Author
**Year:** 2024
**Source URL:** Local File: Design_and_aerodynamic_analysis_of_a_samara-inspir.pdf

## Abstract
Abstract extraction failed. Please read the PDF.

## My Notes
- Start typing your research notes here...

---

## Full Paper Text
Journal of Physics:  
Conference Series  
  
  
  
PAPER • OPEN ACCESS                                                                              You may also like  
                                                                                                     - Research on Passive Location Strategy of  
Design and aerodynamic analysis of a samara-                                                           UAV  
                                                                                                       Yiming Lu  
inspired unmanned aerial vehicle                                                                     - Research on the Working Performance of  
                                                                                                       UAV Hybrid Power System Based on  
                                                                                                       Mission Profile  
To cite this article: Hongming Shi 2025 J. Phys.: Conf. Ser. 2977 012018                               Xiyuan Chai, Xin Fu and Shijie Zhai  
  
                                                                                                     - Tracking control a twin-rotor helicopter  
                                                                                                       used for building measuring with PID-MPC  
                                                                                                       controller  
                                                                                                       F Guo  
View the article online for updates and enhancements.  
  
  
  
  
                             This content was downloaded from IP address 119.13.193.138 on 20/03/2026 at 21:36  
ICOACE-2024                                                                                                              IOP Publishing  
Journal of Physics: Conference Series 2977 (2025) 012018                                       doi:10.1088/1742-6596/2977/1/012018  
  
  
  
  
Design and aerodynamic analysis of a samara-inspired  
unmanned aerial vehicle  
  
                     Hongming Shi*  
                     The Chinese University of Hong Kong, Hong Kong, China  
                     *  
                         Corresponding author: Hongming Shi, Email: miaonyameow@outlook.com  
  
                     Abstract. This paper presents the design and aerodynamic analysis of a novel unmanned aerial  
                     vehicle (UAV) inspired by the samara seed’s natural flight mechanics. The UAV is designed to  
                     emulate the self-stabilizing spin behavior of the winged seed during descent, offering an  
                     innovative approach to UAV swarm applications. The aircraft incorporates a spinning wing, a  
                     load-bearing fuselage, and integrated propulsion and control systems, optimized for both  
                     military and civilian use. Using computational fluid dynamics (CFD) simulations, the  
                     aerodynamic performance of the UAV was evaluated under various operational conditions,  
                     focusing on lift generation, stability, and control dynamics. The results demonstrate that the  
                     samara-inspired design achieves stable spin flight and adequate lift to carry a payload while  
                     maintaining lateral stability. Potential applications for this design include precision aerial  
                     delivery, reconnaissance, and environmental monitoring.  
  
  
  
1. Introduction  
Conventional UAV designs, particularly fixed-wing and rotary-wing models, face limitations in low-  
speed stability, vertical descent, and hovering. Fixed-wing UAVs provide excellent aerodynamic  
efficiency but lack the maneuverability necessary for precise, localized tasks, such as those required in  
search-and-rescue operations or environmental sampling. Rotary-wing UAVs, such as quadcopters,  
offer hovering capabilities but often suffer from lower efficiency and shorter flight times [1–3].  
    In response to these limitations, researchers have increasingly turned to bio-inspired designs,  
particularly those that emulate the natural flight characteristics of winged seeds, such as samaras.  
Samara seeds, found in trees like maples, spin as they fall, generating lift and prolonging their descent  
[4–5]. This natural mechanism of autorotation provides inherent stability and has the potential to  
enhance the capabilities of UAVs, particularly in swarm applications where stability and extended  
flight times are critical [6–7].  
    Several studies have explored the application of samara-inspired designs in UAVs. For instance,  
Win et al. [8] developed a UAV prototype that achieved stable spin flight by leveraging the  
aerodynamic properties of a single rotating wing. This work demonstrated the feasibility of using  
natural flight mechanisms to enhance UAV performance. Soh et al. [9] and Thomas et al. [10]  
introduced propulsion systems to these designs, which allowed for greater maneuverability and control  
during flight. These efforts have significantly advanced the understanding of bio-inspired UAVs,  
particularly in terms of aerodynamic performance.  
    Despite these advances, challenges remain. Most existing designs focus on basic aerodynamic  
performance, with limited attention to control system integration or payload capacity. Moreover, the  
scalability of samara-inspired UAVs for swarm applications has not been thoroughly explored. Other  
  
  
  
              Content from this work may be used under the terms of the Creative Commons Attribution 4.0 licence. Any further distribution  
              of this work must maintain attribution to the author(s) and the title of the work, journal citation and DOI.  
Published under licence by IOP Publishing Ltd                          1  
ICOACE-2024                                                                                   IOP Publishing  
Journal of Physics: Conference Series 2977 (2025) 012018                 doi:10.1088/1742-6596/2977/1/012018  
  
  
  
researchers, such as Lee et al. [11] and Win et al. [12], have begun investigating the role of advanced  
control systems in samara-like UAVs, but further research is needed to fully realize their potential.  
   This study seeks to address these gaps by presenting a comprehensive design and analysis of a  
samara-inspired UAV, focusing on both aerodynamic performance and control system integration. By  
combining bio-inspired design principles with modern UAV technologies, we aim to create a UAV  
that offers superior stability, extended flight times, and precise control, making it suitable for a wide  
range of applications.  
  
2. Design of the samara-inspired UAV  
  
2.1 Conceptual design and structural layout  
The design of the samara-inspired UAV draws directly from the unique aerodynamic properties of  
samara seeds, as shown in Figure 1(a), which generate lift through autorotation as they descend. A  
comparative analysis of samara seed biomechanics reveals several critical principles that inform this  
UAV’s design. First, the asymmetric wing structure, typically observed in samara seeds, creates an  
aerodynamic imbalance that facilitates rotational motion, generating lift through a continuous low-  
pressure vortex along the wing’s upper surface. This UAV design incorporates an elongated, single-  
wing configuration, optimized for a controlled descent through steady autorotation. The single rotating  
wing, spanning 400 mm with a chord length of 80 mm, mimics the seed’s ability to convert vertical  
descent into rotational kinetic energy, stabilizing flight and minimizing power consumption.  
    The wing is modeled after the flat, elongated structure of the samara seed, which uses autorotation  
to maintain a slow, controlled descent. Inspired by the thin, flexible profile of samara wings, the UAV  
utilizes a Clark Y airfoil with a maximum thickness of 11.7% of the chord length. This design, similar  
to the seed’s wing, balances lift and drag to sustain a low descent rate, thereby enhancing stability.  
Additionally, the Clark Y airfoil’s high lift-to-drag ratio and favorable stall characteristics ensure that  
the UAV remains controllable even at low speeds, a key advantage for tasks requiring precision.  
Another essential design principle derived from samara biomechanics is mass distribution, which is  
crucial for maintaining rotational stability. The wing’s span is 400 mm, while the chord length  
measures 80 mm, resulting in a total wing area of 0.032 m². This size was chosen after multiple  
iterations and calculations to balance lift generation with structural simplicity. The Clark Y airfoil,  
well-suited for low-speed flight, was selected for its high lift-to-drag ratio and favorable stall  
characteristics, as shown in Figure 1(b). The Clark Y airfoil has a maximum thickness of 11.7% of the  
chord length, providing the strength needed to withstand aerodynamic loads while maintaining  
flexibility for autorotation.  
    The propulsion system further integrates bio-inspired principles, using a single brushless motor  
mounted at the wingtip. This motor provides the initial thrust required to initiate autorotation, after  
which minimal power is needed to sustain rotational flight, emulating the energy efficiency of samara  
seeds that descend with no active propulsion. By only supplementing with power when necessary, this  
UAV design capitalizes on natural aerodynamic stability to achieve a longer flight duration with  
reduced energy consumption. This motor, a TOKA 2004-1700KV model, is selected for its balance  
between power output and lightweight construction. With a KV rating of 1700 and a maximum power  
output of 418.1 W, the motor provides sufficient torque to spin the wing at speeds up to 10 rad/s. The  
propeller, a 5-inch carbon fiber TAROT 5045, ensures efficient thrust generation while minimizing  
drag, as shown in Figure 1(c). This design enables the UAV to maintain a rotational speed necessary  
for stable flight, generating the lift needed to support the payload.  
  
2.2 Wing design and aerodynamic considerations  
The wing design plays a pivotal role in the UAV’s ability to generate lift through autorotation. The  
rectangular planform, combined with the Clark Y airfoil, was chosen after a thorough evaluation of  
several airfoil profiles. The Clark Y airfoil is particularly suitable due to its performance in low-speed  
regimes, such as those encountered during descent. At an angle of attack of 6°, the airfoil achieves a  
  
  
  
                                                           2  
ICOACE-2024                                                                                   IOP Publishing  
Journal of Physics: Conference Series 2977 (2025) 012018                 doi:10.1088/1742-6596/2977/1/012018  
  
  
  
lift coefficient of 1.2 and a drag coefficient of 0.02. The maximum lift-to-drag ratio is 60, which  
ensures that the UAV can sustain its flight for extended periods without excessive energy consumption.  
    In terms of mass distribution, the design ensures that the center of mass is aligned with the rotation  
axis, which minimizes instability during flight. The lightweight construction of the wing and fuselage  
also reduces the overall moment of inertia, allowing the UAV to rotate smoothly and maintain lateral  
stability.  
  
  
  
  
Figure 1. Global design of the samara-inspired UAV. (a) The structure of samara. (b) General view of  
                samara-inspired UAV. (c) Exploded view of samara-inspired UAV.  
    It is also seen that the specimen SJ1 has the largest deformation, and the maximum deflection of  
the specimen is up to 16 mm, which shows the better plastic deformation ability of the slab. It is also  
seen that the specimen SJ1 has the largest deformation, and the maximum deflection of the specimen  
is up to 16 mm, which shows the better plastic deformation ability of the slab. It is also seen that the  
specimen SJ1 has the largest deformation, and the maximum deflection of the specimen is up to 16  
mm, which shows the better plastic deformation ability of the slab.  
  
2.3 Control systems and propulsion  
The control system is designed to adjust the angle of attack dynamically, thus enabling the UAV to  
control its descent rate, maintain stability, and alter its horizontal trajectory. Two digital servomotors  
are mounted at the base of the wing, controlling the pitch angle. These servos, with a torque of 1.8  
kg/cm and a response frequency of 144 Hz, provide precise control over the wing’s angle of attack,  
allowing the UAV to react swiftly to changes in flight conditions.  
    The servos are integrated with the flight controller, which processes data from the UAV’s onboard  
sensors, including accelerometers, gyroscopes, and an altimeter. The flight controller continuously  
monitors the UAV’s orientation and flight parameters, adjusting the wing’s angle of attack in real time  
to ensure stable flight. This closed-loop control system enables the UAV to adapt to environmental  
changes, such as gusts of wind, while maintaining its intended flight path.  
    The propulsion system is powered by an 11.1 V, 2200 mAh Li-Po battery, providing enough  
energy for approximately 20 minutes of continuous flight under normal conditions. The motor  
operates efficiently at rotational speeds between 5000 and 10,000 RPM, generating a maximum thrust  
of 1.8 N. During powered flight, the motor provides additional thrust to extend the UAV’s range,  
while in autorotation mode, it operates at reduced power to conserve battery life.  
  
  
  
                                                           3  
ICOACE-2024                                                                                 IOP Publishing  
Journal of Physics: Conference Series 2977 (2025) 012018               doi:10.1088/1742-6596/2977/1/012018  
  
  
  
3. Aerodynamic simulation and analysis  
  
3.1 CFD methodology  
The aerodynamic performance of the samara-inspired UAV was evaluated using computational fluid  
dynamics (CFD) simulations in ANSYS Fluent. To enhance the realism of these simulations,  
additional tests were conducted under complex flight environments that emulate practical application  
conditions. Instead of relying solely on idealized, free-stream conditions, we incorporated non-  
standard atmospheric variables, including variable air density, temperature gradients, and simulated  
turbulence, to better reflect the UAV’s performance in real-world scenarios. This approach provides a  
comprehensive understanding of the UAV’s stability, lift, and energy consumption across diverse  
environmental conditions. The primary objectives were to analyze the lift generated by the spinning  
wing, examine the lateral stability of the UAV during descent, and assess the drag and torque forces  
acting on the system.  
    One key factor considered was air density variation, which can significantly impact the UAV’s lift  
and drag. In high-altitude scenarios, where air density decreases, the UAV’s lift generation capacity is  
naturally reduced. Conversely, low-altitude, high-density conditions, such as near sea level, enhance  
lift but increase drag, potentially affecting the UAV’s energy efficiency. By simulating a range of  
altitudes, from 0 to 3000 meters above sea level, the CFD analysis revealed how the UAV’s flight  
characteristics adapt to different air densities, providing insight into operational efficiency across a  
variety of altitudes. Temperature gradients were also introduced, simulating conditions ranging from  
cold environments (0°C) to high temperatures (40°C). These temperature shifts impact air viscosity  
and density, subtly altering the lift-to-drag ratio. The CFD results showed that, under high-temperature  
conditions, the UAV required a slightly higher rotational speed to achieve the same lift as in cooler  
environments, underscoring the importance of temperature-aware control adjustments in operational  
planning. Turbulence was a further variable added to the CFD simulations to model sudden wind gusts  
and unsteady airflow.  
  
  
  
  
Figure 2. CFD finite element simulation modeling. (a) Simplified model of the samara-inspired UAV.  
 (b) Grid effect diagram of samara-inspired UAV. (c) Dynamic lattice regions and outflow fields. (d)  
                            Unstructured grid based on flow field structure.  
   The CFD simulations used a 3D model of the UAV, which was developed in CAD software and  
imported into ANSYS Fluent for analysis, as shown in Figures 2(a-b). The simulation environment  
was configured to simulate free-stream conditions typical of flight at altitudes between 50 and 100 m.  
The Reynolds-Averaged Navier-Stokes (RANS) equations were used to model airflow, with a k-  
epsilon turbulence model to capture the effects of rotational flow around the spinning wing.  
   A cylindrical control volume was created around the UAV to represent the surrounding air, with  
boundary conditions set to ensure that airflow remained uniform and undisturbed at the domain’s  
edges, as shown in Figures 2(c-d). The free-stream velocity was set to 5 m/s, representing typical  
operating conditions in light wind. The air density was set at 1.225 kg/m³, corresponding to standard  
atmospheric conditions.  
  
  
                                                           4  
ICOACE-2024                                                                                   IOP Publishing  
Journal of Physics: Conference Series 2977 (2025) 012018                 doi:10.1088/1742-6596/2977/1/012018  
  
  
  
   The wing’s rotational speed was varied between 4 and 10 rad/s, with the angle of attack adjusted  
between 2° and 16° to assess the UAV’s performance under different flight conditions. The UAV’s  
drag, lift, and torque coefficients were monitored throughout the simulations to provide detailed  
insight into its aerodynamic behavior.  
  
3.2 Simulation setup and parameters  
The UAV’s wing was modeled using an unstructured mesh to capture the fine details around the  
leading and trailing edges of the airfoil. This mesh was refined at critical regions, such as the wing’s  
tips and around the fuselage, to ensure accurate predictions of flow separation and vortex shedding.  
The mesh size near the wing was reduced to 0.002 m to resolve the boundary layer accurately, while  
the overall mesh consisted of approximately 3 million elements.  
    The wing rotation rate was simulated at three distinct speeds - 4, 6, 8, and 10 rad/s - to evaluate its  
performance under varying conditions. At each rotational speed, the wing’s angle of attack was varied  
from 2° to 16° in 2° increments. This range of angles was chosen to ensure that both low-lift and high-  
lift configurations were analyzed, as well as potential stall conditions.  
    The simulation was conducted as a transient analysis to capture the unsteady nature of the flow  
around the spinning wing. Each simulation ran for 0.5 seconds of real time, which corresponds to  
approximately 30 complete rotations of the wing at the lowest speed. The time step was set to 0.001  
seconds to ensure sufficient resolution of the wing’s rotational motion.  
  
3.3 Results: lift, drag, and torque analysis  
The CFD simulations revealed several key insights into the aerodynamic performance of the UAV’s  
wing. In Figure 3(a), which shows the lifting force as a function of the facing angle for various  
rotational speeds, additional data points have been added at higher rotational speeds (up to 18 rad/s).  
These data points provide a more comprehensive view of the lift dynamics across a wider range of  
operational conditions, highlighting how lift generation increases significantly with both the angle of  
attack and the rotational speed. At lower rotational speeds (4-8 rad/s), the lift generated is relatively  
modest, showing gradual increases as the facing angle rises. This trend indicates that at low RPMs, the  
UAV’s lifting capacity is limited, and achieving substantial lift would require a high angle of attack,  
which may lead to increased drag and potential instability. At a rotational speed of 10 rad/s and an  
angle of attack of 6°, the UAV generated a lift force of 4.2 N, sufficient to support a payload of 200 g,  
as shown in Figure 3(a). The lift coefficient reached a maximum of 1.3, while the drag coefficient  
remained low at 0.03, resulting in a high lift-to-drag ratio of 43.3. However, as the rotational speed  
increases beyond 10 rad/s, the lift generated at each facing angle shows a marked increase. For  
instance, at 12 rad/s and a facing angle of 10°, the lift force reaches approximately 6 N, while at 18  
rad/s, the lift force at the same angle nearly doubles to 12 N. This relationship underscores a critical  
design insight: by operating at higher rotational speeds, the UAV can achieve significant lift without  
requiring excessively high facing angles, which reduces the aerodynamic drag typically associated  
with large angles of attack. The data also reveal that at extreme angles, close to 16°, lift generation  
begins to plateau or increase at a slower rate even with higher RPMs. This plateau suggests the onset  
of aerodynamic limits, where further increases in the angle of attack yield diminishing returns in lift  
due to flow separation effects at the wing’s trailing edge. These results confirm that the wing design is  
capable of maintaining stable flight and achieving the necessary lift for operational tasks.  
    The torque analysis showed that the rotational motion remained stable across all tested angles of  
attack. As the facing angle increases, the resistance torque initially remains stable but then rises  
sharply after 10°, reaching a maximum at 16°. At a speed of 10 rad/s, the torque generated by  
aerodynamic forces acting on the wing reached a peak of 0.35 N·m, as shown in Figure 3(b). This  
value was well within the operating limits of the UAV’s motor, confirming that the system can  
maintain autorotation without excessive energy consumption. These results validate the UAV’s design  
strategy of balancing rotational speed and angle of attack to optimize lift without incurring excessive  
  
  
  
  
                                                           5  
ICOACE-2024                                                                                   IOP Publishing  
Journal of Physics: Conference Series 2977 (2025) 012018                 doi:10.1088/1742-6596/2977/1/012018  
  
  
  
resistance torque. This balance is essential for sustaining flight efficiency, as higher torque demands  
would lead to greater energy consumption.  
  
  
  
  
Figure 3. Simulation results analysis. (a) Variation curves of lift as a function of the orientation angle  
 of the guided rotor wing for different rotation speeds. (b) Variation curve of drag moment with the  
                            angle of orientation of the guided rotor wing.  
  
4. Control mechanisms  
  
4.1 Design of the control system  
The control system for the samara-inspired UAV is designed to ensure stable spin flight while  
providing the capability to adjust descent rate and horizontal movement. Unlike traditional UAVs that  
rely on multiple rotors for control, the samara-inspired UAV uses two servomotors to dynamically  
adjust the wing’s angle of attack. These servos allow the UAV to control its lift and drag, effectively  
managing its flight path and stability.  
    The servos are connected to the flight controller, which continuously monitors the UAV’s  
orientation using data from onboard sensors. The accelerometer and gyroscope provide real-time  
feedback on the UAV’s rotational speed and angular velocity, while the altimeter measures the UAV’s  
altitude. This data is processed by the flight controller’s PID (Proportional-Integral-Derivative) control  
algorithm, which adjusts the servos to maintain stable flight.  
  
4.2 Spin flight stabilization  
Maintaining stability during spin flight is a key challenge for the UAV. The spinning wing generates  
both lift and centrifugal forces, which must be carefully balanced to avoid instability. To achieve this  
balance, the servomotors adjust the wing’s pitch in response to changes in the UAV’s flight dynamics.  
For example, if the UAV’s descent rate increases too rapidly, the servos will increase the angle of  
attack to generate more lift and slow the descent.  
    In addition to controlling descent, the servos also allow for lateral maneuvering. By adjusting the  
pitch of one side of the wing more than the other, the UAV can generate a yawing moment, enabling it  
to turn during flight. This level of control makes the UAV suitable for a variety of applications,  
including precision landing and aerial delivery.  
  
4.3 Integration of sensors and actuators  
The UAV’s control system integrates several key components to ensure precise control during flight.  
The accelerometers and gyroscopes provide data on the UAV’s rotational motion, while the altimeter  
monitors altitude changes. These sensors feed real-time data into the flight controller, which uses this  
information to adjust the wing’s pitch and rotational speed. To enhance robustness, additional response  
strategies were developed to handle potential sensor failure or data loss, ensuring continuous operation  
even under adverse conditions. In the event of sensor failure, a hierarchical fallback system activates.  
For instance, if the primary gyroscope fails, the control system automatically switches to backup  
gyroscopes or relies on accelerometer data to approximate the UAV’s orientation. This redundancy  
allows the UAV to maintain stability until repairs can be made. The servomotors, capable of high-  
  
  
  
                                                           6  
ICOACE-2024                                                                                   IOP Publishing  
Journal of Physics: Conference Series 2977 (2025) 012018                 doi:10.1088/1742-6596/2977/1/012018  
  
  
  
frequency adjustments, allow the UAV to respond quickly to environmental changes, such as wind  
gusts or obstacles.  
   The integration of these components ensures that the UAV can maintain stable flight under a wide  
range of conditions. The servomotors’ ability to adjust the wing’s angle of attack in real time enables  
the UAV to maintain its desired flight path, even in turbulent environments.  
  
5. Results and discussion  
The CFD simulations and control system analysis provided valuable insights into the performance of  
the samara-inspired UAV. A detailed energy efficiency analysis was conducted to understand the  
UAV’s power requirements under various flight conditions. Energy consumption was monitored  
across different rotational speeds, altitudes, and angles of attack, revealing that higher rotational  
speeds and increased facing angles lead to significant energy demands due to higher lift and drag  
forces. Comparisons were made between powered flight and autorotation modes, with findings  
indicating that autorotation consumes up to 40% less energy, highlighting it as an effective energy-  
saving mode for descent. To further enhance efficiency, potential energy-saving measures were  
explored, including adaptive power modulation, which adjusts motor output based on real-time lift  
requirements, and optimized flight path planning to minimize high-drag maneuvers. These measures  
have the potential to extend flight duration, especially in low-power, hovering tasks. Compared to  
traditional UAV designs, the samara-inspired model demonstrated superior stability during vertical  
descent, with the spinning wing generating sufficient lift to support a payload of up to 200 g. The  
UAV’s aerodynamic efficiency was also significantly higher than that of rotary-wing UAVs,  
particularly during low-speed flight.  
    To illustrate the practical value of the design, several potential application scenarios were analyzed  
in detail. For search and rescue operations, the UAV’s stable autorotation and precise control make it  
ideal for hovering over specific areas, assisting in locating individuals in challenging terrains. Its  
ability to maintain stability in mild turbulence ensures reliability in diverse weather conditions. In  
precision agriculture, the UAV’s low-energy consumption and extended flight time allow for effective  
crop monitoring and targeted pesticide or fertilizer application over large fields, increasing agricultural  
efficiency. These application scenarios highlight the UAV’s adaptability and utility in real-world  
situations, showcasing how its bio-inspired design can meet the specific demands of tasks requiring  
prolonged flight, stability, and low-energy operation.  
    Overall, the samara-inspired design offers a promising alternative to traditional UAV  
configurations, particularly for applications that require stability and extended flight times. Future  
research will focus on improving the UAV’s control algorithms and exploring additional use cases,  
such as search-and-rescue missions and precision agriculture.  
  
6. Conclusion  
This paper presented the design and aerodynamic analysis of a samara-inspired UAV, leveraging the  
natural stability and lift-generating characteristics of the samara seed to achieve stable spin flight. The  
UAV’s aerodynamic performance was thoroughly evaluated using CFD simulations, which confirmed  
its ability to carry a payload of 200 g while maintaining stability and control.  
    The results of this study highlight the potential of samara-inspired designs for a wide range of UAV  
applications, particularly those that require precise control and extended flight times. However, further  
research is needed to optimize the UAV’s energy efficiency and explore its scalability for swarm  
applications. Future work will focus on refining the control system and integrating more advanced  
sensor technologies to enhance the UAV’s capabilities.  
  
References  
[1] Van Laake, L.C. and Overvelde, J.T.B (2024) Bio-inspired autonomy in soft robots, Commun.  
        Mater., 5, 198.  
  
  
  
  
                                                           7  
ICOACE-2024                                                                             IOP Publishing  
Journal of Physics: Conference Series 2977 (2025) 012018           doi:10.1088/1742-6596/2977/1/012018  
  
  
  
  
[2] Shahzad MM, Saeed Z, Akhtar A, Munawar H, Yousaf MH, Baloach NK and Hussain F (2023) A  
          Review of Swarm Robotics in a NutShell. Drones, 7(4):269.  
[3] Deb, D., Huang, K. and Verma, A. (2023) Thrust enhancement and degradation mechanisms due to  
          self-induced vibrations in bio-inspired flying robots. Sci. Rep., 13, 18317.  
[4] Pan, Y., Guo, S and Huang, X (2024) Research Progress on Bio-inspired Flapping-Wing Rotor  
          Micro Aerial Vehicle Development. J. Bionic Eng., 21, 1621–1643.  
[5] Rinaldi M, Primatesta S and Guglieri G (2023) A Comparative Study for Control of Quadrotor  
          UAVs. Appl. Sci., 13(6):3464.  
[6] Othman, A.K., Zekry, D.A. and Saro-Cortes, V (2023) Aerial and aquatic biological and  
          bioinspired flow control strategies. Commun. Eng. 2, 30.  
[7] Schaeffer, B.M., Truman, S.S. and Truscott, T.T (2024) Maple samara flight is robust to  
          morphological perturbation and united by a classic drag model. Commun. Biol., 7, 248.  
[8] S. K. H. Win, L. S. T. Win, D. Sufiyan, G. S. Soh and S. Foong (2022) An Agile Samara-Inspired  
          Single-Actuator Aerial Robot Capable of Autorotation and Diving. IEEE Trans. on Robot.,  
          38(2), 1033-1046.  
[9] Win, S.K.H., Win, L.S.T. and Soh, G.S (2019) Design, modelling and control of collaborative  
          samara autorotating wings (SAW). Int. J. Intell. Robot. Appl. 3, 144–157.  
[10] Thomas, S., Ho, D., Kerroux, A., Lixi, L., Rackham, N. and Rosenfeld, S. (2014) Concept and  
          design of a biomimetic single-wing MAV. Inter. J. Unman. Sys. Eng., 2(4), 16-26.  
[11] Nave GK Jr., Hall N, Somers K, Davis B, Gruszewski H, Powers C, Collver M, Schmale DG III  
          and Ross SD (2021) Wind Dispersal of Natural and Biomimetic Maple Samaras.  
          Biomimetics. 6(2):23.  
[12] S. K. H. Win, L. S. T. Win, D. Sufiyan, G. S. Soh and S. Foong (2019) Dynamics and Control of  
          a Collaborative and Separating Descent of Samara Autorotating Wings. IEEE Robot. Autom.  
          Lett., 4(3), 3067-3074.  
  
  
  
  
                                                           8  
  
