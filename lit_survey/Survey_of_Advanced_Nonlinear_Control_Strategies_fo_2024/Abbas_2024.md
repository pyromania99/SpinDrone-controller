# Survey of Advanced Nonlinear Control Strategies for UAVs: Integration of Sensors and Hybrid Techniques

**Authors:** Nadir Abbas and Zeshan Abbas and Samra Zafar and Naseem Ahmad and Xiaodong Liu and Saad Saleem Khan and Eric Deale Foster and Stephen Larkin
**Year:** 2024
**Source URL:** Local File: Survey_of_Advanced_Nonlinear_Control_Strategies_fo.pdf

## Abstract
Vehicles (UAVs), including systems such as the Twin Rotor MIMO system (TRMS) and quadrotors. UAVs, with their high nonlinearity and significant coupling effects, serve as crucial benchmarks for testing control algorithms. Integration of sophisticated sensors enhances UAV versatility, making traditional linear control techniques less effective. Advanced nonlinear strategies, including sensorbased adaptive controls and AI, are increasingly essential. Recent years have seen the development of diverse sliding surface-based, sensor-driven, and hybrid control strategies for UAVs, offering superior performance over linear methods. This paper reviews the significance of these strategies, emphasizing their role in addressing UAV complexities and outlining future research directions. Citation: Abbas, N.; Abbas, Z.; Zafar, S.; Ahmad, N.; Liu, X.; Khan, S.S.; Foster, E.D.; Larkin, S. Survey of Keywords: perturbed MIMO system; robust adaptive nonlinear control; sensor integration; intelligent control; UAV dynamics Advanced Nonlinear Control Strategies for UAVs: Integration of Sensors and Hybrid Techniques. Sensors 2024, 24, 3286. https:// doi.org/10.3390/s24113286 Academic Editor: Carlos

## My Notes
- Start typing your research notes here...

---

## Full Paper Text
             sensors  
Review  
Survey of Advanced Nonlinear Control Strategies for UAVs:  
Integration of Sensors and Hybrid Techniques  
Nadir Abbas 1 , Zeshan Abbas 2 , Samra Zafar 3 , Naseem Ahmad 1 , Xiaodong Liu 1, *, Saad Saleem Khan 4 ,  
Eric Deale Foster 5 and Stephen Larkin 5  
  
                                         1   School of Control Science and Engineering, Dalian University of Technology,  
                                             Dalian 116024, China; nadir10@mail.dlut.edu.cn (N.A.); engr.naseem99@mail.dlut.edu.cn (N.A.)  
                                         2   Institute of Ultrasonic Technology, Shenzhen Polytechnic University, Shenzhen 518055, China;  
                                             abbasz@szpu.edu.cn  
                                         3   Department of Computer Science, School of Control Science and Engineering,  
                                             Dalian University of Technology, Dalian 116024, China; samra@mail.dlut.edu.cn  
                                         4   Department of Electrical Engineering, College of Engineering, United Arab Emirates University,  
                                             Al-Ain 15551, United Arab Emirates; 201590095@uaeu.ac.ae  
                                         5   Omega Aviation Ltd., Leicester LE9 4LG, UK; eric.foster@omegaaviation.co.uk (E.D.F.);  
                                             stephen.larkin@omegaaviation.co.uk (S.L.)  
                                         *   Correspondence: xdliuros@dlut.edu.cn  
  
  
                                         Abstract: This survey paper explores advanced nonlinear control strategies for Unmanned Aerial  
                                         Vehicles (UAVs), including systems such as the Twin Rotor MIMO system (TRMS) and quadrotors.  
                                         UAVs, with their high nonlinearity and significant coupling effects, serve as crucial benchmarks for  
                                         testing control algorithms. Integration of sophisticated sensors enhances UAV versatility, making  
                                         traditional linear control techniques less effective. Advanced nonlinear strategies, including sensor-  
                                         based adaptive controls and AI, are increasingly essential. Recent years have seen the development  
                                         of diverse sliding surface-based, sensor-driven, and hybrid control strategies for UAVs, offering  
                                         superior performance over linear methods. This paper reviews the significance of these strategies,  
                                         emphasizing their role in addressing UAV complexities and outlining future research directions.  
  
Citation: Abbas, N.; Abbas, Z.; Zafar,  
                                         Keywords: perturbed MIMO system; robust adaptive nonlinear control; sensor integration; intelligent  
S.; Ahmad, N.; Liu, X.; Khan, S.S.;  
Foster, E.D.; Larkin, S. Survey of  
                                         control; UAV dynamics  
Advanced Nonlinear Control  
Strategies for UAVs: Integration of  
Sensors and Hybrid Techniques.  
Sensors 2024, 24, 3286. https://         1. Introduction  
doi.org/10.3390/s24113286                     The integration of Unmanned Aerial Vehicles (UAVs) with the Internet of Things (IoT)  
Academic Editor: Carlos                  holds significant promise for the advancement of smart cities [1]. Efficient management of  
Tavares Calafate                         UAV flight trajectories is crucial for optimizing the lifespan of onboard systems and energy  
                                         expenditure, which is essential for extensive data acquisition and processing [2]. UAVs  
Received: 21 February 2024               play a pivotal role in bridging data gaps, particularly in remote areas, thereby enhancing  
Revised: 12 May 2024  
                                         environmental surveillance efforts. Their successful deployment across various scenarios  
Accepted: 16 May 2024  
                                         underscores their transformative potential. The adept maneuvering of UAVs coupled  
Published: 21 May 2024  
                                         with sophisticated navigational controls is essential for urban development. Seamlessly  
                                         integrated UAVs can navigate urban environments to collect the comprehensive sensor  
                                         data vital for applications such as traffic management, environmental monitoring, and  
Copyright: © 2024 by the authors.  
                                         structural inspections. These precise control mechanisms facilitate real-time data flow,  
Licensee MDPI, Basel, Switzerland.       enabling informed decision-making and timely actions [3].  
This article is an open access article        Resource optimization within smart cities is further augmented by UAVs’ specialized  
distributed under the terms and          capabilities. Their aerial perspective facilitates the monitoring of public utilities, enabling  
conditions of the Creative Commons       swift identification of service disruptions and supporting maintenance activities. Proactive  
Attribution (CC BY) license (https://    detection of inefficiencies or failures ensures the continuous operation of critical services,  
creativecommons.org/licenses/by/         including water and power distribution and waste management [4]. In terms of public  
4.0/).                                   safety, precisely controlled UAVs play a vital role. They provide a comprehensive view for  
  
  
  
Sensors 2024, 24, 3286. https://doi.org/10.3390/s24113286                                                    https://www.mdpi.com/journal/sensors  
Sensors 2024, 24, 3286                                                                                        2 of 51  
  
  
  
                         crowd surveillance, detect anomalies, and contribute to situational awareness essential for  
                         public security operations. Guided by robust control frameworks, UAVs are able to adeptly  
                         navigate urban environments, adhere to designated flight paths, and promptly respond to  
                         security incidents [5].  
                               During crisis events such as natural calamities or urban mishaps, the deployment of  
                         UAVs with advanced control systems is invaluable. Their ability to swiftly access otherwise  
                         unreachable zones and relay vital insights can revolutionize emergency response strategies,  
                         potentially saving lives and mitigating the impact of disasters. Furthermore, UAVs are set  
                         to revolutionize urban mobility and logistics. Implementing efficient control algorithms  
                         allows UAVs to aid in decongesting roads, optimizing traffic flows, and enhancing route  
                         planning. They additionally stand to streamline delivery services, easing the burden on  
                         urban transit systems and elevating the efficacy of city-wide distribution networks [6].  
                               Over the last two decades, UAVs, commonly known as drones, have carved out a niche  
                         in geosciences and remote sensing, becoming a pivotal tool thanks to their adaptability  
                         and cost-effectiveness. Their ascent to prominence is mirrored in the surge of scholarly  
                         articles examining UAV applications, with Scopus documenting over 80,000 publications  
                         since 2001 that feature terms such as “UAV”, “drone”, “UAS”, and “RPAS” in Figure 1.  
                         This scholarly attention predominantly emanates from fields such as engineering and  
                         computer science [7]. The scientific intrigue around UAVs spans various citation indexes,  
                         underscoring a universal trend towards their study and development. Financially, the  
                         UAV sector has seen a robust upswing, with valuations running into the billions annually.  
                         While the current market is heavily skewed towards military use, the horizon for civilian  
                         applications of UAVs is broadening, propelled by economic factors, technological strides,  
                         miniaturization of sensors, and innovations in software and algorithmic approaches [3,7].  
                         UAVs are making inroads into diverse sectors, with the construction industry being a prime  
                         example [8]. Their ability to capture high-resolution images and gather extensive data  
                         offers a cost-efficient alternative for continuous observation and management of project  
                         timelines [9].  
  
  
  
  
                         Figure 1. Scientific research articles according to SCOPUS [10].  
  
                              Efficient control systems enable UAVs to navigate through densely populated areas,  
                         avoid obstacles, and manage energy consumption while ensuring that they collect accu-  
                         rate and comprehensive data. These data are invaluable for applications such as traffic  
                         management, environmental monitoring, and public safety. Control strategies allow UAVs  
                         to operate autonomously, make decisions in real-time, and interact seamlessly with other  
                         IoT devices, contributing to the overall intelligence and responsiveness of smart city in-  
                         frastructures [11]. In the context of geoscience and remote sensing, which was the focus  
Sensors 2024, 24, 3286                                                                                          3 of 51  
  
  
  
                         of the second rephrased text, control strategies are just as important. They enable UAVs  
                         to conduct detailed surveys of the Earth’s surface, including areas that are otherwise inac-  
                         cessible. With precise control, UAVs can execute complex flight patterns to systematically  
                         collect geospatial data, which is crucial for mapping, natural resource management, disaster  
                         response, and environmental studies [11]. In both applications, the control strategies form  
                         the backbone of UAV operation, determining their efficiency, effectiveness, and adaptability  
                         to various tasks. Whether following a predetermined flight path or responding to dynamic  
                         conditions in real time, the control system is what enables a UAV to meet its operational  
                         objectives, making it a key focus for ongoing research and development in the field [12].  
                               Having establishing the landscape of advanced nonlinear control strategies for UAVs in  
                         this introduction, we now turn to this survey’s contributions. By offering a comprehensive  
                         review and highlighting the significance of sensor integration and hybrid techniques,  
                         the following section elucidates the pivotal role of these methodologies in enhancing  
                         UAV capabilities.  
  
                         Contributions  
                              Through a discussion of recent advancements and future directions, the survey aims  
                         to provide valuable insights for researchers and practitioners in the field.  
                         1.   Comprehensive Review: This survey offers a thorough examination of advanced  
                              nonlinear control strategies tailored for UAVs, emphasizing the integration of sensors  
                              and hybrid techniques.  
                         2.   Highlighting Significance: The survey underscores the importance of nonlinear con-  
                              trol methodologies in addressing the complexities inherent in UAV systems, shedding  
                              light on their efficacy in improving UAV performance and stability.  
                         3.   Role of Sensor Integration: The survey elucidates the pivotal role of sensor integration  
                              in enhancing UAV capabilities, providing insights into how sensor-driven approaches  
                              contribute to real-time data acquisition and informed decision-making.  
                         4.   Future Directions: Finally, by discussing recent advancements and outlining future  
                              challenges in the field, the survey aims to guide future research efforts towards the  
                              development of more efficient and reliable UAV control systems, thereby facilitating  
                              progress in various UAV applications.  
                              The outline of this survey is elaborated through the block diagram in Figure 2. Before  
                         moving on to the background section of this review article on UAVs, it is essential to set  
                         the stage by providing an overview of the burgeoning field of UAVs research in order to  
                         highlight the pivotal role of research. Table 1 offers a succinct summary of related surveys,  
                         providing brief outlines.  
  
  
  
  
                         Figure 2. Survey article organization.  
Sensors 2024, 24, 3286                                                                                                                 4 of 51  
  
  
  
                                      Table 1. Overview of recent surveys on UAV communication and control strategies.  
  
  Authors, Year          Reference    Brief Description  
                                      Review providing an exhaustive perspective on the use of UAVs and discussing practicalities  
  Fotouhi, et al.        [13], 2020   including the hurdles of integration, creation of protocols, establishment of standards, and issues  
                                      pertaining to security.  
                                      Analysis delving into progress on UAVs encompassing essential principles, application contexts,  
   Wang, et al.          [14], 2022   Air-to-Ground (A2G) communication channels, and the functions of UAVs. It includes an evaluation  
                                      of secrecy performance and improvement strategies for both stationary and mobile UAV systems.  
                                      Survey offering a comprehensive examination of studies regarding the utilization and path  
                                      planning of Unmanned Aerial Vehicles (UAVs) to improve the capacity and management of UAV  
        Han              [15], 2023  
                                      wireless networks. It also underscores the hurdles faced in this field and suggests potential  
                                      directions for future research.  
                                      Review centering on cutting-edge network technologies for UAVs and their deployment in  
  Sharma, et al.         [16], 2021   upcoming cellular networks, exploring a range of nascent communication technologies for UAVs  
                                      and evaluating their benefits, prospective uses, technical hurdles, and prospective developments.  
                                      Examination providing an in-depth analysis of UAV communication protocols, network architectures,  
  Hentati, et al.        [17], 2021   frameworks, and practical applications, while emphasizing significant technical obstacles and  
                                      identifying critical areas of research that demand further exploration and advancement.  
                                      Review presenting a comprehensive summary of research pertaining to UAV communications and  
                                      the integration of technologies. It explores the domain of mmWave beamforming in UAV  
    Xiao, et al.         [18], 2022  
                                      communications, addressing the technical potential and difficulties, and delves into the pertinent  
                                      aspects of mmWave antenna design and channel modeling.  
                                      A study demonstrating the efficacy of sub-6GHz massive MIMO technology in handling cell  
   Geraci, et al.        [19], 2023   selection and interference, evaluating the coverage of mmWave frequencies in various  
                                      environments, and scrutinizing the intricacies of initial 2D communication for airborne devices.  
                                      Review investigating how edge artificial intelligence influences crucial technical aspects and  
                                      applications of UAVs, spanning domains such as power management, formation control,  
 McEnroe, et al.         [20], 2023  
                                      autonomous navigation, and computer vision while addressing concerns related to privacy,  
                                      security, and communication.  
                                      Review identifying appropriate management strategies for UAV characteristics and spectrum  
                                      requirements, taking into account their coexistence with current wireless technologies within the  
    Jasim, et al.        [21], 2022  
                                      spectrum. Additionally, it details the guidelines and directives of policymakers and regulators and  
                                      investigates various operational frequency bands and radio interfaces.  
                                      Review scrutinizing the evolution of regulatory policies and key technologies pertinent to the safe  
     Xu, et al.          [22], 2022  
                                      and effective functioning of small civilian UAVs operating at low altitudes in urban settings.  
                                      Review focusing on integrating privacy and security measures in blockchain-supported UAV  
   Hafeez, et al.        [23], 2023   communications, underscoring the need for basic analysis and decentralized data storage solutions  
                                      while laying out crucial prerequisites in the formulation of privacy and security frameworks.  
                                      An assessment offering an extensive examination of various scenarios and pivotal technologies  
                                      relevant to UAV-assisted data gathering in the Internet of Things (IoT) context. It outlines system  
    Wei, et al.          [24], 2023  
                                      architectures, encompassing both the network infrastructure and mathematical modeling, and  
                                      performs an in-depth evaluation of the essential technologies involved.  
                                      Review investigating the role of UAVs in maritime communications and the integration of  
 Nomikos, et al.         [13], 2023   conventional approaches with machine learning techniques to improve performance in aspects  
                                      such as the physical layer, resource allocation, and cloud/edge computing.  
                                      Review presenting an exhaustive overview of UAV caching within 6G networks, covering the  
   Duong, et al.         [25], 2023   progression of caching models from ground-based to aerial systems. It introduces a standard UAV  
                                      caching system and delves into the latest developments and performance indicators in this field.  
                                      A survey reviewing advanced nonlinear control strategies for UAVs, emphasizing the necessity of  
                                      sensor-based adaptive controls and artificial intelligence. It explores innovative control strategies  
                                      such as sliding surface and sensor-driven techniques, highlighting their effectiveness in enhancing  
   This Survey  
                                      UAV performance and stability. This review underscores the complexities of UAV control, the  
                                      critical role of sensors, and the benefits of nonlinear methods while discussing recent advancements  
                                      and future challenges in this rapidly evolving field.  
Sensors 2024, 24, 3286                                                                                              5 of 51  
  
  
  
                         2. Background on UAV Systems  
                               Twin-rotor MIMO (Multiple Input, Multiple Output) systems represent a class of  
                         UAVs designed to simulate the flight dynamics of helicopters. The TRMS typically consists  
                         of two rotors mounted on a fixed beam, with one located at the front (main rotor) and one  
                         at the back (tail rotor). This configuration allows for the separate control of vertical lift and  
                         horizontal movement, making it possible to study complex control system designs within a  
                         MIMO framework. The TRMS is particularly useful in research and education because it  
                         provides a practical example of a system with strong cross-coupling effects, nonlinearities,  
                         and unstable dynamics, similar to those experienced in larger-scale helicopters [26,27]. The  
                         twin-rotor setup enables the study of pitch, roll, and yaw movements and the effects of  
                         MIMO controls on these motions. By manipulating various inputs, such as rotor speed or  
                         blade pitch, the TRMS can hover, move laterally, or rotate about its axis [28,29].  
                               A quadrotor, also known as a quadcopter, is a type of UAV that is lifted and propelled  
                         by four rotors. The configuration of these rotors, typically arranged in a square pattern,  
                         allows for a high degree of stability and maneuverability. Each rotor pair rotates in oppo-  
                         site directions, which counters the reactive torque and provides control over pitch, roll,  
                         and yaw. This design simplifies the mechanics required for flight control compared to  
                         traditional helicopters, which rely on complex rotor mechanisms. Due to their relative  
                         ease of construction and control, quadrotors have become a popular platform for hobbyists  
                         and researchers alike. Additionally, their stability and agility make them ideal for indoor  
                         operations or in environments where precise movements are required [30,31].  
                               Both TRMS and quadrotor designs contribute significantly to UAV technology, each  
                         offering unique advantages for control systems research, practical applications, and the  
                         advancement of aerial robotics. While the TRMS design provides insight into the complexi-  
                         ties of helicopter flight, the quadrotor design offers a more accessible platform for a broad  
                         range of UAV applications [30]. This paper traces the progression of caching models from  
                         aerial applications, introducing a standard UAV and examining recent developments and  
                         performance indicators in this area.  
  
                         3. Nonlinear MIMO System Dynamics in UAVs  
                               An unmanned vehicle is a prototype with a structure almost near to a helicopter, with  
                         a limited degree of freedom. Modifications of such systems are required due to their wide  
                         range of applications in real life. The UAV has two significant parts: the main rotor (vertical  
                         plane) and the tail rotor (horizontal plane) [12,32]. The main rotor has a higher diameter  
                         and controls the movement of the beam on a vertical axis the called pitch angle, while the  
                         tail rotor has a lower diameter and covers the movement of the beam on a horizontal axis,  
                         called the yaw angle. The speed of the rotors manages the equilibrium of the system. Each  
                         rotor of the UAV is connected with a separate DC supply motor, as shown in Figure 3.  
                               The stability is disrupted by the cross-coupling torque produced by the rotational  
                         torque in the UAV rotors. This coupling effect is seen as the disruption that the decoupling  
                         procedure corrects. Understanding all of the variable parameters and necessary UAV  
                         outputs is necessary before we can grasp mathematical modeling. The UAV is a lab tool  
                         that helps in understanding helicopter flight control [6,33]. The considered system has  
                         two rotors, as shown in Figure 4; their design is very important, as different forces affect the  
                         movement of the propellers. These include frictional force, disturbance torque, rotational  
                         force, propulsive force, and centrifugal force. Motors are used to deliver control input volt-  
                         age in order to counteract the impacts of these forces while recognizing and comprehending  
                         the mathematical presumptions that are used to interpret and reduce the mathematical  
                         model. Figure 5 details the rotor measurements along with their thrust directions.  
Sensors 2024, 24, 3286                                                                                       6 of 51  
  
  
  
  
                         Figure 3. Twin-rotor aerodynamic system [6].  
  
  
  
  
                         Figure 4. Mathematical model-based block diagram of UAV with coupling effect.  
  
  
  
  
                         Figure 5. Rotor dimensions with torque.  
  
                              All nonlinear squared factors in the mathematical equations are linearized using the  
                         linearization method. The system’s motions are fixed along the horizontal and azimuthal  
                         planes determined from the model [34,35]. The stream rotates in a manner that can be  
                         characterized as  
                                                                  d2 α v  
                                                                Jv 2 = Mv ,                                     (1)  
                                                                   dt  
Sensors 2024, 24, 3286                                                                                              7 of 51  
  
  
  
                         where Jv represents the inertial motion along the axis of the vertical plane and Mv represents  
                         the entire momentum of the forces applied along the vertical axis. One result, known as  
                         pitch angle, must be regulated by the al phav parameter (vertical axis). All of the factors  
                         acting on the momentum can be represented as follows:  
  
                                                 Mv = Mv 1 + Mv 2 + Mv 3 + Mv 4 + Mv 5 + Mv d .                        (2)  
  
                              The gravitational torque through the gravitational force is provided as  
  
                                                         Mv1 = −k1 cos( av ) − k2 sin( av ),                           (3)  
  
                         where the constants k1 and k2 are displayed and retain the mass mounted on the beam. An  
                         equation for the momentum force produced by the primary propeller is  
  
                                                                 Mv2 = lm Fv (wv ),                                    (4)  
  
                         where lm stands for the beam length, wv describes the main propeller’s spinning speed, and  
                         Fv (wv ) depicts the angular force of the main rotor. The following mathematical expression  
                         represents the moment force along the vertical plane:  
  
                                                         M(v3 ) = −k3 Ω2h sin( av )cos( av ),                          (5)  
  
                         where Ωh = dαh dt is the beam speed in the MIMO system’s vertical plane, αh is the yaw  
                                           
  
                         angle (the angle at which the beam rotates in the azimuth plane), and k3 is the constant  
                         parameter. The movement of the beam with respect to the horizontal plane determines the  
                         frictional momentum:  
                                                             M( v 4 ) = − k f v Ω v ,                               (6)  
  
                         where Ωv = dαv dt indicates the angular speed in the horizontal direction, and k f v displays  
                                           
  
                         the constant value. Momentum between the rotors as a result of input voltage (force) being  
                         applied along the horizontal plane.  
                                                                 M( v 5 ) = − k h v u h ,                              (7)  
  
                         where k hv is regarded as the constant and uh is the input for controlling the horizontal plane.  
                         The torque along the vertical plane causes a disturbance known as the disturbance torque  
                         Mvd . The rotational velocity of the main rotor is produced by the propeller force (propulsive  
                         force) along the vertical line (vertical plane) Fv (wv ). The following is the estimated velocity  
                         along the main rotor:  
  
                                         Fev = −7.13 × 10−19 wv5 − 3.79 × 10−16 wv4 + 2.41 × 10−11 w3v  
                                                           +1.87 × 10−8 w2v + 2.89 × 10−5 wv − 0.0124.  
  
                               The vertical axis can be used to determine the total force (or torque) along the horizon-  
                         tal axis. A force with a different spectrum is generated by the tail rotor’s total torque along  
                         the horizontal axis (intensity of force). A mathematical expression such as the one below  
                         can be used to determine the torque (rotational impact of force) along the horizontal axis:  
  
                                                              Jh (d2 ah )/(dt2 ) = Mh                                  (8)  
  
                         where Mh is the total momentum (force) along the horizontal axis, Jh is the total inertial  
                         force along the vertical plane, Jh = k4 cos2(αv) + k5 , and the coefficients k4 and k5 are  
                         mass-based constants of the beam. The following mathematical expression can be used to  
                         describe the entire force (momentum) along the horizontal axis:  
  
                                                        Mh = Mh1 + Mh2 + Mh3 + Mh d .                                  (9)  
Sensors 2024, 24, 3286                                                                                                      8 of 51  
  
  
  
                                    The propelling momentum (power) along the rotor of the tail can be used to represent  
                               each separate term:  
                                                              Mh1 = lt Fh (wh )cos( av )                             (10)  
                               where lt is the beam length, wh is the rotational velocity, and Fh (wh ) is the rotor’s propelling  
                               motion (force) expressed as the angular velocity. Based on the rotational speed of the beam,  
                               the frictional momentum is provided as follows:  
  
                                                                           Mh2 = − k f h Ω h                                  (11)  
  
                               where k f h is a constant. The cross-sectional velocity caused by the action of the control  
                               input along the horizontal axis is  
                                                                       Mh3 = k v−h cos(αv )uv .                               (12)  
  
                                    The torque along the horizontal plane, also known as the disturbance torque caused  
                               by the tail rotor, is denoted by the symbol Mhd , while the propeller force (propulsive force)  
                               along the horizontal axis (azimuthal plane), denoted by the symbol Fv wv , generates the rate  
                               of rotation on the rotor. The estimated velocity along the tail rotor is provided as follows:  
  
                                                 Feh = −2.56 × 10−20 w5h − 4.10 × 10−17 w4h + 3.17 × 10−12 w3h  
                                                                      +7.34 × 10−9 w2h + 2.13 × 10−5 wh − 9.14,  
  
                                     The equation for the primary propeller (rotor) along its vertical plane is  
  
                                                                  Iv (dwv )/dt = uv − Hv−1 (wv ).                             (13)  
  
                                    Here, the main rotor’s inertial momentum along the vertical axis is denoted by Iv , while  
                               the static motion of the main rotor is provided by wv = Hv uh . Figure 6a shows how the  
                               primary rotor thrust is generated; the figure displays the velocity modeling outcomes based  
                               on experimental verification in Figure 6b. The following is the seventh-order expression for  
                               the primary rotor’s vertical plane velocity:  
  
fv = −6.17 × 103 u7v − 1.30 × 102 u6v + 1.37 × 104 u5v + 1.50 × 102 u4v − 1.10 × 104 u3v − 3.76 × 101 u2v + 7.33 × 103 uv − 5.36.  
w  
  
  
  
  
                               Figure 6. (a) Main rotor thrust and (b) tail rotor thrust.  
  
                                    The following mathematical equation can be used to describe the motion (speed) of  
                               the tail rotor along the horizontal axis:  
  
                                                                  Ih (dwh )/dt = uh − Hh−1 (wh ).                             (14)  
  
                                  Here, Ih is the tail rotor’s inertial motion along the horizontal axis and wh = Hh uh  
                               demonstrates the static velocity (or speed) of the primary rotor. The results of the velocity  
                               modeling and the tail rotor thrust are supported by experimental verification. Figure 7a  
Sensors 2024, 24, 3286                                                                                                    9 of 51  
  
  
  
                         represents the velocity generated by main rotor, while Figure 7b represents the velocity of  
                         the tail rotor. The fifth-order expression for the rotary velocity is as follows:  
  
                                                fh = −6.17 × 103 u5h − 1.30 × 102 u4h + 1.37 × 104 u3h  
                                                w  
                                                                +1.50 × 102 u2h − 1.10 × 104 uh − 37.6.  
  
  
  
  
                         Figure 7. (a) Main rotor velocity and (b) tail rotor velocity.  
  
                              By rearranging Equations (13) and (14) together, we obtain  
  
                                                                        dαv  
                                                                            = Ωv ,                                          (15)  
                                                                         dt  
                                                                  dαh  
                                                                      = Ωh .                                    (16)  
                                                                   dt  
                              The control voltages at the input uh (horizontal plane or yaw angle) and uv are used  
                         in the modeling process to describe the state space description of the UAV (sixth-order  
                         nonlinear system). The yaw (azimuth) angle αh and pitch (vertical) angle αv are the output  
                         angles. In order to fully comprehend its dynamic response, the system must be categorized  
                         as a multivariable system with extremely nonlinear behavior. Two channels in the UAV  
                         model can never be regarded as separate channels. This cross-coupled feature is known as  
                         the coupling effect. Decoupling must be used to overcome the coupling impact and create  
                         an independent two-channel system. The values of the model’s system coefficients are  
                         provided in Table 2.  
                              The decoupling technique divides the system into two distinct planes, namely, the  
                         Vertical Plane System (VPS) and Horizontal Plane System (HPS), by fixing one weight  
                         (motion) for both blades [36].  
  
                         3.1. Unstructured Modeling  
                                  Mathematical complexity and unknown conditions are always present in the various  
                         structure-type systems during modeling. Mathematical expressions and notations are used  
                         to indicate the description of uncertain parameters [35]. Supposing that Jh is the inertial  
                         momentum along the horizontal axis and that k Fh , k Fv are the coefficients of the generated  
                         thrust of the rotors, the rotors then have some velocity gains k Hh , k Hv , the coefficients,  
                         k f h , k f v , and k vh , and k hv are the corresponding cross-momentum and frictional momentum  
                         coefficients, and RV is the returned force between the rotors (coupling effect). All ten of  
                         these modeled factors are dependent on the pitch angle and yaw angle, which are the two  
                         primary outputs.  
                                  Furthermore, we assume that the inertial momentum Jh , with factors k Fh , k Fv , k Hh , k Hv ,  
                         has an error estimation of up to 10%. The remaining coefficients have up to 5% error in  
                         their calculation. The UAV represents the system behavior as the controlled plant through  
                         algebraic expressions:                                        
                                                                                  Gv  
                                                                            G=                                             (17)  
                                                                                  Gh  
Sensors 2024, 24, 3286                                                                                                                   10 of 51  
  
  
  
                         where                                                                                           
                                                         Md                  αh                  uh                    Md h  
                                              y=G                 ,y =                ,u =                , Md =                            (18)  
                                                         u                   αv                  uv                    Md v  
  
  
                         Table 2. Parameters of TRMS (UAV).  
  
                                 Variable Notation                        Description                       Units and Values of Parameters  
                                        I1                           Main rotor inertia                               6.8 × 10−2 kgm2  
                                        I2                            Tail rotor inertia                               2 × 10−2 kgm2  
                                        a1                                 constant                                         0.0135  
                                        b1                                 constant                                         0.0924  
                                        a2                                 constant                                          0.02  
                                        b2                                 constant                                           0.9  
                                       Mg                         Gravitational Momentum                                   0.32 Nm  
                                       B1θ                          Frictional parameter                           6 × 10−3 Nms2 /rad2  
                                       B2θ                          Frictional parameter                           1 × 10−3 Nms2 /rad2  
                                       B1φ                          Frictional parameter                            1 × 10−1 Nm.s/rad  
                                       B2φ                          Frictional parameter                            1 × 10−2 Nms2 /rad  
                                       k gy                        Gyroscopic Parameter                                   0.05 rad/s  
                                        k1                          Gain of Main Motor                                        1.1  
                                        k2                           Gain of Tail Motor                                       0.8  
                                       T11                         Denominator of motor                                       1.1  
                                       T10                          Numerator of motor                                         1  
                                       T21                         Denominator of motor                                        1  
                                       T20                          Numerator of motor                                         1  
                                        kc                        Coupling reaction for gain                                   2  
  
  
                               Figure 8 shows the input–output links for the UAV’s system schematic model, fol-  
                         lowed by a discussion of the unclear model’s fundamental mathematical formulation. The  
                         illustration can be described as follows:  
                                                                             
                                                               G = Gd Gu  
  
                         while  
                                                                                 
                                                                   Gd = Gdh Gdv  
                                                                                   
                                                                     Gu = Guh Guv  
  
                         such that  
                                                                         y = Gd Md + Gu u.                                                  (19)  
                             The above expression states that Gd depicts the plant disturbance as a matrix and  
                         Gu displays the matrix of the control signal. In the event of perturbations (both internal  
                         and exterior disturbances), the uncertain plant must meet the following fundamental  
                         requirements:  
                                                                         T  
                                                u = Kr K y        r −yc        = Kr r − K y y c                 (20)  
                         where Ky represents the feedback matrix function and Kr is the transfer function matrix of  
                         the pre-filter.  
  
                         3.2. Inherent Characteristics: Nonlinearity and Coupling Effects  
                               Nonlinearity is a fundamental characteristic of many UAV systems, including TTRMS  
                         and quadrotors. This implies that the relationship between input variables (such as the  
                         rotor speed or control surface angles) and output responses (such as altitude, position, or  
                         attitude) is not proportional or does not follow a straight line when graphed. In UAVs,  
                         nonlinearity can arise from aerodynamic effects, changes in the air density at different  
                         altitudes, battery voltage drops, or the complex interplay between thrust, drag, lift, and  
                         weight. For instance, increasing the thrust on a rotor does not result in a linear increase  
                         in altitude due to factors such as aerodynamic drag. Similarly, the efficiency of the rotor  
Sensors 2024, 24, 3286                                                                                          11 of 51  
  
  
  
                         blades changes with speed and angle of attack, further contributing to nonlinearity. These  
                         nonlinear dynamics can make control systems more complex, as linear control strategies  
                         may not suffice to stabilize or maneuver the UAV effectively across its entire operating  
                         envelope [14].  
  
  
  
  
                         Figure 8. Block diagram of decoupled UAV input/output.  
  
                              Coupling effects refer to the interdependence of control channels within a UAV. In an  
                         ideal linear system, adjusting one control input would affect only the corresponding output.  
                         However, in UAVs, a change in one control input can affect multiple outputs due to coupling  
                         effects. For TRMS, the pitch control can influence both the altitude and the forward motion  
                         due to the aerodynamic interactions between the rotors and the fuselage. In quadrotors,  
                         the situation is similar; controlling roll or pitch also affects horizontal displacement and  
                         yaw due to the interrelated rotor speeds needed to produce these movements. Coupling is  
                         particularly pronounced in UAVs due to the close proximity of the rotors and the need for  
                         differential thrust to control movement and orientation. For example, increasing the speed  
                         of one rotor on a quadrotor to initiate a roll movement will inadvertently affect the lift and  
                         could cause a change in altitude if not compensated for by the other rotors [15].  
                              Both nonlinearity and coupling effects pose significant challenges in UAV control sys-  
                         tem design. Controllers must be robust and adaptive, often relying on complex algorithms  
                         such as those found in nonlinear control theory and MIMO system design to ensure the  
                         stability and responsiveness of the UAV in the face of these inherent characteristics [16].  
  
                         3.3. Role of Sensors in UAVs  
                              Sensors play a crucial role in the operation and functionality of TRMS and quadrotor-  
                         based UAVs. These sensors are fundamental in providing the necessary data for navigation,  
                         stabilization, and execution of specific tasks. While the types and roles of these sensors can  
                         vary depending on the UAV’s application, some of the most common uses include:  
                         •    GPS (Global Positioning System): Provides accurate location data, enabling UAVs to  
                              navigate to specific coordinates and maintain a stable position [16].  
                         •    Gyroscopes: Essential for detecting and measuring rotational motion and orientation,  
                              gyroscopes help in maintaining the UAV’s balance and orientation [17].  
                         •    Accelerometers: Measuring linear acceleration, in UAVs they are crucial for detecting  
                              changes in the speed and tilt for orientation control [17,18].  
                         •    Magnetometers: Act as digital compasses, aiding in orientation relative to the Earth’s  
                              magnetic field, which is particularly useful in environments where GPS signals are  
                              weak or unavailable [37].  
                         Environmental Interaction and Data Collection  
                         •    Lidar (Light Detection and Ranging): Used for high-precision mapping and terrain  
                              analysis by measuring distances with laser light [38].  
Sensors 2024, 24, 3286                                                                                           12 of 51  
  
  
  
                         •    Infrared Sensors: Employed in various applications, from detecting heat signatures for  
                              search and rescue operations to assessing crop health in precision agriculture [38,39].  
                         •    Optical Cameras: Provide visual data, critical for tasks such as aerial photography,  
                              surveillance, and visual inspection of infrastructure [40].  
                         •    Magnetometers: Act as digital compasses, aiding in orientation relative to the Earth’s  
                              magnetic field, which is particularly useful in environments where GPS signals are  
                              weak or unavailable [40,41].  
                         •    Multispectral Cameras: Capture data across different wavelengths, useful in environ-  
                              mental monitoring, agriculture (for assessing plant health), and resource mapping [38].  
                         Obstacle Detection and Collision Avoidance  
                         •    Ultrasonic Sensors: Measure the distance to nearby objects, aiding in collision avoid-  
                              ance, especially in tight or cluttered spaces [33].  
                         •    Infrared Sensors: Employed in various applications, from detecting heat signatures  
                              for search and rescue operations to assessing crop health in precision agriculture [42].  
                         •    Radar: Used for detecting and avoiding obstacles, particularly effective in poor  
                              weather conditions where optical sensors might be less effective [38].  
                         Specialized Applications  
                         •    Chemical Sensors: Detect specific chemicals or environmental pollutants, useful in  
                              environmental monitoring [43].  
                         •    Thermal Cameras: Useful for search and rescue operations at night or for detecting  
                              energy inefficiencies in buildings [37].  
                               The integration of these sensors into UAV systems allows for a wide range of capabili-  
                         ties, from basic flight stabilization and navigation to complex tasks such as 3D mapping,  
                         agricultural monitoring, infrastructure inspection, and search and rescue operations. The  
                         choice of sensors is dictated by the specific requirements of each application, balancing  
                         factors such as accuracy, range, size, weight, and power consumption against the UAV’s  
                         intended use. Advanced UAVs often combine multiple sensors, using sensor fusion tech-  
                         niques to enhance reliability and the quality of the data collected [37,43].  
  
                         4. Challenges in UAV Control  
                         4.1. Discussion of Nonlinearities and Coupling Effects in UAVs  
                             The operation of UAVs such as TRMS and quadrotors is inherently complex due to  
                         nonlinearities and coupling effects. These characteristics present significant challenges in  
                         UAV control system design and operation [19].  
                         •    Complex Dynamics: UAVs exhibit nonlinear dynamics due to factors such as aerody-  
                              namic forces, changes in air density at different altitudes, and the nonlinear responses  
                              of the motors and rotors. For instance, the relationship between the throttle input  
                              and the resultant lift is not linear, especially when the UAV operates in different flight  
                              regimes (e.g., hover, ascent, descent) [20].  
                         •    Control Challenges: Linear control strategies may not be effective across the entire  
                              operational range of a UAV. Nonlinear control systems must be designed to adapt to  
                              these varying dynamics, ensuring stable and responsive flight under a wide range of  
                              conditions [44].  
                         •    Modeling Difficulties: Accurately modeling the nonlinear behavior of UAVs for simula-  
                              tion and control design is complex and computationally intensive, requiring advanced  
                              algorithms and significant processing power [44,45].  
                         Coupling Effects in UAVs  
                         •    Interdependent Controls: In UAVs, especially those with multiple rotors such as  
                              quadrotors, changes in one control input often affect multiple axes of motion. For  
                              example, adjusting rotors to initiate a turn may inadvertently affect altitude and pitch,  
                              requiring compensatory control inputs [46].  
Sensors 2024, 24, 3286                                                                                          13 of 51  
  
  
  
                         •    Design of Control Systems: The control system must account for these coupling  
                              effects in order to achieve precise maneuverability. This often leads to more complex  
                              control algorithms that can manage the interdependencies of the control inputs and  
                              outputs [46,47].  
                         •    Flight Stability and Precision: Maintaining stability and precision in flight maneu-  
                              vers is challenging due to coupling effects. Controllers must continuously adjust  
                              for the unintended consequences of control inputs on various aspects of the UAV’s  
                              motion [47].  
                         •    Real-Time Adaptation: Effective UAV control requires real-time adaptation to the  
                              coupling effects, especially in dynamic environments or during complex tasks such as  
                              payload delivery, precise inspections, or navigating through cluttered spaces [8].  
                         Addressing the Challenges  
                         •    Advanced Control Strategies: Developing advanced control strategies such as nonlin-  
                              ear control, adaptive control, and robust control is crucial. These strategies can handle  
                              the complexities and variabilities in UAV dynamics [48].  
                         •    Sensor Fusion and Feedback: Utilizing sensor fusion techniques can enhance the  
                              UAV’s understanding of its state and environment, leading to better control. Feedback  
                              mechanisms are crucial for adjusting control inputs in real-time based on the UAV’s  
                              response [49].  
                         •    Machine Learning and AI: Implementing machine learning and artificial intelligence  
                              can help to predict and adapt to nonlinearities and coupling effects, especially in  
                              unpredictable environments [9].  
                         •    Simulation and Testing: Rigorous simulation and real-world testing are vital to under-  
                              standing and mitigating the effects of nonlinearities and couplings in UAVs. This aids  
                              in refining control algorithms and ensuring reliable UAV performance [50].  
                              Addressing the challenges posed by nonlinearities and coupling effects is fundamental  
                         for the advancement of UAV technology, particularly as their applications become more  
                         diverse and their operational environments grow more complex as depicted in Figure 9.  
  
  
  
  
                         Figure 9. Challenges in UAV control.  
  
                         Evolution of UAV Control Strategies  
                               Over the past few decades, the field of UAV control has witnessed significant advance-  
                         ments, evolving from simple manual control systems to sophisticated autonomous control  
                         strategies capable of handling complex tasks. Early UAV control strategies primarily relied  
                         on manual piloting or basic autopilot systems, providing limited autonomy and functional-  
                         ity. With the advent of advanced computational methods and sensor technologies, there has  
                         been a paradigm shift towards more intelligent and adaptive control strategies, including  
                         PID control, Linear Quadratic Regulators (LQR), and nonlinear control techniques such as  
                         backstepping and feedback linearization [10,51].  
Sensors 2024, 24, 3286                                                                                           14 of 51  
  
  
  
                               In recent years, the integration of artificial intelligence, machine learning, and pre-  
                         dictive control algorithms such as MPC has further revolutionized UAV control, enabling  
                         enhanced performance, robustness, and autonomy. Despite these advancements, chal-  
                         lenges remain in achieving optimal control of UAVs, particularly in handling nonlin-  
                         ear dynamics, coupling effects, and uncertainties associated with complex operational  
                         environments [19,52].  
                               In this context, our work makes several unique contributions to the existing litera-  
                         ture. Unlike previous studies that focused on individual control strategies or specific UAV  
                         platforms, our review provides a comprehensive analysis of various nonlinear control  
                         strategies, with a particular emphasis on their applicability to UAVs such as TRMS and  
                         quadrotors. We delve into the intricacies of sensor integration, adaptive controls, and artifi-  
                         cial intelligence-driven approaches, highlighting their efficacy in addressing the challenges  
                         associated with UAV control [53].  
                               Furthermore, our review offers insights into the latest developments, research direc-  
                         tions, and future challenges in this rapidly evolving field, serving as a valuable resource  
                         for researchers, engineers, and practitioners working on UAV control systems. By bridg-  
                         ing the gap between theory and practice, our work aims to advance the state-of-the-art  
                         in UAV control, paving the way for the development of more intelligent, robust, and  
                         autonomous UAV systems capable of operating in increasingly complex and dynamic  
                         environments [24,26,28].  
  
                         4.2. Limitations of Traditional Linear Control Techniques  
                              Traditional linear control techniques, while effective in many standard control appli-  
                         cations, face inherent limitations when applied to the complex dynamics of UAVs [54].  
                         These limitations stem from the fact that UAVs often exhibit nonlinear behaviors and  
                         interactions that cannot be adequately addressed by linear control methods. Some of the  
                         key limitations include:  
                         Inadequacy in Handling Nonlinear Dynamics  
                         •    Linear Approximation: Linear control techniques are based on linear approximations  
                              of system dynamics. However, UAVs often exhibit strongly nonlinear behaviors due  
                              to aerodynamic forces, changes in air density at different altitudes, and the nonlinear  
                              response of their propulsion systems [55].  
                         •    Limited Operational Range: Linear controllers are typically designed around a specific  
                              operating point, such as hovering. They tend to be less effective when the UAV  
                              operates outside this narrow range, such as during rapid maneuvers or in response to  
                              strong external disturbances, and may even fail [56].  
                         Oversimplification of Real-World Scenarios  
                         •    Ignoring External Factors: Linear controllers often do not account for varying external  
                              conditions such as wind gusts, temperature changes, or variable payloads, which can  
                              significantly affect UAV performance [57].  
                         •    Simplified Models: These controllers rely on simplified models that may not capture  
                              the full complexity of UAV dynamics, leading to suboptimal or unstable flight in  
                              real-world scenarios [58].  
                         Inability to Manage Coupling Effects  
                         •    Independent Control Assumption: Linear control techniques often assume that each  
                              control input affects only one output. However, in UAVs, especially multi-rotor sys-  
                              tems such as quadrotors, there is significant coupling between controls, e.g., changing  
                              rotor speed to control roll might inadvertently affect pitch and yaw [59].  
                         •    Challenges in MIMO Systems: UAVs are often MIMO (Multiple Input, Multiple  
                              Output) systems where the interaction between multiple inputs and outputs needs to  
                              be managed simultaneously, a task for which traditional linear control methods are  
                              not well-suited [60].  
Sensors 2024, 24, 3286                                                                                       15 of 51  
  
  
  
                              Lack of Adaptability and Robustness  
                         •    Fixed Parameters: Traditional linear controllers have fixed parameters and do not  
                              adapt to changes in the UAV’s dynamics or the environment. This lack of adaptability  
                              can lead to poor performance in changing conditions [61].  
                         •    Robustness Issues: They may not be robust against uncertainties or unmodeled dy-  
                              namics, which are common in UAV operations, especially in complex or unstructured  
                              environments [62].  
                         Increased Complexity in Design and Tuning  
                         •    Complex Design for Multivariable Systems: Designing linear controllers for systems  
                              with multiple interdependent variables, such as UAVs, can be complex and time-  
                              consuming [29].  
                         •    Manual Tuning Limitations: These controllers often require manual tuning, which can  
                              be a laborious process and might not yield the best possible performance [63].  
                             While traditional linear control techniques have been foundational in control theory  
                         and have applications in many areas, their limitations become apparent in the context of  
                         UAVs. As explained in Figure 10, the complexity, nonlinearity, and dynamic nature of  
                         UAVs call for more advanced control strategies that can adapt to changing conditions,  
                         handle nonlinear dynamics, and manage the coupling effects inherent in these systems.  
  
  
  
  
                         Figure 10. Limitations of traditional linear control techniques.  
  
                         4.3. Impact of Sensor Data Complexity on UAV Control  
                             The integration and utilization of sensor data plays a critical role in the control and  
                         operation of UAVs [64]. However, the complexity of sensor data can significantly impact  
                         UAV control systems in various ways:  
                         Data Volume and Processing Challenges  
                         •    High Data Volume: UAVs often utilize a multitude of sensors such as cameras, LiDAR,  
                              GPS, gyroscopes, and accelerometers, which generate a vast amount of data. Process-  
                              ing this high volume of data in real time can be challenging, requiring robust and  
                              efficient data processing algorithms [29].  
                         •    Computational Load: The need to swiftly process and analyze complex sensor data  
                              places a considerable computational load on the UAV’s onboard systems, which could  
                              impact its operational efficiency and responsiveness [65].  
                         Sensor Fusion and Integration  
                         •    Complexity of Sensor Fusion: Integrating data from multiple sensors to form a cohe-  
                              sive understanding of the UAV’s environment (sensor fusion) is complex. It requires  
                              sophisticated algorithms to accurately combine and interpret disparate data sources,  
                              which can be challenging, especially in dynamically changing environments [66].  
                         •    Inconsistencies and Conflicting Data: Different sensors may provide conflicting in-  
                              formation due to varied accuracy, resolution, or response times. Resolving these  
                              inconsistencies is crucial for accurate flight control and decision-making [67].  
Sensors 2024, 24, 3286                                                                                         16 of 51  
  
  
  
                              Accuracy and Reliability Concerns  
                         •    Sensor Accuracy: The precision and accuracy of sensors directly affect the control  
                              and stability of UAVs. Inaccuracies in sensor readings can lead to incorrect control  
                              commands and potentially unstable flight [68].  
                         •    Reliability under Diverse Conditions: Sensors must be reliable under a wide range of  
                              operational conditions. For example, visual sensors might be less effective in low-light  
                              or foggy conditions, impacting the UAV’s ability to navigate and avoid obstacles [69].  
                         Real-Time Decision Making  
                         •    Delayed Response: The time taken to process complex sensor data can lead to de-  
                              lays in decision-making, which is critical for UAVs that need to respond quickly to  
                              environmental changes or obstacles [70].  
                         •    Autonomous Operations: For autonomous UAVs, the ability to make real-time deci-  
                              sions based on sensor data is crucial. Complex data can complicate the algorithms  
                              needed for autonomous navigation and task execution [71].  
                         Energy Consumption  
                         •    Delayed Responses: The time taken to process complex sensor data can lead to  
                              delays in decision-making, which is critical for UAVs that need to respond quickly to  
                              environmental changes or obstacles [72].  
                         •    Autonomous Operations: For autonomous UAVs, the ability to make real-time deci-  
                              sions based on sensor data is crucial. Complex data can complicate the algorithms  
                              needed for autonomous navigation and task execution [73].  
                         •    Increased Power Demand: Processing complex sensor data requires significant com-  
                              putational resources, which in turn increases energy consumption. This can reduce  
                              the UAV’s operational endurance and limit its range or mission duration [74].  
                         Calibration and Maintenance  
                         •    Calibration Complexity: Accurate sensor data depends on proper calibration. Complex  
                              sensor systems may require frequent and sophisticated calibration procedures, adding  
                              to the operational overhead [75].  
                         •    Maintenance Requirements: More complex sensor systems might have higher mainte-  
                              nance needs, impacting the UAV’s readiness and operational costs [76].  
                               While sensors are indispensable for modern UAV operations, their complexity intro-  
                         duces several challenges. As shown in Figure 11, effectively managing these challenges  
                         involves developing advanced computational algorithms for real-time processing, improv-  
                         ing sensor technologies for better accuracy and reliability, and optimizing UAV designs for  
                         efficient energy use. The ultimate goal is to ensure that UAVs can effectively interpret and  
                         respond to their environment, balancing the complexity of sensor data with the need for  
                         swift and accurate control.  
  
  
  
  
                         Figure 11. Sensor data complexity and UAV control.  
Sensors 2024, 24, 3286                                                                                            17 of 51  
  
  
  
                         5. Nonlinear Control Strategies: Sliding Surface-Based Control Strategies  
                              Nonlinear control strategies are essential for highly nonlinear and coupled UAV  
                         systems. Linear control techniques are not suitable for UAVs because of nonlinearities and  
                         coupling effects. Nonlinear control strategies are designed to handle these nonlinearities  
                         and coupling effects, making them more effective for UAVs [76,77].  
                              There are various nonlinear control strategies available for UAVs, including slid-  
                         ing mode control [76,77], fuzzy control [76], adaptive control [78], and neural network  
                         control [79,80]. Sliding mode control is one of the most popular nonlinear control tech-  
                         niques for UAVs. It provides robustness against parameter uncertainties and external  
                         disturbances. Fuzzy control is a nonlinear control technique that uses linguistic variables to  
                         control the system. It is robust against uncertainties and disturbances, and can handle the  
                         nonlinearities and coupling effects of UAVs. Adaptive control uses adaptive laws to adjust  
                         the control parameters based on the system’s dynamics, making it suitable for TRMS with  
                         varying operating conditions. Neural network control uses artificial neural networks to  
                         model the system and provide control.  
  
                         5.1. First-Order Sliding Mode Control (SMC)  
                               The foundation of SMC is VSS control theory, which operates under the tenet that in  
                         order to maintain the states of the system in sliding mode, the controller structure must  
                         continuously change in response to variations in the state variables. By using switching  
                         control with a high frequency, the SMC has a tendency to alter system dynamics [81–83].  
                         The VSS control theory is the foundation for the idea of sliding mode control, which  
                         operates on the idea that the controller design should change continually in response to  
                         changes in the state variables in order to maintain the system states in sliding mode. There  
                         are two distinct parts to the controller design [34,84–86]. The sliding surface is assessed in  
                         the first portion using the system’s order as a basis.  
                               The intended sliding surface is expressed as follows:  
                                                                             n  
                                                                      d  
                                                         s(t) =          +λ        ∫0t eξ (t) dt .                   (21)  
                                                                      dt  
  
                              Here, eξ (t) is the observed monitored output with the new state as the state variable  
                         and s(t) is the chosen manifold used as the sliding surface. It is vital to consider the  
                         control rule that forces the control factors to their reference value while choosing the sliding  
                         surface [87,88]. The mathematical formulation of the control law is  
  
                                                                  u = ueq + udis ,                                   (22)  
  
                         while  
                                                               udis = −ksign(s).                                     (23)  
                               Here, the constant k1 , the sign function sign(s), and the equivalent controller ueq  
                         provide a fragmented control input as a result of a limited switching over the sliding  
                         surface. The movement of numerous control structures causes the controlled system’s  
                         trajectory to slide along a variety of planes while adhering to the switching condition. It has  
                         been noted that switching functions s( x ), where x is either a scalar or vector, can be used  
                         during SMC to define the system structure [89–91]. A line on the phase plane represents  
                         the switching surface, denoted by s( x ) = 0.  
                               The instability of a nonlinear system can be examined using the Lyapunov function  
                         and the theory of ODEs [92,93]. The ODE class theory calculates the system’s Lyapunov  
                         function, which has to be negative definite, and uses the results to confirm the stability of  
                         the system. The nonlinear system’s asymptotic stability is guaranteed by this necessary  
                         condition. However, an appropriate methodology for building functions for ODEs is  
                         lacking [94,95]. In practice, the sliding motion is observed to exist in the vicinity of the  
                         sliding surface, much like the frequency-switching phenomenon in Figure 12. The system’s  
Sensors 2024, 24, 3286                                                                                            18 of 51  
  
  
  
                         nonlinear behavior will attempt to stray off the sliding surface, while the controller will  
                         push it to stay on course until it converges at the boundary layer or origin [96,97]. The  
                         power supply needs to be highly optimized because of the quick and abrupt changes in  
                         voltage patterns. Optimization methods can be used to achieve the required UAV output  
                         with steady settling and a predictable voltage pattern.  
  
  
  
  
                         Figure 12. Chattering phenomenon.  
  
                              Chattering is a well-known phenomenon in SMC that can cause high-frequency oscil-  
                         lations in the control signal. These oscillations can lead to actuator failures due to excessive  
                         wear and tear, and can result in degraded control performance [98–100].  
                              In one study [51], SMC was used to control a UAV for target tracking and hovering  
                         tasks. The results showed that SMC was able to track the target accurately and robustly even  
                         in the presence of disturbances and model uncertainties. SMC is a popular nonlinear control  
                         strategy for UAVs due to its robustness against disturbances and uncertainties. In SMC, a  
                         sliding surface is defined such that the dynamics of the system are constrained to remain on  
                         the surface. Several studies have used sliding mode control for UAVs. In one study [101],  
                         SMC was used for the trajectory tracking control of a UAV. The proposed controller was  
                         able to achieve accurate and robust tracking even in the presence of disturbances and  
                         modeling uncertainties. In another study [102], SMC was used for the attitude control of  
                         a UAV. The proposed controller was able to achieve robust performance in the presence  
                         of external disturbances and parameter uncertainties. UAV systems are highly nonlinear  
                         and coupled, making them challenging to control using traditional linear control methods.  
                         SMC has limitations around obtaining the required response, making adaptation law-based  
                         SMC variants better for complex MIMO models.  
  
                         Remarks  
                              SMC is known for being insensitive to external disturbances and nonlinearities.  
                         Observer-based SMC is a good option for reducing chattering while maintaining the  
                         invariance property of SMC; however, observer-based SMC is not suitable for dealing with  
                         parametric uncertainties or for maintaining tracking accuracy. Adaptive sliding mode  
                         control (ASMC) is more robust than either conventional SMC or observer-based SMC.  
                         Therefore, ASMC can be used instead of SMC to achieve adaption and remove chattering.  
Sensors 2024, 24, 3286                                                                                         19 of 51  
  
  
  
                         5.2. Backstepping (Recursive) Structure-Based Control Strategies  
                              Backstepping (recursive) structure-based control strategies are an important class of  
                         nonlinear control techniques that have gained significant attention in recent years. The  
                         importance of these strategies lies in their ability to handle complex nonlinear control  
                         problems with high precision and efficiency. Backstepping control strategies are important  
                         for a number of specific reasons:  
                         •    Nonlinear control: Backstepping control strategies are designed specifically for non-  
                              linear control problems, which are common in many engineering applications. These  
                              strategies provide a framework for designing controllers that can handle nonlinear  
                              dynamics and uncertainties while ensuring stability and convergence.  
                         •    Recursive structure: Backstepping control strategies have a recursive structure that  
                              allows for the systematic design of control laws. This structure provides a natural way  
                              to build up the control law step-by-step, starting from the highest-order states and  
                              working downwards. This recursive approach simplifies the control design process,  
                              making it easier to develop complex controllers.  
                         •    Lyapunov stability analysis: Backstepping control strategies are typically designed  
                              using Lyapunov stability analysis, which provides a rigorous mathematical frame-  
                              work for assessing the stability and convergence properties of the control system.  
                              This analysis ensures that the designed controller is stable and that the system state  
                              converges to the desired state in a finite time.  
                         •    Robustness: Backstepping control strategies are inherently robust against disturbances  
                              and uncertainties, as they are designed to handle nonlinear dynamics and uncertainties  
                              in a systematic way. This robustness makes them particularly useful in applications  
                              where the system model is uncertain or poorly known.  
                         •    Performance: Backstepping control strategies can achieve high control performance  
                              because they are designed to optimize a performance criterion based on the sys-  
                              tem dynamics and control objectives. This optimization ensures that the control  
                              law is designed to achieve the desired control performance while ensuring stability  
                              and robustness.  
                               Backstepping control is a type of nonlinear control technique that has gained attention  
                         for its effectiveness in controlling UAVs. The backstepping control approach is particularly  
                         suitable for UAVs because it can handle complex the nonlinear dynamics and uncertainties  
                         that are present in these vehicles. Backstepping control has been used in various UAV  
                         applications, including trajectory tracking, altitude control, and stabilization. One of  
                         the most significant advantages of backstepping control is that it can provide robust  
                         performance in the presence of disturbances and uncertainties. Additionally, it allows for  
                         the design of a control law that ensures the stability of the system while achieving the  
                         desired tracking performance.  
                               Several studies have reported the effectiveness of backstepping control for UAV appli-  
                         cations. For example, in one study [103], backstepping control was used for the trajectory  
                         tracking of a UAV, with the results showing improved tracking accuracy compared to  
                         other control methods. In another study [104,105], backstepping control was used for the  
                         altitude control of a fixed-wing UAV, and the results showed good performance even in the  
                         presence of wind disturbances.  
  
                         5.2.1. Mathematical Formulation of Backstepping Control  
                              Backstepping is a recursive control design technique that is particularly well-suited  
                         for nonlinear systems such as UAVs. For TRMS UAVs, we can consider the control of the  
                         pitch (θ) and yaw (ψ) angles using backstepping.  
                              The dynamic equations for the pitch and yaw angles of a TRMS UAV can be repre-  
                         sented as follows:  
                                                          Ixx θ̈ = τθ − ( Izz − Iyy )ψ̇ϕ̇                       (24)  
                                                           Iyy ψ̈ = τψ − ( Ixx − Izz )ϕ̇θ̇                        (25)  
Sensors 2024, 24, 3286                                                                                                      20 of 51  
  
  
  
                         where Ixx , Iyy , and Izz are the moments of inertia about the principal axes, τθ and τψ are the  
                         respective control torques for the pitch and yaw, and ϕ̇ is the roll rate.  
                              To design a backstepping controller, we introduce virtual control inputs and Lyapunov  
                         functions to recursively stabilize the system. The control law for the pitch and yaw angles  
                         can be formulated as follows:  
  
                                                          τθ = −k θ (θ − θref ) − k θ̇ (θ̇ − θ̇ref )                           (26)  
                                                         τψ = −k ψ (ψ − ψref ) − k ψ̇ (ψ̇ − ψ̇ref )                            (27)  
  
                         where k θ , k θ̇ , k ψ , and k ψ̇ are the control gains and θref , θ̇ref , ψref , and ψ̇ref are the desired  
                         pitch and yaw angles and their corresponding rates.  
                              The Lyapunov functions are chosen to ensure the stability of the closed-loop system;  
                         the recursive nature of backstepping allows for the design of controllers for higher-order  
                         systems by sequentially stabilizing subsystems. By applying this backstepping control law,  
                         the pitch and yaw angles of the TRMS UAV can be effectively controlled, ensuring stability  
                         and desired performance.  
  
                         5.2.2. Remarks  
                               Backstepping control has been successfully applied to various physical systems, in-  
                         cluding UAVs, robots, and mechanical systems. However, as with any control strategy,  
                         backstepping control has some limitations. Backstepping control can be complex to design  
                         and implement, especially for systems with many states or highly nonlinear dynamics. The  
                         controller design involves multiple steps, and the resulting controller may be difficult to  
                         analyze and understand. Backstepping control assumes that the system model is known,  
                         which may not be true in practice. If there are uncertainties or disturbances in the system,  
                         the controller may not perform well and may even become unstable. Backstepping control  
                         relies on accurate measurements of the system states. If there is measurement noise, the  
                         controller’s performance may be degraded. The performance of the backstepping con-  
                         troller depends on the choice of tuning parameters, such as gains and feedback coefficients.  
                         Tuning these parameters can be time-consuming and may require expert knowledge. Back-  
                         stepping control may not be robust against changes in the system parameters or external  
                         disturbances. This can be a problem in practical applications where the system may experi-  
                         ence uncertain or varying environmental conditions. Finally, backstepping control is not a  
                         panacea and may not be suitable for all types of MIMO channel systems or applications.  
                         Proper consideration of these limitations and appropriate modifications may be necessary  
                         for successful application in specific situations.  
  
                         5.3. Feedback Linearization Control  
                               Instead of linearizing via small angle approximation, this control paradigm transforms  
                         the UAV’s nonlinear dynamics into linear corresponding dynamics via feedback. This control  
                         method has been used by several researchers to create UAV flight controllers [53,106–114].  
                         Due to the nonlinear nature of UAV dynamics, this method is well suited for removing  
                         nonlinearities, after which linear control theory can be employed to develop a flight  
                         controller. Feedback linearization eliminates the system’s nonlinearity by transforming the  
                         variables and choosing the right input. Then, using an outer feedback loop, a new control  
                         input v is computed using linear control theory. The transformation must be diffeomorphic  
                         in order to guarantee that it is comparable to the original system. Zero dynamics, or states  
                         that cannot be observed from the system output, are present in transformed systems, and  
                         may cause instability; therefore, the stability of these zero dynamics should be maintained.  
                         Due to the high-order Lie derivatives used in the linearization process, this technique  
                         is quite susceptible to outside disturbances. The fact that feedback linearization only  
                         applies to a particular class of nonlinear systems that satisfy a particular involutivity  
                         criterion is another disadvantage of this technique [115–117]. Zero dynamics, also known as  
                         internal dynamics, are crucial in determining the stability of the control system. Technically  
Sensors 2024, 24, 3286                                                                                             21 of 51  
  
  
  
                         speaking, zero dynamics are the internal system states that are not visible from the system  
                         output. These dynamics could be unstable and harmful for the internal states, as they can  
                         become unbounded. It is necessary to take steps to guarantee that these internal dynamics  
                         do not lead to stability issues, and these unobservable states should at the very least be  
                         stable or controllable; see [53,115,118] for further information on zero dynamics.  
                               A comprehensive study of accurate linearization for a UAV model has been presented  
                         in [53,119]. This study showed that static feedback cannot solve the decoupling issue of  
                         the UAV paradigm. Instead, it was discovered that dynamic feedback can transform a  
                         system into one that is linear, disconnected, and non-interactive. Furthermore, dynamic  
                         feedback can guarantee the controllability of the altered system. On the basis of attitude  
                         representation, an exact input-output linearization of the UAV model was developed  
                         in [107,120]. The linearization of the z and ( x, y) states was performed separately, resulting  
                         in what is known as quasi-static feedback linearization [120,121]. A similar investigation  
                         of accurate linearization can be found in [53,106,122], while the inclusive mathematical  
                         design and feedback linearization approach can be found in [109]. By adding successive Lie  
                         derivative terms, UAV nonlinear dynamics were converted to linear dynamics. Transformed  
                         linear systems have derivative terms up to the third order, which renders the control  
                         strategy extremely susceptible to external errors and sensor noise. Similar work with  
                         feedback linearized controllers has been reported in [11,53]. In [106,108], along with a fix  
                         for rotor failure, the authors proposed a feedback linearization-based aircraft controller  
                         for the problem of trajectory tracking. The same method was used to achieve steady  
                         rotational motion across the z-axis in the event of rotor failure. A similar investigation was  
                         presented in [113,115], where several fault scenarios involving UAV flights were looked  
                         into. Attitude stabilization was accomplished by combining feedback linearization control  
                         and an inner loop.  
  
                         5.3.1. Mathematical Formulation  
                              Feedback linearization is a control design technique that aims to transform a nonlinear  
                         system into a linear one through a suitable change of coordinates. For a TRMS UAV, the  
                         pitch (θ) and yaw (ψ) angles can be controlled using feedback linearization.  
                              The dynamic equations for the pitch and yaw angles of a TRMS UAV can be represented  
                         as shown below.  
                                                           Ixx θ̈ = τθ − ( Izz − Iyy )ψ̇ϕ̇                        (28)  
                                                           Iyy ψ̈ = τψ − ( Ixx − Izz )ϕ̇θ̇                            (29)  
  
                              To apply feedback linearization, we introduce a change of coordinates to linearize the  
                         system. We first define the following virtual control inputs.  
  
                                                                  v1 = θ̇ − θ̇ref                                     (30)  
                                                                  v2 = ψ̇ − ψ̇ref                                     (31)  
  
                              The transformed system can be written as follows:  
  
                                                              v̇1 = −k θ (v1 − θ̇ref )                                (32)  
                                                              v̇2 = −k ψ (v2 − ψ̇ref )                                (33)  
  
                         where k θ and k ψ are the control gains and θ̇ref and ψ̇ref are the desired pitch and yaw rates.  
                             The control torques τθ and τψ can then be determined using the inverse dynamics.  
  
                                                           τθ = Ixx θ̈ + ( Izz − Iyy )ψ̇ϕ̇                            (34)  
                                                           τψ = Iyy ψ̈ + ( Ixx − Izz )ϕ̇θ̇                            (35)  
  
                             Substituting the expressions for θ̈ and ψ̈ from the transformed system into the inverse  
                         dynamics equations yields the control inputs required to achieve the desired pitch and yaw rates.  
Sensors 2024, 24, 3286                                                                                         22 of 51  
  
  
  
                              By applying feedback linearization, the pitch and yaw angles of the TRMS UAV can  
                         be effectively controlled by linearizing the system and designing control inputs to track  
                         the desired rates. This approach offers a systematic way of handling the nonlinearities and  
                         coupling effects present in the UAV dynamics, ensuring stable and precise control of the  
                         UAV’s motion.  
  
                         5.3.2. Remarks  
                               Feedback linearization is a useful technique for changing a nonlinear system into  
                         a matching linear system; however, as already discussed, it suffers from instability zero  
                         dynamics. Linearization of a quadrotor produces zero dynamics, as its nonlinear dynamics  
                         are underactuated. By placing two consecutive integrators in the direction of each control  
                         input, as in [123,124], the issue of unstable zero dynamics can be handled. The complexity  
                         of zero dynamics requires in-depth analysis. The need for full-state statistics is one of the  
                         biggest drawbacks of the feedback linearization approach, necessitating the design of a  
                         separate observer/estimator for estimation of the system’s states. In [123,125], observer  
                         design and feedback linearization for UAVs were both achieved. The third derivative of the  
                         output states is required for feedback linearization of a UAV model; however, in this study  
                         the observer model was employed to constrain the third derivative, making the control  
                         technique more acceptable for use with nonlinear systems.  
                               The UAV model’s parametric uncertainties may lead to performance problems. This  
                         calls for the application of adaption laws and control strategies that can lessen paramet-  
                         ric uncertainty and accommodate changes in parametric values. A linearized adaptive  
                         feedback controller was created in [126] and a thorough structure was provided. A similar  
                         investigation of an adaptive feedback linearized controller architecture was presented  
                         in [123,127]. The adaptive technique decreased the tracking error and enhanced the con-  
                         troller’s performance. Through adaptation, the control parameters can be changed, improv-  
                         ing the overall performance of the control system.  
  
                         5.3.3. Remarks  
                              When the system variables are known and there is no related uncertainty, feedback  
                         linearization performs well; on the other hand, UAVs may encounter significant problems  
                         due to their own parametric instabilities and feedback linearization-based controllers. In  
                         order to completely solve the aforementioned issue, an adaptive intelligent technique must  
                         be used along with traditional feedback linearization. Control performance can be greatly  
                         enhanced by online updating and by employing nonlinear approximators to approximate  
                         the unknown parameters.  
  
                         5.4. Model Predictive Control (MPC)  
                              MPC is a nonlinear control technique that can optimize the control inputs over a finite  
                         time horizon. MPC has been used for UAVs in control applications such as surveillance  
                         and mapping tasks. In one study [128–132], MPC was used to control a fixed-wing UAV  
                         for surveillance tasks. The results showed that MPC was able to improve the accuracy  
                         and efficiency of the UAV during the surveillance task [129,133]. MPC has a prediction ad-  
                         vantage over LQR control because the latter cannot foresee future control inputs [134,135].  
                         Second, although LQR only offers optimal control for linearized plant model systems, MPC  
                         may be used for nonlinear systems as well. Without a precise prediction model, the MPC  
                         controller’s algorithms cannot achieve extraordinary stability and performance [135]. As a  
                         result, if the prediction model is imprecise, the system may become unstable, and creating  
                         such a model demands an expensive control design effort. In addition, the capacity of  
                         adaptive MPC to estimate uncertainty makes it more reliable and useful for optimizing  
                         the control of a UAV. Future research could concentrate on developing an adaptive MPC  
                         aircraft controller, as this area of research into UAV control has seen little study.  
Sensors 2024, 24, 3286                                                                                               23 of 51  
  
  
  
                         5.4.1. Mathematical Formulation of MPC  
                              Model Predictive Control (MPC) is an advanced control strategy that uses a predictive  
                         model of the system to optimize control inputs over a finite horizon. For a TRMS UAV, the  
                         pitch (θ) and yaw (ψ) angles can be controlled using MPC.  
                              The dynamic equations for the pitch and yaw angles of a TRMS UAV are represented in  
                         following equations.  
                                                          Ixx θ̈ = τθ − ( Izz − Iyy )ψ̇ϕ̇                      (36)  
                                                              Iyy ψ̈ = τψ − ( Ixx − Izz )ϕ̇θ̇                           (37)  
  
                              In MPC, we formulate a predictive model of the system and optimize a control se-  
                         quence to minimize a cost function over a finite prediction horizon. We can define the state  
                         vector as shown below.                           
                                                                         θ  
                                                                        θ̇   
                                                                 x=   ψ  
                                                                                                                (38)  
                                                                         ψ̇  
  
                              The state space representation of the system can be written as follows:  
  
                                                                       ẋ = Ax + Bu                                     (39)  
                                                                       y = Cx                                           (40)  
  
                         where                                                            
                                                                              0        0  
                                                                
                                           0     1   0       0  
                                         0                                1          0  
                                                                                                                
                                                 0   0       0                                  1   0   0   0  
                                       A=                     ,     B =  Ixx          ,   C=                 .      (41)  
                                                                                          
                                         0      0   0       1           0           0        0   0   1   0  
                                                                                       1  
                                           0     0   0       0               0        Iyy  
  
                              The control inputs u can be represented as follows:  
                                                                                      
                                                                                  τθ  
                                                                        u=             .                                (42)  
                                                                                  τψ  
  
                             In MPC, the control inputs are optimized by solving the following optimization  
                         problem at each time step:  
                                                    min J ( x, u)                                       (43)  
                                                         u  
                                                         s.t.       x (k + 1) = Ax (k) + Bu(k)                          (44)  
                                                                    umin ≤ u ≤ umax                                     (45)  
  
                         where J ( x, u) is the cost function and umin and umax are the minimum and maximum torque  
                         limits, respectively.  
                              By solving this optimization problem at each time step, MPC computes the optimal  
                         control inputs that minimize the cost function while satisfying the system dynamics and  
                         control constraints. This enables precise and robust control of the pitch and yaw angles of  
                         TRMS UAVs even in the presence of uncertainties and disturbances.  
  
                         5.4.2. Remarks  
                              MPC has several benefits for controlling highly coupled wind rotor MIMO systems  
                         such as UAVs. First, MPC is capable of accurately tracking a desired trajectory for the  
                         UAV. This is important for UAVs that need to follow specific flight paths or perform  
                         complex maneuvers. Second, MPC is robust against uncertainties and disturbances in the  
                         UAV’s dynamics, making it a suitable control method for UAVs operating in dynamic and  
                         uncertain environments. Third, MPC can easily handle constraints on the UAV’s inputs and  
                         states. This is important for UAVs that need to operate within certain physical limitations,  
Sensors 2024, 24, 3286                                                                                         24 of 51  
  
  
  
                         such as maximum velocity, altitude, or acceleration. Third, MPC can simultaneously  
                         optimize multiple objectives, such as minimizing energy consumption while maximizing  
                         tracking performance. This makes MPC a versatile control method that can adapt to  
                         different operating conditions and environmental factors. Fourth, MPC can be easily  
                         integrated with other systems, such as sensors, communication networks, and mission  
                         planning software, making it a suitable control method for UAVs in complex and dynamic  
                         environments. Fifth, MPC can be easily adapted to different UAV configurations, allowing it  
                         to handle different operating conditions and environmental factors. Finally, MPC produces  
                         control signals that are smooth and continuous, which reduces chattering in the control  
                         signal. This results in smoother control actions and reduces wear and tear on the UAV’s  
                         mechanical components.  
                              While MPC has many benefits for controlling highly coupled wind rotor MIMO  
                         systems such as UAVs, there are also limitations to this control method. First, MPC requires  
                         solving optimization problems at each time step, which can be computationally intensive  
                         and may require high-performance computing resources. This can be a limitation for UAVs  
                         that require fast and real-time control responses. Second, MPC requires tuning of several  
                         parameters, such as the prediction horizon, control horizon, and weighting factors for the  
                         optimization objectives. If these parameters are not tuned correctly, the control performance  
                         can be degraded. This can be a limitation for UAVs that operate in changing environments  
                         where the optimal parameter values may vary. Third, MPC is dependent on the accuracy  
                         of the mathematical model used to describe the system dynamics. If the model is not  
                         accurate, the control performance can be compromised. This can be a limitation for UAVs  
                         that have complex and highly nonlinear dynamics. Fourth, MPC relies on predicting the  
                         future state of the system, which can lead to a delayed response to disturbances or changes  
                         in the environment. This can be a limitation for UAVs that require fast and immediate  
                         responses to changes in the environment. Fifth, MPC is a complex control method that  
                         requires a significant amount of technical expertise to implement and maintain. This can be  
                         a limitation for UAVs that have limited resources or operate in remote locations. Finally,  
                         the control decisions made by MPC can be difficult to interpret or explain, making it  
                         challenging to diagnose or troubleshoot control system issues.  
                              While MPC has many benefits for controlling highly coupled MIMO systems such as  
                         UAVs, the above limitations must be carefully considered when selecting a control method  
                         for a particular application. Proper implementation and tuning are critical to ensuring  
                         the control system’s stability, tracking performance, and robustness in the presence of  
                         disturbances and uncertainties.  
  
                         5.5. Contribution of Nonlinear Control Strategies for UAVs  
                              Nonlinear control strategies offer a diverse range of approaches that can be effectively  
                         applied to UAV systems, each with its own unique advantages and capabilities. Among  
                         the nonlinear control strategies discussed in this review, MPC stands out for its ability to  
                         handle complex dynamical systems and optimize control inputs over a predictive horizon,  
                         making it particularly suitable for UAVs operating in dynamic environments. SMC, on  
                         the other hand, excels in robustness against model uncertainties and external disturbances,  
                         ensuring reliable performance in challenging conditions. Adaptive and neural network-  
                         based control strategies offer flexibility and adaptability by learning from the UAV’s  
                         environment, enabling autonomous decision-making and adaptation to varying operating  
                         conditions. Finally, feedback linearization provides a systematic approach to transforming  
                         nonlinear dynamics into linear ones, facilitating the design of linear controllers that can  
                         effectively stabilize and control UAV systems. Each of these nonlinear control strategies  
                         has been tailored to address specific challenges encountered in UAV applications, offering  
                         a versatile toolkit for enhancing UAV performance, autonomy, and reliability.  
Sensors 2024, 24, 3286                                                                                                       25 of 51  
  
  
  
                                  6. Hybrid Control Strategies  
                                      Hybrid control theory is a branch of modern control theory that focuses on systems  
                                 demonstrating both continuous and discrete behaviors. These systems, referred to as hybrid  
                                 systems, present a unique challenge due to their combination of continuous dynamics (such  
                                 as those seen in traditional control systems with smooth and continuous changes) and  
                                 discrete events (such as switching between different operational modes or the occurrence  
                                 of events triggering abrupt changes in behavior) [136,137].  
                                      In hybrid systems, the interaction between continuous dynamics and discrete events  
                                 is fundamental. Continuous dynamics represent the gradual evolution of system states  
                                 over time, while discrete events introduce sudden changes or switches in the system’s  
                                 behavior or mode of operation. For example, in a hybrid control system for an electric  
                                 vehicle, continuous dynamics might govern the motor’s speed and acceleration, while  
                                 discrete events could include switching between driving modes (e.g., electric-only mode  
                                 and hybrid mode) or activating regenerative braking when certain conditions are met [138].  
                                      Hybrid control theory aims to develop modeling, analysis, and control techniques  
                                 tailored to these complex systems considering both continuous and discrete aspects. It  
                                 addresses challenges such as the design of controllers that can handle both continuous  
                                 dynamics and discrete events, thereby ensuring stability and performance in the face of  
                                 mode switches or sudden disturbances and optimizing system behavior under various op-  
                                 erating conditions. Overall, hybrid control theory provides a framework for understanding  
                                 and effectively managing the intricate behavior of hybrid systems in various engineering  
                                 applications [136]. Developments in hybrid control are provided in Table 3.  
  
                                 Table 3. Hybrid control developments.  
  
 Year      Contributor                                                History  
                                                                      Proposed a framework for analyzing the behavior of hybrid  
 1980s     Edward A. Lee and Alberto L. Sangiovanni-Vincentelli  
                                                                      systems, introducing the concept itself.  
                                                                      Over time, hybrid control theory has witnessed significant  
                                                                      progress, propelled by contributions from numerous  
                                                                      researchers. Among the pivotal advancements in this  
 1990s     Rajeev Alur, Thomas A. Henzinger, and Orna Kupferman.      domain is the introduction of the hybrid automaton model.  
                                                                      This model serves as a formal framework for both  
                                                                      representing and scrutinizing the intricate dynamics of  
                                                                      hybrid systems [136,137].  
                                                                      Additional significant contributions to hybrid control theory  
                                                                      encompass the formulation of control synthesis  
                                                                      methodologies. These methods are instrumental in crafting  
                                                                      effective control strategies tailored specifically for hybrid  
 1995      Henzinger, T. A., and Kopke, P. W.  
                                                                      systems. Moreover, extensive research has been dedicated to  
                                                                      investigating the stability and performance characteristics  
                                                                      inherent in hybrid systems, further enriching the  
                                                                      understanding of their complex behavior [138].  
                                                                      Another noteworthy advancement in hybrid control theory  
                                                                      is the emergence of reachability analysis techniques. These  
                                                                      methods serve the crucial function of identifying the range  
                                                                      of states attainable by a hybrid system starting from a  
 2008      Cassandras, C. G., and Lafortune, S.  
                                                                      specified initial state. Demonstrating efficacy in analyzing  
                                                                      the behavior of hybrid systems, reachability analysis  
                                                                      methodologies also play a pivotal role in the formulation of  
                                                                      tailored control strategies for such systems [139,140].  
  
  
                                       UAVs represent a sophisticated control system utilized within the UAV’s operations.  
                                  In this context, hybrid control plays a crucial role due to its capability to offer a robust and  
                                  adaptable control strategy. In a hybrid control setup, various control methodologies are  
                                  integrated to enhance performance and ensure stability. This approach proves particularly  
Sensors 2024, 24, 3286                                                                                         26 of 51  
  
  
  
                         beneficial in UAV applications in light of the diverse and dynamic environments in which  
                         these systems operate. By amalgamating different control techniques, the UAV system can  
                         effectively adjust to varying conditions and uphold stability, thereby enhancing its overall  
                         operational efficiency and reliability [141,142].  
  
                         6.1. Adaptive Sliding Mode Control (ASMC)  
                               Adaptive sliding mode control (ASMC) is a variant of SMC that incorporates adap-  
                         tive techniques to improve the control performance of nonlinear MIMO (multiple-input  
                         multiple-output) systems. For several reasons, ASMC is generally considered to be better  
                         than SMC for nonlinear MIMO systems. Nonlinear MIMO systems often have complex and  
                         uncertain dynamics that can degrade the performance of conventional control strategies  
                         such as SMC. ASMC incorporates adaptive mechanisms that allow the control parameters  
                         to be adjusted in real time based on the system’s current operating conditions, making it  
                         more robust against model uncertainties [143]. ASMC can achieve better tracking accuracy  
                         compared to SMC for nonlinear MIMO systems. This is because ASMC uses a model refer-  
                         ence adaptive control (MRAC) approach that provides more accurate tracking of the desired  
                         trajectory while maintaining robustness against system uncertainties [144]. SMC can suffer  
                         from chattering, which is a phenomenon characterized by high-frequency oscillations in  
                         the control signal. ASMC can reduce chattering by incorporating adaptive gain scheduling,  
                         which adjusts the control gain in real time to reduce the effects of chattering and improve  
                         control performance [145]. ASMC can achieve the same control performance as SMC with  
                         lower control effort, which can be beneficial for systems with limited actuator capacity  
                         or those that operate in harsh environments [143,145]. Overall, ASMC offers several ad-  
                         vantages over SMC for nonlinear MIMO systems, including improved robustness, better  
                         tracking accuracy, and reduced chattering. However, the design and implementation of  
                         ASMC can be more complex than SMC, and it may require higher computational overhead.  
                         Adaptive sliding mode control (ASMC) is a nonlinear control strategy that has been applied  
                         to UAVs for effective control in the presence of uncertainties and disturbances. ASMC has  
                         been shown to be effective in controlling UAV in various scenarios. In one study [146],  
                         an ASMC approach was used for the position and attitude control of a UAV. The results  
                         showed improved control performance compared to traditional linear control methods.  
                         In another study [147], an ASMC approach was used for the stabilization of a UAV in the  
                         presence of parameter uncertainties, with the results showing good robustness and dis-  
                         turbance rejection. The main advantage of ASMC is its ability to handle uncertainties and  
                         disturbances by using a sliding mode control law that drives the system to a sliding surface,  
                         where it can maintain stability and robustness. Additionally, the use of an adaptive law  
                         allows the system to adapt to changing conditions and uncertainties, improving the overall  
                         control performance. Although ASMC is an efficient control strategy, the convergence time  
                         must be faster and the system should move toward stability more quickly.  
  
                         Remarks  
                               While adaptive SMC is more robust than conventional SMC, it has slower convergence;  
                         therefore, ATSMC can be used to achieve faster convergence along with chattering removal.  
                         According to the above discussion, using ATSMC with an adaptation design can result  
                         in the best control performance. Furthermore, ATSMC offers a simple nonlinear design  
                         procedure for flight controllers, which leads to lower computational costs compared to  
                         other methods. Overall, the findings highlight the benefits of SMC and the use of ATSMC  
                         for flight control to achieve better control performance, faster convergence, and reduced  
                         computational cost, particularly with adaptive designs.  
  
                         6.2. Adaptive Fast-Terminal Sliding Mode Control (AFTSMC)  
                               UAVs are a popular benchmark system in the field of control engineering. Adaptive  
                         fast-terminal sliding mode control (AFTSMC) is a robust control technique that has been  
Sensors 2024, 24, 3286                                                                                           27 of 51  
  
  
  
                         applied to UAVs to achieve improved control performance. The design procedure of  
                         AFTSMC for UAVs involves the following steps:  
                         •    System modeling: The first step is to develop a mathematical model of the UAV. The  
                              model should include the dynamics of both the rotor and the platform, and should be  
                              expressed in state space form.  
                         •    Control objective: The control objective is to design a control law that can stabilize the  
                              UAV at a desired position and orientation while rejecting disturbances and uncertainties.  
                         •    Sliding mode control: AFTSMC is based on sliding mode control (SMC), which  
                              involves the design of a sliding surface that ensures fast convergence to the desired  
                              state. The sliding surface should be designed such that its derivative is negative  
                              definite and the system trajectory approaches it asymptotically.  
                         •    Terminal sliding mode control: In addition to SMC, AFTSMC incorporates terminal  
                              sliding mode control (TSMC) to achieve faster convergence to the desired state. TSMC  
                              involves the design of a terminal sliding surface which is reached in a finite time and  
                              remains stable thereafter.  
                         •    Adaptive control: AFTSMC incorporates adaptive control to account for uncertainties  
                              in the system parameters. Adaptive control involves the design of an adaptation law  
                              that updates the control gains in real time based on the estimated system parameters.  
                         •    Design of control gains: The final step is to design the control gains such that the  
                              sliding surface and the terminal sliding surface are reached in a finite time and the  
                              system remains stable thereafter. The control gains can be tuned using simulation or  
                              experimental data to achieve optimal performance.  
                               The design procedure of AFTSMC for UAVs involves the integration of SMC, TSMC,  
                         and adaptive control to achieve robust and efficient control performance. AFTSMC is an im-  
                         provement over ASMC that addresses some of the limitations of the latter. Among the main  
                         advantages of AFTSMC over ASMC, it converges to the sliding surface much faster due to  
                         the addition of a terminal sliding mode surface that has a higher convergence rate. This  
                         means that AFTSMC can achieve better tracking performance in a shorter time. In addition,  
                         AFTSMC is more robust against parameter uncertainties and disturbances compared to  
                         ASMC, as the terminal sliding mode surface provides guarantees convergence within a  
                         finite time even in the presence of disturbances and parameter uncertainties [148]. Finally,  
                         AFTSMC reduces chattering compared to ASMC. Chattering is a phenomenon in which  
                         the control signal switches rapidly between different values; this can cause wear and tear  
                         on the actuators, and reduces the overall system performance [149]. Adaptive fast-terminal  
                         sliding mode control (AFTSMC) is a nonlinear control technique that has been applied to  
                         UAVs for effective control in the presence of uncertainties and disturbances. AFTSMC is an  
                         extension of the traditional sliding mode control approach which incorporates a terminal  
                         sliding mode to improve the speed and accuracy of the system’s response. Several studies  
                         have investigated the use of AFTSMC for UAV control. In [149], AFTSMC was used for  
                         the attitude control of a UAV in the presence of uncertainties and external disturbances.  
                         The results showed improved control performance compared to traditional sliding mode  
                         control approaches. In another study [148] where AFTSMC was used for the position  
                         control of a UAV, the results again showed good robustness and disturbance rejection.  
                               The key advantage of AFTSMC is its ability to achieve fast and accurate tracking of  
                         the desired trajectory even in the presence of uncertainties and disturbances. The terminal  
                         sliding mode allows the system to converge to a desired state in a finite time, while the  
                         adaptive law ensures that the system can adapt to changing conditions and uncertainties.  
  
                         6.2.1. Remarks for AFTSMC, ASMC, and SMC  
                              Sliding surface-based control strategies are a class of control methods that rely on the  
                         design of a sliding surface, which is a function of the system state that is used to enforce a  
                         desired behavior or trajectory of the system. Several sliding surface-based control strategies,  
                         such as sliding mode control, adaptive sliding mode control, and adaptive fast terminal  
                         sliding mode control, have been reviewed for use in controlling highly coupled rotor MIMO  
Sensors 2024, 24, 3286                                                                                        28 of 51  
  
  
  
                         systems such as UAVs. General remarks that apply to all sliding surface-based control  
                         strategies are provided below:  
                         Limitations  
                              While sliding surface-based control has several benefits for twin-rotor MIMO systems  
                         such as UAVs, it has several limitations as well. These include:  
                         •   Control signal chattering: Although first-order SMC can reduce chattering compared  
                             to higher-order sliding mode control techniques, it can still generate high-frequency  
                             oscillations in the control signal. This can cause wear and tear on mechanical compo-  
                             nents, decrease the lifespan of the vehicle, and lead to suboptimal control performance.  
                         •   Parameter sensitivity: First-order SMC can be sensitive to changes in the system  
                             parameters, such as the mass and moment of inertia of the UAV. This can lead to poor  
                             control performance or even instability if the parameters are not accurately known or  
                             change during operation.  
                         •   Simplicity of implementation: First-order SMC is relatively easy to implement and  
                             does not require extensive tuning of control parameters. This can reduce the develop-  
                             ment time and cost of UAV control systems.  
                         •   Reduced tracking accuracy: First-order SMC may not provide the same level of  
                             tracking accuracy as other control techniques such as model predictive control or  
                             linear quadratic regulator. This can be a limitation in applications where precise  
                             tracking of a desired trajectory is critical.  
                         •   Limited applicability: First-order SMC may not be suitable for all types of UAVs or  
                             operating conditions; for example, it may not be effective for highly dynamic systems  
                             with rapid changes in speed or acceleration.  
                         •   Limited convergence rate: The adaptation process in ASMC can lead to slower conver-  
                             gence rates compared to traditional sliding mode control techniques. This can be a  
                             limitation for UAVs that need to respond quickly to changes in their environment.  
                         •   Sensitivity to modeling errors: ASMC can be sensitive to modeling errors, which can  
                             lead to poor control performance or even instability. This is because the adaptation  
                             process relies on accurate knowledge of the system dynamics.  
                         •   Limited applicability: ASMC may not be suitable for all types of UAVs or operating  
                             conditions. For example, it may not be effective for highly nonlinear systems or  
                             systems with significant time delays.  
                         •   Chattering: A common issue with sliding surface-based control strategies is chattering,  
                             which involves high-frequency oscillation of the control signal around the desired  
                             value. Chattering can cause mechanical wear and tear in the actuators as well as noise  
                             and vibration. Techniques such as the use of saturation functions or the introduction  
                             of a switching gain can reduce the effect of chattering.  
                         •   Non-smoothness: Sliding surface-based control strategies are non-smooth, which  
                             means that the control signal can switch abruptly between different values. This non-  
                             smoothness can cause difficulties in the implementation of the control law, and can  
                             introduce high-frequency noise and vibration. Careful consideration of the physical  
                             limitations of the actuators and sensors is required to ensure that the control law can  
                             be implemented smoothly.  
                         •   Model dependency: The performance of sliding surface-based control strategies is  
                             highly dependent on the accuracy of the system model. Errors in the model can lead  
                             to poor performance or instability. Techniques such as adaptive control or model  
                             predictive control can be used to address this issue.  
                         •   Potential for actuator wear: The high-frequency control actions generated by TSMC  
                             can potentially cause wear and tear on the UAV’s mechanical components, such as its  
                             actuators. This can lead to increased maintenance costs and reduced system reliability  
                             over time.  
                              AFTSMC may require the use of specialized hardware or software, such as high-  
                         speed processors or real-time operating systems. This can add to the complexity and  
                         cost of implementing the control technique. Adaptive fast-terminal sliding mode control  
Sensors 2024, 24, 3286                                                                                           29 of 51  
  
  
  
                         (AFTSMC) is a relatively new control technique that has shown promising results in  
                         various nonlinear control applications. However, as with any control technique, it has  
                         its limitations.  
                         •   One limitation of AFTSMC is that it can suffer from chattering, which is a phenomenon  
                             in which the control signal oscillates rapidly around the desired value. This can result  
                             in high-frequency noise as well as potential wear and tear on the mechanical compo-  
                             nents of the system. Several researchers have proposed modifications to AFTSMC  
                             to reduce chattering, such as adding a boundary layer to the sliding mode control or  
                             using fuzzy logic to dynamically adjust the sliding mode gain [149,150].  
                         •   Another limitation of AFTSMC is that it can be sensitive to model uncertainties and  
                             disturbances. While AFTSMC is designed to be adaptive to such uncertainties, in  
                             practice there may be situations where the uncertainties are too large or the adaptation  
                             process is too slow to compensate adequately. Several researchers have proposed mod-  
                             ifications to AFTSMC to improve its robustness, such as using disturbance observers  
                             or incorporating online learning algorithms [151,152].  
                             Sliding surface-based control strategies offer a powerful tool for controlling highly  
                         coupled rotor MIMO systems such as UAVs. While they provide robustness to distur-  
                         bances and uncertainties, careful consideration of chattering, non-smoothness, and model  
                         dependency is required to ensure the successful implementation of the control law.  
                         Benifits of AFTSMC over SMC, TSMC, and ASMC  
                              AFTSMC is a robust control technique that has several advantages over other sliding  
                         surface-based hybrid controllers for UAVs. Among the benefits of AFTSMC are the following:  
                         •    Fast convergence: AFTSMC combines SMC and TSMC to achieve fast convergence to  
                              the desired state. TSMC enables the system trajectory to reach the sliding surface in a  
                              finite time, while SMC ensures that the sliding surface is stable thereafter. This results  
                              in faster convergence compared to other sliding surface-based hybrid controllers.  
                         •    Robustness: AFTSMC incorporates adaptive control to account for uncertainties in the  
                              system parameters. The adaptation law updates the control gains in real time based  
                              on the estimated system parameters, which enhances the robustness of the controller.  
                              This makes AFTSMC more effective in dealing with uncertainties and disturbances  
                              compared to other hybrid controllers.  
                         •    Chattering reduction: Chattering is a common problem in sliding mode control, result-  
                              ing in high-frequency oscillations in the control signal. AFTSMC reduces chattering  
                              by incorporating a fast terminal sliding surface, which reduces the time spent on the  
                              sliding surface, and consequently the amplitude of the oscillations.  
                         •    Reduced control effort: AFTSMC reduces the control effort required to stabilize the  
                              UAV. Both the sliding surface and the terminal sliding surface are designed to minimize  
                              the control effort required to maintain stability, which reduces wear and tear on  
                              the system.  
                              Overall, AFTSMC combines the benefits of SMC, TSMC, and adaptive control to  
                         achieve faster convergence, enhanced robustness, reduced chattering, and reduced control  
                         effort compared to other sliding surface-based hybrid controllers for UAVs.  
  
                         6.2.2. Remarks  
                              Hybrid control is a control technique that combines multiple control strategies to  
                         achieve better control performance. Compared to other sliding surface-based controllers  
                         for UAVs, hybrid control has several advantages. Hybrid control has a number of benefits  
                         over AFTSMC and other sliding surface controllers. Hybrid control for UAVs can improve  
                         the robustness of the controller by combining multiple control strategies that can handle  
                         different types of uncertainties and disturbances. For example, hybrid control can combine  
                         adaptive control with model predictive control to achieve robust control performance in the  
                         presence of model uncertainties and disturbances. In addition, hybrid control can improve  
                         the tracking performance of the controller by combining different control strategies that  
Sensors 2024, 24, 3286                                                                                          30 of 51  
  
  
  
                         are optimized for different aspects of the control problem. For example, hybrid control can  
                         combine feed-forward control with feedback control to achieve better tracking performance  
                         while minimizing control effort. Hybrid control can reduce chattering by combining  
                         different control strategies that can mitigate chattering in different ways. For example,  
                         hybrid control can combine sliding mode control with integral action to reduce chattering  
                         while maintaining fast convergence. Finally, hybrid control is a flexible control technique  
                         that can be customized to suit specific control problems. By combining different control  
                         strategies, hybrid control can be tailored to optimize control performance while taking into  
                         account system constraints and other requirements.  
  
                         6.3. Adaptive Backstepping Control  
                              Adaptive backstepping control is a nonlinear control approach that is used to design  
                         control systems for complex dynamic systems such as UAVs. It has the ability to handle  
                         parametric uncertainties and external disturbances effectively. By using adaptive back-  
                         stepping control, UAVs can achieve better tracking performance, stability, and robustness.  
                         Several studies have proposed and investigated the use of adaptive backstepping control  
                         for UAVs. For instance, in [153] an adaptive backstepping control algorithm was proposed  
                         for UAVs with parametric uncertainties and external disturbances. The proposed algorithm  
                         was effective in tracking control and disturbance rejection. Another study [154] proposed  
                         an adaptive backstepping control method for UAVs with input saturation and external  
                         disturbances. The proposed control method was shown to achieve better tracking per-  
                         formance compared to traditional backstepping control. Moreover, in [155] an adaptive  
                         backstepping control method was proposed for UAVs with actuator saturation and external  
                         disturbances. The proposed method was shown to achieve improved tracking performance  
                         and robustness. Adaptive backstepping control has several benefits for controlling highly  
                         coupled twin-rotor MIMO systems such as UAVs, including the following:  
                         •    Stability: Adaptive backstepping control is a Lyapunov-based control method that  
                              ensures stability of the closed-loop system. This means that the control system is  
                              guaranteed to converge to a stable equilibrium point and remain there even in the  
                              presence of disturbances and uncertainties.  
                         •    Tracking performance: Adaptive backstepping control is capable of achieving high  
                              tracking performance, which is important for UAVs that need to follow specific flight  
                              paths and maintain a stable flight.  
                         •    Robustness: Backstepping control is a robust control method that can handle parameter  
                              uncertainties, external disturbances, and measurement noise. This is particularly  
                              important for UAVs that are subject to varying wind conditions, temperature changes,  
                              and other environmental factors that can affect their flight dynamics.  
                         •    Reduced chattering: Adaptive backstepping control produces control signals that are  
                              smooth and continuous, which reduces chattering in the control signal. This results in  
                              smoother control actions and reduces wear and tear on the UAV’s mechanical components.  
                         •    Reduced chattering: Backstepping control produces control signals that are smooth and  
                              continuous, which reduces chattering in the control signal. This results in smoother  
                              control actions and reduces wear and tear on the UAV’s mechanical components.  
                         •    Energy efficiency: Backstepping control can be designed to minimize energy consump-  
                              tion, which increases flight time and reduces the need for frequent battery replacements.  
                         •    Energy efficiency: Adaptive backstepping control can be designed to minimize en-  
                              ergy consumption, which increases flight time and reduces the need for frequent  
                              battery replacements.  
                         •    Adaptability: Adaptive backstepping control is capable of adapting to changes in the  
                              UAV’s dynamics over time. This makes it a versatile control method that can handle  
                              different operating conditions and environmental factors.  
                         •    Easy implementation: Adaptive backstepping control can be implemented using  
                              standard digital signal processing techniques, making it easy to implement in modern  
                              UAV control systems.  
Sensors 2024, 24, 3286                                                                                           31 of 51  
  
  
  
                              The benefits of backstepping control make it a promising approach for controlling  
                         highly coupled systems such as UAVs. By improving the stability, tracking performance,  
                         robustness, energy efficiency, and ease of implementation of control systems, backstep-  
                         ping control can help to improve the performance and reliability of UAVs in a variety  
                         of applications.  
  
                         Remarks  
                              While backstepping and adaptive backstepping control have many benefits for control-  
                         ling highly coupled systems such as UAVs, there are a number of limitations to this control  
                         method. The adaptive law used in adaptive backstepping control may not converge in  
                         certain situations, leading to unstable or oscillatory control performance. This can be a limi-  
                         tation for UAVs that operate in highly dynamic and uncertain environments. Backstepping  
                         and adaptive backstepping control are complex control methods that require a significant  
                         amount of computing power and may be difficult to implement in real-time applications.  
                         This can be a limitation for UAVs that require fast and accurate control responses. Back-  
                         stepping and adaptive backstepping control require tuning of several parameters, such as  
                         the control gains and the stability functions. If these parameters are not tuned correctly, the  
                         control performance can be degraded. This can be a limitation for UAVs that operate in  
                         changing environments where the optimal parameter values may vary. Backstepping and  
                         adaptive backstepping control are dependent on the accuracy of the mathematical model  
                         used to describe the system dynamics. If the model is not accurate, the control performance  
                         can be compromised. This can be a limitation for UAVs that have complex and highly  
                         nonlinear dynamics. Backstepping and adaptive backstepping control require accurate  
                         measurements of the UAV’s state variables, such as its position and velocity. This can be a  
                         limitation for UAVs that have limited sensor capabilities or operate in environments where  
                         accurate measurements are difficult to obtain. Finally, designing a backstepping control for  
                         a complex UAV system can be challenging, and may require significant expertise in control  
                         theory and UAV dynamics.  
                              Although backstepping and adaptive backstepping control have many benefits for  
                         controlling UAVs, these limitations must be carefully considered when selecting a control  
                         method for a particular application. Proper implementation and tuning are critical to  
                         ensure the control system’s stability, tracking performance, and robustness in the presence  
                         of disturbances and uncertainties.  
  
                         6.4. Adaptive Backstepping Fast-Terminal Sliding Mode Control (AB-FTSMC)  
                               Adaptive backstepping fast-terminal sliding mode control (ABFTSMC) is an advanced  
                         control strategy that has gained attention in recent years due to its ability to provide robust  
                         control for uncertain systems. In the context of a twin-rotor MIMO system, ABFTSMC can  
                         provide effective control for the system despite the presence of external disturbances and  
                         parametric uncertainties.  
                               One of the advantages of ABFTSMC is that it combines the benefits of both adap-  
                         tive control and sliding mode control. The adaptive control component allows the con-  
                         troller to adapt to changes in the system dynamics or uncertainties in real-time, while the  
                         sliding mode control component provides robustness against external disturbances and  
                         parametric uncertainties.  
                               Recent studies have demonstrated the effectiveness of ABFTSMC for UAVs. For  
                         example, in [156], ABFTSMC was used to control a UAV with input nonlinearity. The  
                         authors demonstrated that the proposed controller can effectively mitigate the effects of  
                         input nonlinearity and external disturbances on the system. Similarly, in [157], ABFTSMC  
                         was used to control a twin rotor MIMO system with actuator saturation and external  
                         disturbances. The authors demonstrated the proposed controller’s ability to effectively  
                         attenuate the effects of actuator saturation and external disturbances and showed that it  
                         outperformed other control strategies.  
Sensors 2024, 24, 3286                                                                                          32 of 51  
  
  
  
                         Remarks  
                               ABFTSMC uses adaptive backstepping and fast terminal sliding mode control tech-  
                         niques to achieve robust and accurate control of nonlinear MIMO systems. This con-  
                         troller is capable of handling uncertain system parameters, external disturbances, and  
                         input saturation, making it suitable for a wide range of practical applications. Simulation  
                         results [156,157] have demonstrated the effectiveness of the proposed controller in control-  
                         ling nonlinear MIMO systems, with improved tracking performance and reduced control  
                         input chattering compared to other control methods. The proposed controller represents a  
                         significant contribution to the field of nonlinear control and has the potential to be applied  
                         in various real-world scenarios such as robotics, aerospace, and industrial control systems.  
                         ABFTSMC is highly robust against parametric uncertainties, external disturbances, and  
                         nonlinearities in the system. This makes it a suitable choice for controlling nonlinear MIMO  
                         systems, which are typically subject to these types of uncertainty. ABFTSMC incorporates  
                         adaptive control techniques, which means that it can adapt to changes in the system dy-  
                         namics and uncertainties in real time. This makes it a highly flexible and adaptive control  
                         strategy. The fast=terminal sliding mode control component of ABFTSMC ensures that the  
                         control error converges to zero in a finite time. This means that the controller can quickly  
                         achieve the desired control objective even in the presence of uncertainties and disturbances.  
                         ABFTSMC can effectively handle input saturation, which is a common problem in MIMO  
                         systems. The adaptive component of the controller ensures that the system can operate  
                         within the input limits, while the sliding mode component ensures robustness against  
                         external disturbances and uncertainties.  
  
                         6.5. Model Predictive-Based Sliding Mode Control (MPSMC)  
                              Model predictive-based sliding mode control (MPSMC) is a hybrid control strategy  
                         that combines the benefits of MPC and SMC to control dynamic systems. In MPSMC, the  
                         SMC is used to design a sliding surface that ensures the system’s stability and tracking  
                         performance, while the MPC is used to optimize the control actions based on the predicted  
                         future behavior of the system.  
                              The basic idea behind MPSMC is to use the SMC to design a sliding surface that  
                         guarantees the system’s stability and robustness against uncertainties and disturbances.  
                         The sliding surface is defined as a hyperplane that separates the system’s behavior into  
                         two regions: the sliding mode and the reaching mode. In the sliding mode, the system’s  
                         behavior is constrained to follow the sliding surface, while in the reaching mode the sys-  
                         tem’s behavior is guided towards the sliding surface. In [158], the authors proposed an  
                         MPSMC strategy for the stabilization and trajectory tracking of a UAV in the presence of  
                         uncertainties and disturbances, presented a detailed explanation of the MPSMC approach,  
                         and demonstrated its effectiveness through simulations. In [159], the MPSMC strategy  
                         was expressed with a different sliding surface design based on the predicted future behav-  
                         ior of the UAV system. The authors showed the effectiveness of their approach through  
                         simulations and experiments. In [160], the MPSMC approach was exploited for the atti-  
                         tude stabilization of a UAV using a quaternion-based representation of the system. The  
                         authors explained the details of their approach and demonstrate its effectiveness through  
                         simulations and experiments.  
                              In their review article, the authors of [161] compared the performance of MPC, SMC,  
                         and MPSMC for the stabilization and trajectory tracking of UAVs. They provide a de-  
                         tailed explanation of the MPSMC approach and show its superiority in terms of tracking  
                         performance and robustness.  
  
                         Remarks  
                              After the sliding surface is designed, the MPC is used to optimize the control actions  
                         based on the predicted future behavior of the system. The MPC predicts the future states of  
                         the system and calculates the optimal control actions that minimize a cost function subject to  
                         constraints. The control actions are then applied to the system and the process is repeated at  
Sensors 2024, 24, 3286                                                                                          33 of 51  
  
  
  
                         each time step. The MPC can handle constraints on the system’s inputs and states, ensuring  
                         that the control actions remain within physical limitations. MPSMC can be easily adapted  
                         to different systems and operating conditions, making it a versatile control method. The  
                         SMC guarantees the system’s stability and tracking performance, while the MPC optimizes  
                         the control actions to minimize a cost function subject to constraints. MPSMC is a promising  
                         approach for controlling dynamical systems, including highly coupled wind rotor MIMO  
                         systems such as UAVs, that require robustness, tracking performance, constraints handling,  
                         adaptability, and reduced chattering. By combining the benefits of MPC and SMC, MPSMC  
                         can help to improve the performance and reliability of control systems for a variety of  
                         applications. Despite its many benefits, there are limitations on the use of model predictive  
                         based sliding mode control (MPSMC) to control highly coupled wind rotor MIMO systems  
                         such as UAVs, which are discussed below.  
                              The use of MPC in MPSMC can lead to increased computational complexity, especially  
                         for systems with large state and control spaces. This can lead to significant computational  
                         overhead and make it difficult to implement MPSMC in real-time control applications.  
                         MPSMC requires the tuning of several parameters, including the sliding surface, cost func-  
                         tion, and prediction horizon, which can be challenging and time-consuming. In addition,  
                         the tuning process can be sensitive to changes in the system dynamics or environmental  
                         conditions, requiring frequent recalibration. MPSMC relies on accurate system models for  
                         prediction and optimization. However, there may be modeling errors due to uncertainties,  
                         disturbances, or unmodeled dynamics that can affect the accuracy of the control actions.  
                         The use of MPC in MPSMC can lead to input saturation, where the control inputs reach  
                         their physical limits. This can result in degraded control performance and reduced stability  
                         margins. The complexity of MPSMC can make it difficult to interpret and understand the  
                         control actions and their effects on the system. This can make it challenging to diagnose  
                         and correct control errors or to explain the system’s behavior to stakeholders.  
  
                         6.6. Fuzzy Logic-Based Nonlinear Control Strategies  
                              Instead of using the Boolean logic that computers typically utilize, fuzzy logic operates  
                         on the concept of “degree of truth”. Because the degree of truth can have any real value  
                         among 1 and 0, it is a many-valued logic. This control strategy’s logical thinking is similar  
                         to that of humans in that there are many options in between the Boolean values “yes”  
                         and “no”. Therefore, in this method the level of input possibilities is used to compute  
                         the precise output. The word “fuzzy” used to describe this control strategy alludes to  
                         its ability to produce predictable outputs for a variety of distorted and unclear inputs.  
                         Fuzzy also means uncertain or confused. While fuzzy logic controllers do not depend on a  
                         precise mathematical model, other linear and nonlinear controllers are more dependent on  
                         a precise and accurate mathematical model of the system. This characteristic, which relies  
                         solely on approximation reasoning, makes building a control system for any complicated  
                         system simpler. The min–max rules, which combine the membership functions using  
                         OR and AND logic, are a significant flaw in fuzzy logic [162,163]. Both the robustness  
                         and the human-like reasoning for several inputs are lacking from these principles. This  
                         control strategy has been applied in several UAV control investigations [162–166], and  
                         better outcomes have been noted [166,167]. Investigations in several articles [166,168,169]  
                         explain hybrid control strategies based on backstepping. A robust adaptive controller for a  
                         UAV was designed using the aforementioned control schemes after the UAV dynamical  
                         model was initially published. The adaptive fuzzy system was utilized to approximate  
                         the value of the generated control law, while backstepping was employed to derive the  
                         control law required to stabilize and track the intended location. The resulting controller  
                         assures asymptotic tracking. The developed adaptive fuzzy backstepping controller and  
                         the backstepping controller’s comparative findings were compared to alternatives, with  
                         the proposed controller performing accurate tracking [170].  
                              Another study [171] concentrated on the notion of combining integral backstepping  
                         with fuzzy logic to enhance the performance of the flight controller. Simulation findings  
Sensors 2024, 24, 3286                                                                                          34 of 51  
  
  
  
                         showed that the fuzzy-integral backstepping strategy performed better than traditional  
                         integral backstepping. An evolutionary approach was used to add a control structure to  
                         the fuzzy-based sliding mode control for UAVs. To prevent chattering, this method can be  
                         researched further using second-order sliding mode control.  
  
                         Remarks  
                              The use of fuzzy logic-based nonlinear control designs for UAV systems is appropriate  
                         in that a global controller can be created by locally defining control laws in terms of verbal  
                         expressions. Another benefit of this method is that no precise model is required for the  
                         design of the controller. Instead, it necessitates thorough knowledge and experience of the  
                         designer with the specific system. The system’s local behavior can be changed through  
                         the defined local rules, which are user-generated and depend on selected membership  
                         functions. While fuzzy controllers can technically be used with nonlinear systems, proving  
                         their stability and robustness analytically is time-consuming.  
  
                         6.7. Neural Network-Based Nonlinear Control Strategies  
                               Artificial neural networks (ANNs) process information similarly to organic neurons. In  
                         this paradigm, the structure of the linked neurons and their capacity for adaptive learning is  
                         crucial for information processing. The processing components of interconnected neurons  
                         cooperate to solve a particular issue and learn from examples in a manner similar to  
                         humans. Learning in ANNs occurs in a similar way to how it occurs in biological neurons  
                         when synaptic connections are changed by training. This learning can take place online or  
                         offline, allowing ANNs to be used in practical applications.  
                               The utilization of ANNs for various tasks, such as data categorization, pattern iden-  
                         tification, picture recognition, speech recognition, handwriting recognition, extracting  
                         patterns from large datasets, sales forecasting, and risk management, has been the focus of  
                         significant research efforts. Moreover, ANNs find applications in electronics and control  
                         systems. In the realm of unmanned aerial vehicles, several studies have employed ANNs  
                         to develop controllers for UAV models and understand the complex nonlinear dynamics  
                         through their approximation capabilities. A notable example is the comprehensive explo-  
                         ration of backstepping control and neural network adaptation conducted by Madani and  
                         Benallegue [100,172].  
                               The deployment of ANNs spans a plethora of domains, including data categorization,  
                         pattern recognition, image analysis, speech processing, handwriting interpretation, data  
                         mining from extensive datasets, forecasting sales trends, and managing risks, all of which  
                         have garnered substantial attention in recent research endeavors. Furthermore, ANNs  
                         exhibit utility in electronics and control systems, showcasing considerable versatility. In  
                         the context of unmanned aerial vehicles, ANNs have been extensively utilized to craft  
                         controllers tailored to UAV models as well as to unravel the intricate nonlinear dynamics  
                         inherent in their operations, leveraging the approximation properties of neural networks.  
                         Noteworthy among these efforts is the in-depth investigation into backstepping control  
                         and neural network adaptation conducted by Madani and Benallegue [173].  
                               The utilization of Artificial Neural Networks spans across various domains, encom-  
                         passing tasks such as data classification, pattern recognition, image processing, speech  
                         analysis, and handwriting interpretation, as well as extracting insights from vast datasets  
                         for purposes such as sales prediction and risk assessment. Additionally, ANNs show-  
                         case their versatility in applications involving electronics and control systems. In the  
                         realm of unmanned aerial vehicles, ANNs have been extensively employed to develop  
                         controllers tailored to specific UAV models and to comprehend the intricate nonlinear  
                         dynamics governing their behavior, leveraging the approximation capabilities inherent  
                         in neural networks. A notable contribution in this regard is the exhaustive investigation  
                         into backstepping control and neural network adaptation undertaken by Madani and  
                         Benallegue [172].  
Sensors 2024, 24, 3286                                                                                         35 of 51  
  
  
  
                               One of the primary advantages of this proposed control strategy lies in its indepen-  
                         dence from detailed knowledge of physical characteristics and system models. Operating  
                         under a few broad assumptions, this control method ensures the convergence of all closed-  
                         loop dynamic states. Furthermore, it offers versatility, as it can be applied to UAVs of  
                         varying masses and inertia within the same class without necessitating knowledge of  
                         specific physical parameters. Notably, the concept of online adjustment of AANN weights,  
                         obviating the need for a separate learning phase, is elucidated in this approach. Nonlineari-  
                         ties inherent in the UAV model are estimated through multilayer ANNs.  
                               Several studies [110,174,175] have explored dynamic inversion and direct inverse  
                         control methods incorporating ANNs within the control system architecture. Meanwhile,  
                         sliding mode control stands out as a prevalent technique for managing nonlinear systems.  
                         This method operates by introducing a sliding manifold, which is a low-dimensional  
                         subspace of the system’s state space along which the system’s state is steered towards a  
                         desired trajectory.  
                               In [176], the authors proposed a neural network-based adaptive backstepping control  
                         approach for nonlinear MIMO systems with input saturation. The proposed approach uses  
                         neural networks to approximate the unknown nonlinearities of the system and adaptive  
                         laws to estimate the saturation boundaries of the input signals. The backstepping design  
                         approach is employed to design a series of feedback control laws that drive the system’s  
                         state to a desired trajectory. The control approach was shown to be effective in controlling  
                         a two-link robot arm system, and the results were compared with those obtained using  
                         a traditional backstepping control approach. The proposed approach was demonstrated  
                         to outperform the traditional approach in terms of tracking accuracy, control effort, and  
                         robustness against disturbances and uncertainties. This article provides a useful contribu-  
                         tion to the field of adaptive backstepping control and its application to nonlinear MIMO  
                         systems with input saturation. Another article [177] proposed an adaptive neural network  
                         backstepping control approach for MIMO nonlinear systems with time-varying input delay.  
                         The proposed approach uses a neural network to approximate the unknown nonlinearities  
                         of the system and a backstepping design approach to design a series of feedback control  
                         laws that drive the system’s state to a desired trajectory. An adaptive law was developed  
                         to estimate the time-varying input delay, which was then compensated for in the control  
                         design. The control approach was shown to be effective in controlling a three-tank system  
                         with time-varying input delay, and the results were compared with those obtained using a  
                         traditional backstepping control approach. The proposed approach was demonstrated to  
                         outperform the traditional approach in terms of tracking accuracy and robustness against  
                         input delay variations. This article provides a useful contribution to the field of adaptive  
                         backstepping control and its application to MIMO nonlinear systems with time-varying  
                         input delay.  
                               In [178], the authors proposed a robust adaptive neural network backstepping control  
                         approach for MIMO nonlinear systems with input saturation. The proposed approach  
                         uses a neural network to approximate the unknown nonlinearities of the system and a  
                         backstepping design approach to design a series of feedback control laws that drive the  
                         system’s state to a desired trajectory. An adaptive law was developed to estimate the  
                         saturation boundaries of the input signals, which were then compensated for in the control  
                         design. The control approach was shown to be effective in controlling an unmanned  
                         aerial vehicle system with input saturation, and the results were compared with those  
                         obtained using a traditional backstepping control approach. The proposed approach  
                         was demonstrated to outperform the traditional approach in terms of tracking accuracy  
                         and robustness to input saturation. This article provides a useful contribution to the  
                         field of adaptive backstepping control and its application to MIMO nonlinear systems  
                         with input saturation. Learning the nonlinear behavior of any complex system using  
                         neural network approximation property is a worthwhile technique to account for the  
                         system’s uncertainties and bounded disturbances. The nonlinear functions [179–181]  
                         have been commonly approximated using radial basis functions (RBFs). Extended radial  
Sensors 2024, 24, 3286                                                                                           36 of 51  
  
  
  
                         basis functions (E-RBFs) and normalized radial basis functions (N-RBFs) are examples of  
                         augmented RBF functions. In [181], both N-RBFs and E-RBFs were used to approximate a  
                         UAV’s unknown dynamics.  
                              In a few studies, the position controller has been viewed as a MIMO nonlinear system  
                         and the function of the system approximated using a radial basis function neural network  
                         (RBFNN). This is described in [182,183]. The originality of this study is in the online  
                         training of an adaptive rule to approximate a complete controller. This controller, which  
                         is RBFNN-based, offers adaptation for parametric uncertainties as well as other outside  
                         disturbances. The controller’s generated outputs include the total thrust term as well as  
                         the necessary roll and pitch angles for the attitude subsystem. Reinforcement learning  
                         (RL) has been used to successfully tackle many complex issues, and well-trained networks  
                         have surpassed human specialists in many challenging applications. Unfortunately, the  
                         majority of robotics research on reinforcement learning has mostly been limited to higher-  
                         level choices, while the state-actuator space, e.g., low-level actuator commands, has not  
                         received much attention. A UAV flight controller was trained using reinforcement learning  
                         in [183,184]. In [185], the authors proposed a neural network-based SMC method for a UAV.  
                         This control algorithm combines the advantages of SMC and neural network control to  
                         achieve improved control performance. Another paper [186] presented a neural network-  
                         based SMC method for a UAV. The proposed control approach is designed to improve  
                         the tracking performance of the system while maintaining robustness against parameter  
                         uncertainties and external disturbances. A neural network-based SMC approach for a UAV  
                         was presented in [187]. The control algorithm employs a feedforward neural network to  
                         approximate the unknown system dynamics and an SMC approach to handle parameter  
                         uncertainties and external disturbances.  
                              Another neural network-based SMC method for a UAV was presented in [188]. The  
                         control approach employs a feed-forward neural network to approximate the system  
                         dynamics and a SMC approach to handle parameter uncertainties and external disturbances.  
                         The proposed control approach was tested on a real-time experimental platform, and the  
                         experimental results demonstrated its effectiveness in controlling the UAV. In [184], a policy  
                         network was trained to use high-quality deterministic samples to translate UAV states  
                         into thrust commands. While the majority of recent studies have provided learning rules  
                         based on stochastic samples, deterministic policies use comparatively fewer samples than  
                         stochastic policies, which lowers the processing cost. In order to control the flight of a UAV,  
                         a model-based reinforcement learning policy was implemented in [189]. However, the  
                         synthesized controller does not exhibit a promising response to the step input. In [190], a  
                         neural network-based adaptive control was developed for UAV formation flight.  
                              A small amount of research on UAV model neuro-fuzzy controller systems has  
                         been published as well. Through the use of an adaptive neuro-fuzzy inference system,  
                         Bhatkhande and Havens created an intelligent neuro-fuzzy controller [163]. The developed  
                         controller was trained using traditional PD controller data. The controller’s real-time imple-  
                         mentation demonstrated acceptable control performance. Similar outcomes were reported  
                         in [191]. A UAV attitude controller was developed using type-2 fuzzy neural networks  
                         according to an advanced study [192,193]. In this study, the network-based controller was  
                         trained using the best parameter update rule. Unmanned aerial vehicle safety is crucial,  
                         particularly when they are utilized for military operations and public missions. The UAVs  
                         must be sophisticated enough to successfully mitigate any potential undesirable condition  
                         in the event of any introduced defect, whether hardware- or software-based [163,191,193].  
                         Insufficient research has been carried out on fault-tolerant control of quadrotors, and there  
                         are a number of related problems that need to be addressed thoroughly, such as preventing  
                         cyberattacks during high-profile spying missions, identifying injected flawed data, and  
                         detecting sensor spoofing and wireless communication attacks. The authors of [190,193]  
                         presented a thorough analysis of the susceptibility of autopilots to potential cyberthreats  
                         and discussed several cyberattack tactics.  
Sensors 2024, 24, 3286                                                                                          37 of 51  
  
  
  
                         Remarks  
                               Ensuring resilience and tolerance against cyberthreats such as injected false data,  
                         wireless communication attacks, and sensor spoofing, is imperative for intelligent control  
                         of UAVs. Several research teams have delved into the utilization of neural network-  
                         based observers to address the aforementioned challenges. However, this underscores  
                         the pressing need for further research aimed at developing control structures that are  
                         resistant to cyberattacks and other potential UAV hijacking scenarios. A fault-tolerant flight  
                         controller must possess the ability to keenly sense anomalies and exhibit low susceptibility  
                         to disturbances. Moreover, it should be capable of accurately estimating the magnitude  
                         and severity of faults as well as promptly and precisely detecting and diagnosing them.  
                               In the realm of UAV technology, sensor-based adaptive controls are revolutionizing  
                         capabilities, facilitating more intelligent, autonomous, and efficient operations [78]. These  
                         systems play a pivotal role in broadening the scope of applications for UAVs and represent a  
                         crucial area of focus in both current and future UAV technology development. An overview  
                         of sensor-based adaptive strategies are elaborated in Figure 13.  
  
                         6.8. Integration of Sensors into UAV Control Systems  
                              The integration of sensors into UAV control systems is a critical aspect of their opera-  
                         tion, enhancing their capabilities and allowing for a wide range of functionalities [79,80].  
                         This process involves several key elements and considerations.  
                              Types of Sensors Used in UAVs  
                         •    Navigation Sensors: GPS for location, gyroscopes for orientation and balance, ac-  
                              celerometers for speed and direction, magnetometers for heading.  
                         •    Environmental Sensors: Lidar for terrain mapping, infrared and thermal sensors for  
                              night vision or heat mapping, multispectral cameras for environmental monitoring.  
                         •    Obstacle Detection Sensors: Ultrasonic sensors, radars, optical cameras for real-time  
                              obstacle avoidance and situational awareness.  
  
  
  
  
                         Figure 13. Overview of sensor-based adaptive control of UAVs.  
  
                         Sensor Fusion  
                         •    Combining Data: Sensor fusion involves integrating data from multiple sources to  
                              create a more comprehensive understanding of the UAV’s environment. For instance,  
Sensors 2024, 24, 3286                                                                                         38 of 51  
  
  
  
                              combining GPS and IMU data can provide accurate positioning and movement infor-  
                              mation [81].  
                         •    Enhanced Decision-Making: Sensor fusion allows for more precise control and decision-  
                              making, as it mitigates the limitations of individual sensors and leverages their  
                              strengths [82,83].  
                             Control System Integration  
                         •   Feedback Loops: Sensors provide critical real-time data that feeds into the UAV’s control  
                             system, enabling it to adjust its flight path, speed, altitude, and orientation [92].  
                         •   Autonomous Operations: For autonomous UAVs, sensors are integral to their ability  
                             to navigate, perform tasks, and make decisions independently [92,93].  
                             Data Processing and Analysis  
                         •   Onboard Processing: Advanced UAVs often have onboard computers to process  
                             sensor data in real time, enabling immediate response to environmental changes.  
                         •   Algorithm Development: Developing algorithms that can efficiently and accurately  
                             process sensor data is crucial. These algorithms must be capable of handling high  
                             volumes of data from various sensors simultaneously.  
                             Calibration and Synchronization  
                         •   Sensor Calibration: Calibration ensures that sensors are accurately calibrated, which is  
                             vital for obtaining reliable data. Incorrect calibration can lead to errors in navigation  
                             and environmental interpretation [51].  
                         •   Time Synchronization: Synchronizing data from various sensors is essential, especially  
                             when combining data for decision-making [101].  
                         Energy and Resource Management  
                         •   Power Consumption: Sensors and data processing both consume power. Balancing  
                             energy consumption with operational efficiency is crucial, especially for missions  
                             requiring longer flight times [102].  
                         •   Weight and Space Constraints: The size and weight of sensors need to be considered,  
                             as they affect the UAV’s payload capacity and flight dynamics [103].  
                         Challenges and Considerations  
                         •    Environmental Factors: Sensors must be robust enough to operate in various environ-  
                              mental conditions, including weather changes, lighting variations, and temperature  
                              extremes [123].  
                         •    Reliability and Redundancy: Ensuring sensor reliability and incorporating redundancy  
                              for critical sensors is important for the safe operation of UAVs [123,124].  
                               The integration of sensors into UAV control systems is a multifaceted process that  
                         enhances UAVs’ capabilities in navigation, task execution, and environmental interaction.  
                         It involves careful consideration of sensor selection, data fusion, control integration, and  
                         resource management to ensure efficient and effective UAV operations.  
  
                         6.9. How Sensor Data Enhance UAV Control Strategies  
                             Sensor data play a pivotal role in enhancing the control strategies of Unmanned Aerial  
                         Vehicles (UAVs), enabling more sophisticated, reliable, and adaptive operations [123,125].  
                         Sensor data contribute to this enhancement in several ways.  
                         Improved Situational Awareness  
                         •    Comprehensive Environmental Understanding: Sensors such as cameras, Lidar, and  
                              radar provide UAVs with a detailed understanding of their surroundings. These  
                              data is crucial for navigation, obstacle avoidance, and mission execution in complex  
                              environments.  
                         •    Real-Time Adjustments: With real-time data, UAVs can adapt to changing environ-  
                              mental conditions such as weather changes, unforeseen obstacles, and variable terrain,  
                              thereby improving flight safety and effectiveness.  
Sensors 2024, 24, 3286                                                                                        39 of 51  
  
  
  
                         Enhanced Navigation and Positioning  
                         •    Precise Location Tracking: GPS and IMU data enable accurate positioning, which is  
                              essential for waypoint navigation and geospatial tasks [126].  
                         •    Stable Flight Control: Gyroscopes and accelerometers provide critical information  
                              about the UAV’s orientation and movement, enabling stabilization and precise ma-  
                              neuvering [127].  
                         Optimized Flight Paths and Energy Efficiency  
                         •   Efficient Route Planning: Sensor data can be used to optimize flight paths for energy  
                             efficiency, reduce battery usage, and extend mission duration [135].  
                         •   Adaptive Speed Control: By analyzing environmental data, UAVs can adjust their  
                             speed to conserve energy or avoid hazards, contributing to smarter energy manage-  
                             ment [136].  
                         Advanced Task Execution  
                         •   Target Detection and Analysis: Specialized sensors such as thermal or multispectral  
                             cameras allow UAVs to perform specific tasks such as crop monitoring, search and  
                             rescue, and infrastructure inspection with greater accuracy [136].  
                         •   Automated Payload Deployment: In applications such as agricultural spraying or  
                             package delivery, sensor data can guide precise payload deployment, enhancing the  
                             effectiveness of these operations [137].  
                         Improved Safety and Collision Avoidance  
                         •    Obstacle Detection: Ultrasonic sensors, Lidar, and optical cameras enable UAVs to  
                              detect and avoid obstacles, which is crucial for safe operation in crowded or dynamic  
                              spaces [138].  
                         •    Emergency Response: Sensor data can trigger automatic safety protocols such as  
                              return-to-home or landing procedures in response to critical situations such as battery  
                              failure or extreme weather conditions [194].  
                         Facilitation of Autonomous Operations  
                         •    Self-Guided Systems: Integration of sensor data is essential for fully autonomous  
                              UAVs. These data allow UAVs to make independent decisions about navigation and  
                              task execution and to respond to environmental changes [143].  
                         •    Machine Learning and AI Integration: Sensor data can be fed to machine learning  
                              algorithms, enabling UAVs to learn from past experiences, improve their responses,  
                              and handle complex tasks with greater autonomy [144].  
                         Enhanced Communication and Data Transmission  
                         •   Data for Ground Control: Sensor data transmitted to ground control stations provide  
                             operators with essential information for remote decision-making and intervention  
                             when necessary [145].  
                         •   Network Integration: In applications involving IoT, sensor data can be used to in-  
                             tegrate UAVs into broader networks, facilitating tasks such as data collection and  
                             monitoring across various locations [147].  
                              Sensor data can significantly enhance UAV control strategies by providing the nec-  
                         essary inputs for improved situational awareness, precise navigation, energy-efficient  
                         operations, advanced task execution, safety protocols, autonomous functionalities, and  
                         effective communication. This integration of sensor data into UAV systems is fundamental  
                         to the advancement and diversification of UAV applications.  
  
                         6.10. Examples of Sensor-Based Adaptive Control Methods in UAVs  
                               Sensor-based adaptive control methods in Unmanned Aerial Vehicles (UAVs) leverage  
                         real-time data from various onboard sensors to dynamically adjust control strategies [148].  
                         These methods enhance the ability of UAVs to operate effectively and execute complex  
                         tasks in varying conditions.  
Sensors 2024, 24, 3286                                                                                          40 of 51  
  
  
  
                         GPS-Assisted Navigation  
                         •    Waypoint Navigation: GPS data are used to guide the UAV along predefined coordi-  
                              nates while adapting its path based on real-time location data [149].  
                         •    Geofencing: GPS sensors enable UAVs to recognize and adhere to virtual boundaries,  
                              automatically adjusting their flight path to stay within designated areas [151].  
                         Vision-Based Obstacle Avoidance  
                         •    Optical Flow Sensors: When combined with cameras, these sensors allow UAVs to  
                              detect and avoid obstacles by analyzing visual data and adapting their flight path  
                              accordingly [152].  
                         •    Stereo Vision: By using two cameras to simulate 3D vision, UAVs can gauge the  
                              distance and size of obstacles, then adjust their flight to avoid collisions [153].  
                         •    Lidar for Terrain Mapping: UAVs equipped with Lidar sensors can create detailed  
                              3D maps of terrain and structures, helping to adapt their altitude and position for  
                              precise mapping [155].  
                         Inertial Measurement Unit (IMU) for Stabilization  
                         •    Gyroscopic Control: Gyroscopes in the IMU provide data for roll, pitch, and yaw  
                              stabilization, adapting control inputs to maintain stable flight [156].  
                         •    Accelerometer Data: Accelerometers aid in maintaining a steady altitude and velocity  
                              and in adjusting the UAV’s thrust and tilt in response to changes in movement [157].  
                         •    Thermal Imaging for Search and Rescue: UAVs can use thermal sensors to locate  
                              people or animals by their heat signatures, especially in low-visibility conditions, by  
                              adapting their search patterns based on thermal data [158].  
                         •    Multispectral Imaging for Precision Agriculture: These sensors enable UAVs to moni-  
                              tor crop health by capturing data in various spectral bands by adapting their flight  
                              over farmlands to identify areas needing attention [159].  
                         •    Wind Sensors for Energy-Efficient Flight: By measuring wind speed and direction,  
                              UAVs can adapt their flight path and speed for more energy-efficient routing [162,163].  
                         •    Ultrasonic Sensors for Indoor Navigation: In indoor environments or when flying  
                              close to surfaces, ultrasonic sensors can help maintain a safe distance by adapting the  
                              UAV’s altitude and position to avoid collisions [163].  
                              These sensor-based adaptive control methods demonstrate the enhanced capabilities of  
                         modern UAVs, enabling them to perform a wide range of tasks more safely and effectively.  
                         The integration of sensor data and intelligent utilization is fundamental in enhancing UAV  
                         adaptability and potential applications.  
  
                         7. Importance of Artificial Intelligence Control Techniques  
                               Hybrid control theory is a specialized area within modern control theory that focuses  
                         on systems demonstrating a combination of continuous and discrete behaviors. These  
                         systems, referred to as hybrid systems, are marked by the interaction between continuous  
                         dynamics and discrete events. Continuous dynamics refer to gradual changes in the  
                         system’s state over time, much like those seen in traditional control systems, where variables  
                         evolve smoothly; in contrast, discrete events represent abrupt changes or transitions in the  
                         system’s behavior, such as switching between different operational modes or the occurrence  
                         of events triggering sudden alterations in system dynamics [141].  
                               For instance, consider an autonomous vehicle system. The continuous dynamics might  
                         govern factors such as vehicle speed and acceleration, which change smoothly over time.  
                         However, discrete events could include actions such as switching between driving modes  
                         (e.g., from manual to autonomous) or reacting to sudden obstacles on the road [142].  
                               Hybrid control theory addresses the unique challenges posed by systems exhibiting  
                         both continuous and discrete behaviors. It focuses on understanding and effectively  
                         managing the interaction between these dynamics, enabling the design of control strategies  
                         that can handle the complexities inherent in hybrid systems [136,195].  
Sensors 2024, 24, 3286                                                                                            41 of 51  
  
  
  
                         Real Word Applications of Control Strategies  
                              In addition to theoretical advancements, the practical implementation of advanced  
                         control strategies has demonstrated significant improvements in UAV operations across  
                         various real-world applications. For instance, MPC has been successfully applied in au-  
                         tonomous surveillance missions, enabling UAVs to dynamically track and follow moving  
                         targets while avoiding obstacles in complex urban environments [196]. Feedback lineariza-  
                         tion control has proven effective in enhancing the stability and maneuverability of UAVs  
                         during high-speed flight operations, making it suitable for applications such as search and  
                         rescue missions or aerial photography [197]. Moreover, the integration of sensor-based  
                         adaptive controls and artificial intelligence-driven approaches has enabled UAVs to adapt  
                         and respond to changing environmental conditions, enhancing their versatility and ability  
                         to perform a wide range of tasks [198]. These case studies and real-world applications  
                         highlight the practical impact and potential of advanced control strategies in advancing  
                         UAV capabilities and expanding their range of applications [199]. By leveraging the insights  
                         and methodologies discussed in this review, researchers, engineers, and practitioners can  
                         further enhance the performance, autonomy, and reliability of UAV systems [200].  
  
                         8. Recent Developments and Research Directions in UAV Control Strategies  
                              In recent years, the field of UAV control has seen significant advancements driven by  
                         technological innovations and the increasing complexity of applications [171].  
  
                         8.1. Key Developments in UAV Control Strategies  
                         •    Advanced Sensor Integration: Enhanced use of sensors such as Lidar, GPS, thermal  
                              imaging, and multispectral cameras has improved UAV capabilities in navigation,  
                              obstacle avoidance, and task-specific operations [171].  
                         •    Improved Sensor Fusion Techniques: Developments in sensor fusion have allowed for  
                              more accurate and reliable interpretation of environmental data, enhancing situational  
                              awareness and decision-making [173,201].  
                         •    AI and Machine Learning Integration: The incorporation of AI and machine learning  
                              has led to smarter UAVs capable of adaptive decision-making, predictive analytics,  
                              and learning from past operations [172].  
                         •    Nonlinear and Adaptive Control Systems: To address the challenges posed by nonlin-  
                              ear dynamics and coupling effects in UAVs, recent control strategies have focused on  
                              adaptive and robust control systems that can operate effectively in a wide range of  
                              conditions [176].  
                         •    Autonomous and Collaborative Operations: Progress in autonomous control has en-  
                              abled UAVs to perform complex tasks with minimal human intervention. Developments  
                              in swarm technology have opened avenues for collaborative UAV operations [177].  
  
                         8.2. Current Research Trends and Emerging Techniques  
                         •    Energy-Efficient Control Algorithms: With the growing need for longer flight times, re-  
                              search is focusing on developing control strategies that optimize energy consumption [132].  
                         •    Enhanced Autonomy in Unstructured Environments: Researchers are exploring ways  
                              to improve UAV autonomy in complex and unstructured environments, such as dense  
                              urban areas or natural disaster sites [178].  
                         •    Human–Machine Interaction: There is increasing interest in intuitive control interfaces  
                              and systems that allow seamless human–UAV interaction, especially for applications  
                              such as search and rescue and surveillance [131,132].  
                         •    5G and IoT Integration: Leveraging 5G networks and IoT integration for real-time  
                              data transmission and enhanced connectivity is a growing research area [130,131].  
                         •    Swarm Intelligence: The study of UAV swarms for coordinated multi-UAV oper-  
                              ations offers significant potential for applications such as agricultural monitoring,  
                              environmental surveillance, and defense [128,130–132].  
Sensors 2024, 24, 3286                                                                                          42 of 51  
  
  
  
                              The advancements in UAV control strategies and the current focus of research reflect  
                         the evolving needs and potentials of UAV technology. These developments promise to  
                         enhance the capabilities, efficiency, and applicability of UAVs in various sectors.  
  
                         8.3. Challenges and Opportunities in Current Research  
                               While there are exciting prospects in UAV research, several challenges and opportuni-  
                         ties need to be addressed:  
                         •    Regulatory and Safety Concerns: Ensuring that UAV operations comply with evolv-  
                              ing regulatory frameworks and addressing safety concerns in shared airspace is a  
                              major challenge.  
                         •    Cybersecurity: As UAVs become more connected and autonomous, they face in-  
                              creasing risks due to cyberthreats. Research into secure communication and data  
                              transmission is crucial.  
                         •    Robustness in Diverse Conditions: Developing UAV control systems that are robust  
                              in a variety of environmental conditions, including adverse weather or GPS-denied  
                              environments, remains a challenge.  
                         •    Scalability of Swarm Technologies: While swarm technology is promising, scaling it  
                              for large-scale operations poses technical and logistical challenges.  
                         •    Ethical and Privacy Considerations: As UAVs become more pervasive, addressing  
                              ethical and privacy concerns is essential, especially in surveillance applications [194].  
                              The field of UAV control is rapidly evolving, with significant advancements in sensor  
                         technology, AI, and autonomous systems driving research forward. Addressing the chal-  
                         lenges in safety, regulatory compliance, cybersecurity, and ethical considerations will be  
                         key to unlocking the full potential of these technological developments.  
  
                         9. Future Challenges and Opportunities in UAV Control Strategies  
                         •    Greater Autonomy and Intelligence: Future UAV control systems are expected to  
                              exhibit higher levels of autonomy and intelligence and be capable of complex decision-  
                              making with minimal human input. Developments in AI and machine learning will en-  
                              able UAVs to learn from experiences and adapt to new situations more effectively [140].  
                         •    Advanced Swarm Coordination: There is potential for significant advancements in  
                              swarm coordination, allowing for more complex and scalable UAV operations. This  
                              includes enhanced communication systems and algorithms for real-time adaptive  
                              coordination among multiple UAVs [135].  
                         •    Robustness in Adverse Conditions: Control strategies that maintain robustness in  
                              challenging environments such as extreme weather, high electromagnetic interference,  
                              or GPS-denied areas are likely to be a key focus [134,135].  
                         •    Energy-Efficient Designs: With the increasing emphasis on sustainability, future control  
                              strategies will need to prioritize energy efficiency, including the development of algo-  
                              rithms that minimize power consumption and extend flight durations [128,130–132].  
                         •    Environmental and Wildlife Monitoring: Advanced UAVs can play a crucial role in  
                              large-scale environmental monitoring, biodiversity conservation, and management of  
                              natural resources, especially in remote and inaccessible areas [123,125].  
                         •    Urban Planning and Smart Cities: UAVs equipped with advanced control systems  
                              will be instrumental in urban planning, traffic management, and infrastructure main-  
                              tenance within smart cities [123].  
                         •    Disaster Response and Humanitarian Aid: UAVs can provide rapid real-time informa-  
                              tion and aid delivery in disaster-stricken areas, significantly improving emergency  
                              response efforts [125,127].  
                         •    Healthcare and Medical Delivery: There is potential for UAVs to be used in remote or  
                              urgent medical deliveries such as transporting medication, blood, or medical supplies  
                              to hard-to-reach areas [127].  
Sensors 2024, 24, 3286                                                                                          43 of 51  
  
  
  
                         •    Space Exploration: Advanced control systems could enable UAVs to explore extrater-  
                              restrial environments, such as Mars or other planets, where manual control is not  
                              feasible [125].  
  
                         Significance of Sensor Integration in UAVs  
                              Sensor integration plays a pivotal role in advancing the capabilities and performance  
                         of UAV control systems. Recent advancements in sensor technology have enabled signif-  
                         icant improvements in UAV navigation, stability, and autonomous operation [202]. For  
                         instance, the integration of high-precision Global Navigation Satellite Systems (GNSS)  
                         with inertial sensors has enhanced UAV positioning accuracy, enabling precise navigation  
                         and waypoint following even in GPS-denied environments. Additionally, the adoption of  
                         advanced optical and infrared cameras along with LiDAR and radar sensors has revolution-  
                         ized the sensing capabilities of UAVs for obstacle detection and avoidance. Furthermore,  
                         the incorporation of sophisticated environmental sensors such as air quality and tem-  
                         perature sensors has expanded the scope of UAV applications to include environmental  
                         monitoring and surveillance. These examples illustrate the practical impact of sensor  
                         integration on UAV control systems, demonstrating how cutting-edge sensor technologies  
                         are driving advancements in UAV performance, reliability, and autonomy. By leveraging  
                         these integrated sensor systems, UAVs are capable of executing complex missions with  
                         increased efficiency and safety, paving the way for broader adoption in various industries  
                         and applications [196,203,204].  
  
                         10. Open Research Questions and Future Research Directions  
                         1.   Human–UAV Interaction: How can interfaces for human–UAV interaction be im-  
                              proved to make them more intuitive and effective, especially for untrained users?  
                         2.   Ethical and Privacy Concerns: What are the ethical implications of widespread UAV us-  
                              age and how can privacy concerns be addressed, especially in surveillance applications?  
                         3.   Integration with Manned Aircraft: How can UAVs be safely and effectively integrated  
                              into existing airspace, which is predominantly occupied by manned aircraft?  
                         4.   Counter-UAV Systems: As UAVs become more common, what are the strategies for  
                              counter-UAV systems to prevent misuse or hostile UAV activities?  
                         5.   Cross-Domain Coordination: What are the prospects and challenges around coordi-  
                              nating UAV operations across different domains, such as air, ground, and maritime?  
                             The future of UAV control strategies is poised for significant advancements, offering a  
                         myriad of opportunities across various sectors. Addressing open research questions and  
                         overcoming challenges in integration, safety, and ethics will be crucial in realizing the full  
                         potential of UAV technology in the years to come.  
  
                         11. Conclusions  
                               In this survey, we have thoroughly examined UAV control systems, focusing on TRMS  
                         and quadrotor platforms, and have highlighted the necessity of advanced nonlinear control  
                         strategies due to system complexity. While sensor integration enhances UAV capabilities,  
                         it introduces challenges in data management. Traditional linear methods fall short, while  
                         sensor-based adaptive and AI-driven approaches excel. Looking ahead, ongoing advance-  
                         ments will shape UAV capabilities, unlocking new opportunities in complex environments.  
                         This survey bridges theory and practice, deepening the understanding of nonlinear control  
                         and the pivotal role of sensor integration in advancing UAV capabilities.  
  
                         Funding: The research work was funded by Omega Aviation Ltd. UK and the National Key R&D  
                         Program of China under Grant 2018YFB170220 (Dalian University of Technology, China).  
                         Data Availability Statement: Not applicable.  
Sensors 2024, 24, 3286                                                                                                             44 of 51  
  
  
  
                                  Acknowledgments: Li-Ying Hao at Dalian Maritime University, Dalian, China, Nokhaiz Irshad  
                                  at Bahauddin Zakariya University, Multan (Pakistan) and Jamshed Iqbal, Senior Lecturer at the  
                                  University of Hull, UK helped us to write the survey article regarding its technical aspects.  
                                  Conflicts of Interest: Authors Eric Deale Foster and Stephen Larkin was employed by the company  
                                  Omega Aviation Ltd. The remaining authors declare that the research was conducted in the absence  
                                  of any commercial or financial relationships that could be construed as a potential conflict of interest.  
  
References  
1.    Eldeeb, E.; de Souza Sant’Ana, J.M.; Pérez, D.E.; Shehab, M.; Mahmood, N.H.; Alves, H. Multi-UAV path learning for age and  
      power optimization in IoT with UAV battery recharge. IEEE Trans. Veh. Technol. 2022, 72, 5356–5360. [CrossRef]  
2.    Mahmood, A.; Vu, T.X.; Khan, W.U.; Chatzinotas, S.; Ottersten, B. Optimizing computational and communication resources for  
      MEC network empowered UAV-RIS communication. In Proceedings of the 2022 IEEE Globecom Workshops (GC Wkshps), IEEE,  
      Rio de Janeiro, Brazil, 4–8 December 2022; pp. 974–979.  
3.    Liu, C.; Ke, L. Cloud assisted Internet of things intelligent transportation system and the traffic control system in the smart city.  
      J. Control. Decis. 2023, 10, 174–187. [CrossRef]  
4.    Javaid, S.; Saeed, N.; Qadir, Z.; Fahim, H.; He, B.; Song, H.; Bilal, M. Communication and Control in Collaborative UAVs: Recent  
      Advances and Future Trends. IEEE Trans. Intell. Transp. Syst. 2023, 24, 5719–5739. [CrossRef]  
5.    Gohari, A.; Ahmad, A.B.; Rahim, R.B.A.; Supa’at, A.; Abd Razak, S.; Gismalla, M.S.M. Involvement of surveillance drones in  
      smart cities: A systematic review. IEEE Access 2022, 10, 56611–56628. [CrossRef]  
6.    Tastemirov, A.; Lecchini-Visintini, A.; Morales, R.M. Complete dynamic model of the Twin Rotor MIMO System (TRMS) with  
      experimental validation. Control Eng. Pract. 2017, 66, 89–98. [CrossRef]  
7.    Mohsan, S.A.H.; Othman, N.Q.H.; Li, Y.; Alsharif, M.H.; Khan, M.A. Unmanned aerial vehicles (UAVs): Practical aspects,  
      applications, open challenges, security issues, and future trends. Intell. Serv. Robot. 2023, 16, 109–137. [CrossRef]  
8.    Geraci, G.; Garcia-Rodriguez, A.; Giordano, L.G.; López-Pérez, D.; Björnson, E. Understanding UAV cellular communications:  
      From existing networks to massive MIMO. IEEE Access 2018, 6, 67853–67865. [CrossRef]  
9.    Li, B.; Fei, Z.; Zhang, Y. UAV communications for 5G and beyond: Recent advances and future trends. IEEE Internet Things J.  
      2018, 6, 2241–2263. [CrossRef]  
10.   Abbas, N.; Liu, X.; Iqbal, J. Robust GDI-based adaptive recursive sliding mode control (RGDI-ARSMC) for a highly nonlinear  
      MIMO system with varying dynamics of UAV. J. Mech. Sci. Technol. 2024, 38, 2015–2028. [CrossRef]  
11.   Benallegue, A.; Mokhtari, A.; Fridman, L. Feedback linearization and high order sliding mode observer for a quadrotor UAV. In  
      Proceedings of the International Workshop on Variable Structure Systems, VSS’06, IEEE, Alghero, Italy, 5–7 June 2006; pp. 365–372.  
12.   Avram, R.C.; Zhang, X.; Muse, J. Quadrotor actuator fault diagnosis and accommodation using nonlinear adaptive estimators.  
      IEEE Trans. Control. Syst. Technol. 2017, 25, 2219–2226. [CrossRef]  
13.   Wang, J.; Wang, X.; Gao, R.; Lei, C.; Feng, W.; Ge, N.; Jin, S.; Quek, T.Q.S. Physical layer security for UAV communications:  
      A comprehensive survey. China Commun. 2022, 19, 77–115. [CrossRef]  
14.   Han, S.I. Survey on UAV deployment and trajectory in wireless communication networks: Applications and challenges.  
      Information 2022, 13, 389. [CrossRef]  
15.   Sharma, A.; Vanjani, P.; Paliwal, N.; Basnayaka, C.M.W.; Jayakody, D.N.K.; Wang, H.C.; Muthuchidambaranathan, P. Communi-  
      cation and networking technologies for UAVs: A survey. J. Netw. Comput. Appl. 2020, 168, 102739. [CrossRef]  
16.   Hentati, A.I.; Fourati, L.C. Comprehensive survey of UAVs communication networks. Comput. Stand. Interfaces 2020, 72, 103451.  
      [CrossRef]  
17.   Xiao, Z.; Zhu, L.; Liu, Y.; Yi, P.; Zhang, R.; Xia, X.G.; Schober, R. A survey on millimeter-wave beamforming enabled UAV  
      communications and networking. IEEE Commun. Surv. Tutor. 2022, 24, 557–610. [CrossRef]  
18.   Geraci, G.; Garcia-Rodriguez, A.; Azari, M.M.; Lozano, A.; Mezzavilla, M.; Chatzinotas, S.; Chen, Y.; Rangan, S.; Renzo, M.D.  
      What will the future of UAV cellular communications be? A flight from 5G to 6G. IEEE Commun. Surv. Tutor. 2022, 24, 1304–1335.  
      [CrossRef]  
19.   McEnroe, P.; Wang, S.; Liyanage, M. A survey on the convergence of edge computing and AI for UAVs: Opportunities and  
      challenges. IEEE Internet Things J. 2022, 9, 15435–15459. [CrossRef]  
20.   Nomikos, N.; Gkonis, P.K.; Bithas, P.S.; Trakadas, P. A survey on UAV-aided maritime communications: Deployment considera-  
      tions, applications, and future challenges. IEEE Open J. Commun. Soc. 2023, 4, 56–78. [CrossRef]  
21.   Jasim, M.A.; Shakhatreh, H.; Siasi, N.; Sawalmeh, A.H.; Aldalbahi, A.; Al-Fuqaha, A. A survey on spectrum management for  
      unmanned aerial vehicles (UAVs). IEEE Access 2022, 10, 11443–11499. [CrossRef]  
22.   Xu, C.; Liao, X.; Tan, J.; Ye, H.; Lu, H. Recent research progress of unmanned aerial vehicle regulation policies and technologies in  
      urban low altitude. IEEE Access 2020, 8, 74175–74194. [CrossRef]  
23.   Hafeez, S.; Khan, A.R.; Al-Quraan, M.M.; Mohjazi, L.; Zoha, A.; Imran, M.A.; Sun, Y. Blockchain-assisted UAV communication  
      systems: A comprehensive survey. IEEE Open J. Veh. Technol. 2023, 4, 558–580. [CrossRef]  
24.   Wei, Z.; Zhu, M.; Zhang, N.; Wang, L.; Zou, Y.; Meng, Z.; Wu, H.; Feng, Z. UAV-assisted data collection for Internet of Things:  
      A survey. IEEE Internet Things J. 2022, 9, 15460–15483. [CrossRef]  
Sensors 2024, 24, 3286                                                                                                               45 of 51  
  
  
  
25.   Duong, T.Q.; Kim, K.J.; Kaleem, Z.; Bui, M.P.; Vo, N.S. UAV caching in 6G networks: A survey on models, techniques, and  
      applications. Phys. Commun. 2022, 51, 101532. [CrossRef]  
26.   Wei, H.; Lou, B.; Zhang, Z.; Liang, B.; Wang, F.Y.; Lv, C. Autonomous Navigation for eVTOL: Review and Future Perspectives.  
      IEEE Trans. Intell. Veh. 2024, 9, 4145–4171. [CrossRef]  
27.   Thantharate, P.; Thantharate, A.; Kulkarni, A. GREENSKY: A fair energy-aware optimization model for UAVs in next-generation  
      wireless networks. Green Energy Intell. Transp. 2024, 3, 100130. [CrossRef]  
28.   Aggarwal, S.; Kumar, N. Path planning techniques for unmanned aerial vehicles: A review, solutions, and challenges. Comput.  
      Commun. 2020, 149, 270–299. [CrossRef]  
29.   Iqbal, J.; Ullah, M.; Khan, S.G.; Khelifa, B.; Ćuković, S. Nonlinear control systems-A brief overview of historical and recent  
      advances. Nonlinear Eng. 2017, 6, 301–312. [CrossRef]  
30.   Pan, E.; Xu, H.; Yuan, H.; Peng, J.; Xu, W. HIT-Hawk and HIT-Phoenix: Two kinds of flapping-wing flying robotic birds with  
      wingspans beyond 2 meters. Biomim. Intell. Robot. 2021, 1, 100002. [CrossRef]  
31.   Geranmehr, B.; Khanmirza, E.; Kazemi, S. Trajectory control of aggressive maneuver by agile autonomous helicopter. Proc. Inst.  
      Mech. Eng. Part G J. Aerosp. Eng. 2019, 233, 1526–1536. [CrossRef]  
32.   Puttannaiah, K. A Generalized H-Infinity Mixed Sensitivity Convex Approach to Multivariable Control Design Subject to Simultaneous  
      Output and Input Loop-Breaking Specifications; Technical Report; Arizona State University: Tempe, AZ, USA, 2018.  
33.   Shah, S.H.; Khan, S.G.; Iqbal, J.; Alharthi, M. Modeling and robust control of twin rotor mimo system. In Proceedings of the 2019  
      International Conference on Robotics and Automation in Industry (ICRAI), IEEE, Rawalpindi, Pakistan, 21–22 October 2019;  
      pp. 1–5.  
34.   Abbas, N.; Pan, X.; Raheem, A.; Shakoor, R.; Arfeen, Z.A.; Rashid, M.; Umer, F.; Safdar, N.; Liu, X. Real-time robust generalized  
      dynamic inversion based optimization control for coupled twin rotor MIMO system. Sci. Rep. 2022, 12, 1–17.  
35.   Chalupa, P.; Přikryl, J.; Novák, J. Modelling of twin rotor MIMO system. Procedia Eng. 2015, 100, 249–258. [CrossRef]  
36.   Abbas, N.; Liu, X. A mixed dynamic optimization with µ-synthesis (DK iterations) via gain scheduling for varying dynamics of  
      decoupled twin-rotor MIMO system based on the method of inequality (MOI). J. Control. Eng. Appl. Inform. 2022, 24, 13–23.  
37.   Ahmed, F.; Mohanta, J.; Keshari, A.; Yadav, P.S. Recent Advances in Unmanned Aerial Vehicles: A Review. Arab. J. Sci. Eng. 2022,  
      47, 7963–7984. [CrossRef] [PubMed]  
38.   Bucolo, M.; Buscarino, A.; Fortuna, L.; Gagliano, S. Bifurcation scenarios for pilot induced oscillations. Aerosp. Sci. Technol. 2020,  
      106, 106194. [CrossRef]  
39.   Orozco, R.G. Mixed sensitivity control: a non-iterative approach. Syst. Sci. Control Eng. 2020, 8, 441–453. [CrossRef]  
40.   Chaudhary, S.; Kumar, A. Control of Twin Rotor MIMO system using 1-degree-of-freedom PID, 2-degree-of-freedom PID and  
      fractional order PID controller. In Proceedings of the 2019 3rd International Conference on Electronics, Communication and  
      Aerospace Technology (ICECA), IEEE, Coimbatore, India, 12–14 June 2019; pp. 746–751.  
41.   Mechali, O.; Xu, L.; Xie, X.; Iqbal, J. Theory and practice for autonomous formation flight of quadrotors via distributed robust  
      sliding mode control protocol with fixed-time stability guarantee. Control Eng. Pract. 2022, 123, 105150. [CrossRef]  
42.   Ramalakshmi, A.; Manoharan, P. Non-linear modeling and PID control of twin rotor MIMO system. In Proceedings of the  
      2012 IEEE International Conference on Advanced Communication Control and Computing Technologies (ICACCCT), IEEE,  
      Ramanathapuram, India, 23–25 August 2012; pp. 366–369.  
43.   Dube, D.Y.; Patel, H.G. Suppressing the noise in measured signals for the control of helicopters. Fluct. Noise Lett. 2019, 18, 1950002.  
      [CrossRef]  
44.   Liu, H.; Xi, J.; Zhong, Y. Robust attitude stabilization for nonlinear quadrotor systems with uncertainties and delays. IEEE Trans.  
      Ind. Electron. 2017, 64, 5585–5594. [CrossRef]  
45.   Mofid, O.; Mobayen, S. Adaptive sliding mode control for finite-time stability of quad-rotor UAVs with parametric uncertainties.  
      ISA Trans. 2018, 72, 1–14. [CrossRef]  
46.   Hayat, S.; Yanmaz, E.; Muzaffar, R. Survey on unmanned aerial vehicle networks for civil applications: A communications  
      viewpoint. IEEE Commun. Surv. Tutor. 2016, 18, 2624–2661. [CrossRef]  
47.   Ebeid, E.; Skriver, M.; Jin, J. A survey on open-source flight control platforms of unmanned aerial vehicle. In Proceedings of the  
      2017 Euromicro Conference on Digital System Design (dsd), IEEE, Vienna, Austria, 30 August–1 September 2017; pp. 396–402.  
48.   Fotouhi, A.; Qiang, H.; Ding, M.; Hassan, M.; Giordano, L.G.; Garcia-Rodriguez, A.; Yuan, J. Survey on UAV cellular communica-  
      tions: Practical aspects, standardization advancements, regulation, and security challenges. IEEE Commun. Surv. Tutor. 2019,  
      21, 3417–3442. [CrossRef]  
49.   Mozaffari, M.; Saad, W.; Bennis, M.; Nam, Y.H.; Debbah, M. A tutorial on UAVs for wireless networks: Applications, challenges,  
      and open problems. IEEE Commun. Surv. Tutor. 2019, 21, 2334–2360. [CrossRef]  
50.   Zhang, L.; Zhao, H.; Hou, S.; Zhao, Z.; Xu, H.; Wu, X.; Wu, Q.; Zhang, R. A survey on 5G millimeter wave communications for  
      UAV-assisted wireless networks. IEEE Access 2019, 7, 117460–117504. [CrossRef]  
51.   Mo, H.; Farid, G. Nonlinear and adaptive intelligent control techniques for quadrotor uav—A survey. Asian J. Control. 2019,  
      21, 989–1008. [CrossRef]  
52.   Abbas, N.; Liu, X.; Iqbal, J. A flexible mixed-optimization with H∞ control for coupled twin rotor MIMO system based on the  
      method of inequality (MOI)-An experimental study. PLoS ONE 2024, 19, e0300305. [CrossRef] [PubMed]  
Sensors 2024, 24, 3286                                                                                                             46 of 51  
  
  
  
53.   Lin, C.; Wang, X.; Zhang, Q. Feedback Linearization Control of Twin Rotor MIMO System. In Proceedings of the 2020 IEEE  
      International Conference on Mechatronics and Automation (ICMA), IEEE, Beijing, China from 2–5 August 2020; pp. 764–769.  
54.   Ullah, Z.; Al-Turjman, F.; Mostarda, L. Cognition in UAV-aided 5G and beyond communications: A survey. IEEE Trans. Cogn.  
      Commun. Netw. 2020, 6, 872–891. [CrossRef]  
55.   Oubbati, O.S.; Atiquzzaman, M.; Ahanger, T.A.; Ibrahim, A. Softwarization of UAV networks: A survey of applications and  
      future trends. IEEE Access 2020, 8, 98073–98125. [CrossRef]  
56.   Zhi, Y.; Fu, Z.; Sun, X.; Yu, J. Security and privacy issues of UAV: a survey. Mob. Netw. Appl. 2020, 25, 95–101. [CrossRef]  
57.   Skorobogatov, G.; Barrado, C.; Salamí, E. Multiple UAV systems: A survey. Unmanned Syst. 2020, 8, 149–169. [CrossRef]  
58.   Jiang, X.; Sheng, M.; Nan, Z.; Chengwen, X.; Weidang, L.; Xianbin, W. Green UAV communications for 6G: A survey. Chin. J.  
      Aeronaut. 2022, 35, 19–34. [CrossRef]  
59.   Song, Q.; Zeng, Y.; Xu, J.; Jin, S. A survey of prototype and experiment for UAV communications. Sci. China Inf. Sci. 2021, 64, 1–21.  
      [CrossRef]  
60.   Srivastava, S.; Narayan, S.; Mittal, S. A survey of deep learning techniques for vehicle detection from UAV images. J. Syst. Archit.  
      2021, 117, 102152. [CrossRef]  
61.   Haider, S.K.; Nauman, A.; Jamshed, M.A.; Jiang, A.; Batool, S.; Kim, S.W. Internet of drones: Routing algorithms, techniques and  
      challenges. Mathematics 2022, 10, 1488. [CrossRef]  
62.   Poudel, S.; Moh, S. Task assignment algorithms for unmanned aerial vehicle networks: A comprehensive survey. Veh. Commun.  
      2022, 35, 100469.  
63.   Bennett, S. A History of Control Engineering 1800–1930: Peter Peregrinus; Stevenage, UK, 1979; Volume 10. Available online:  
      https://digital-library.theiet.org/content/books/ce/pbce008e (accessed on 11 May 2024).  
64.   Bennett, S. A History of Control Engineering, 1930–1955; Number 47; IET: Wales, UK, 1993.  
65.   Willems, J.C. Dissipative dynamical systems part I: General theory. Arch. Ration. Mech. Anal. 1972, 45, 321–351. [CrossRef]  
66.   Sontag, E.D.; Wang, Y. On characterizations of the input-to-state stability property. Syst. Control. Lett. 1995, 24, 351–359.  
      [CrossRef]  
67.   Isidori, A.; van Schuppen, J.; Sontag, E.; Thoma, M.; Krstic, M. Communications and control engineering. In Nonlinear Control  
      Systems; Springer: Berlin/Heidelberg, Germany, 1995.  
68.   Kokotović, P.; Arcak, M. Constructive nonlinear control: A historical perspective. Automatica 2001, 37, 637–662.  
69.   Poincaré, H. Science and Hypothesis, Translated by G. Halsted; Netherland, 1905. Available online: https://www.bol.com/nl/nl/f/  
      science-and-hypothesis/9200000023783187/ (accessed on 11 May 2024).  
70.   Kalman, R.E.; Bertram, J.E. Control system analysis and design via the “second method” of Lyapunov: I—Continuous-time  
      systems. J. Basic Eng. 1960, 82, 371–393. [CrossRef]  
71.   Krstic, M.; Kanellakopoulos, I.; Kokotovic, P.V. Nonlinear and Adaptive Control Design; John Wiley & Sons: Hoboken, NJ,  
      USA, 2008.  
72.   Åström, K.J.; Wittenmark, B. Adaptive Control; Dover Publications: Garden, NY, USA, 2008.  
73.   Kiumarsi, B.; Ioannou, P.A. Nonlinear Control Systems Using Deep Learning Techniques. arXiv 2018, arXiv:1809.08079.  
74.   Rawlings, J.B.; Mayne, D.Q. Model Predictive Control: Theory, Computation, and Design; Nob Hill Publishing: Madison, WI,  
      USA, 2017.  
75.   Antsaklis, P.J.; Wonham, W.M. Linear Systems Theory: An Introduction; Springer: Berlin/Heidelberg, Germany, 2001.  
76.   Li, T.; Wen, B.; Tian, Y.; Wang, S.; Yin, Y. Optimized radar waveform parameter design for small drone detection based on echo  
      modeling and experimental analysis. IEEE Access 2019, 7, 101527–101538. [CrossRef]  
77.   Qian, H.; Fang, Y. First-Order Sliding Mode Control of Twin Rotor MIMO System with Nonlinear Disturbance Observer. IEEE  
      Trans. Ind. Electron. 2020, 67, 7861–7870. [CrossRef]  
78.   Tong, S.; He, X.; Li, Y.; Zhang, H. Adaptive fuzzy backstepping robust control for uncertain nonlinear systems based on small-gain  
      approach. Fuzzy Sets Syst. 2010, 161, 771–796. [CrossRef]  
79.   Pratap, B.; Purwar, S. Neural network observer for twin rotor MIMO system: an LMI based approach. In Proceedings of the  
      Proceedings of the 2010 International Conference on Modelling, Identification and Control, IEEE, Okayama, Japan, 17–19 July  
      2010; pp. 539–544.  
80.   Liu, Y.J.; Zhao, W.; Liu, L.; Li, D.; Tong, S.; Chen, C.P. Adaptive neural network control for a class of nonlinear systems with  
      function constraints on states. IEEE Trans. Neural Netw. Learn. Syst. 2021, 34, 2732–2741. [CrossRef] [PubMed]  
81.   Rashad, R.; El-Badawy, A.; Aboudonia, A. Sliding mode disturbance observer-based control of a twin rotor MIMO system. ISA  
      Trans. 2017, 69, 166–174. [CrossRef] [PubMed]  
82.   Liu, W.; Hu, Y.; Lin, C. Robust Control of Twin Rotor MIMO System with First-Order Sliding Mode Control and Disturbance  
      Observer. IEEE Trans. Ind. Electron. 2021, 68, 4425–4434. [CrossRef]  
83.   Ullah, N.; Mehmood, Y.; Aslam, J.; Ali, A.; Iqbal, J. UAVs-UGV leader follower formation using adaptive non-singular terminal  
      super twisting sliding mode control. IEEE Access 2021, 9, 74385–74405. [CrossRef]  
84.   Rojas-Cubides, H.; Cortés-Romero, J.; Coral-Enriquez, H.; Rojas-Cubides, H. Sliding mode control assisted by GPI observers for  
      tracking tasks of a nonlinear multivariable Twin-Rotor aerodynamical system. Control Eng. Pract. 2019, 88, 1–15. [CrossRef]  
Sensors 2024, 24, 3286                                                                                                            47 of 51  
  
  
  
85.  Saroj, D.K.; Kar, I.; Pandey, V.K. Sliding mode controller design for Twin Rotor MIMO system with a nonlinear state observer. In  
     Proceedings of the 2013 International Mutli-Conference on Automation, Computing, Communication, Control and Compressed  
     Sensing (iMac4s), IEEE, Kottayam, India, 22–23 March 2013; pp. 668–673.  
86. Ilyas, M.; Abbas, N.; UbaidUllah, M.; Imtiaz, W.A.; Shah, M.; Mahmood, K. Control law design for twin rotor MIMO system with  
     nonlinear control strategy. Discret. Dyn. Nat. Soc. 2016, 2016, 2952738. [CrossRef]  
87. Zhang, F.; Li, Q.; Huang, W. First-Order Sliding Mode Control of Twin Rotor MIMO System with Parameter Uncertainties. IEEE  
     Access 2020, 8, 51685–51696. [CrossRef]  
88. Mechali, O.; Iqbal, J.; Mechali, A.; Xie, X.; Xu, L. Finite-time attitude control of uncertain quadrotor aircraft via continuous  
     terminal sliding-mode-based active anti-disturbance approach. In Proceedings of the 2021 IEEE International Conference on  
     Mechatronics and Automation (ICMA), IEEE, Takamatsu, Japan, 8–11 August 2021; pp. 1170–1175.  
89. Zhang, N.; Wang, H.; Zhang, C. Terminal sliding mode control of twin rotor MIMO system based on adaptive fuzzy sliding  
     mode observer. Int. J. Control Autom. Syst. 2021, 19, 1–14. [CrossRef]  
90. Guo, X.; Gu, X. Robust control for a twin rotor MIMO system based on first-order sliding mode control and RBF neural network.  
     J. Intell. Fuzzy Syst. 2020, 39, 4849–4859. [CrossRef]  
91. Kumar, A.D.; Kumar, D.L.; Nair, M.N.M. Sliding mode control for twin rotor MIMO system: A fractional order approach. ISA  
     Trans. 2021, 110, 50–59. [CrossRef]  
92. Sun, T.; Zhang, X.; Zhao, J. Adaptive fuzzy sliding mode control for a class of uncertain nonlinear systems with time-varying  
     delay. IEEE Access 2021, 9, 34437–34448. [CrossRef]  
93. Arriaga, S.S.; Davila, J.C.; Sira-Ramirez, H. First-order sliding-mode control for affine nonlinear systems with disturbances. IET  
     Control Theory Appl. 2020, 14, 653–662. [CrossRef]  
94. Zhang, M.; Liu, Y.; Liu, C. Robust Fault-Tolerant Control of Hypersonic Vehicle with Control Allocation. J. Aerosp. Eng. 2021, 34.  
     [CrossRef]  
95. Kumar, R.; Srivastava, S.; Gupta, J. Diagonal recurrent neural network based adaptive control of nonlinear dynamical systems  
     using lyapunov stability criterion. ISA Trans. 2017, 67, 407–427. [CrossRef] [PubMed]  
96. Mechali, O.; Iqbal, J.; Wang, J.; Xie, X.; Xu, L. Distributed leader-follower formation control of quadrotors swarm subjected  
     to disturbances. In Proceedings of the 2021 IEEE International Conference on Mechatronics and Automation (ICMA), IEEE,  
     Takamatsu, Japan, 8–11 August 2021; pp. 1442–1447.  
97. Labbadi, M.; Iqbal, J.; Djemai, M.; Boukal, Y.; Bouteraa, Y. Robust tracking control for a quadrotor subjected to disturbances using  
     new hyperplane-based fast Terminal Sliding Mode. PLoS ONE 2023, 18, e0283195. [CrossRef] [PubMed]  
98. Choi, K.S.; Won, S.K. Robust Nonlinear Control for MIMO Systems With Chattering Suppression. IEEE Trans. Control Syst.  
     Technol. 2021, 29, 1634–1641. [CrossRef]  
99. Zhao, R.; Zhang, Y.; Xie, L. Chattering-free sliding mode control for MIMO nonlinear systems with mismatched uncertainties.  
     IET Control Theory Appl. 2021, 15, 63–71. [CrossRef]  
100. Shamsi, M.; Hoseini, S.M. New chattering-free control design for MIMO systems with uncertain dynamics. ISA Trans. 2021,  
     116, 13–24. [CrossRef]  
101. Wang, R.; Li, Y.; Hu, Z. Adaptive sliding mode control of a class of uncertain nonlinear systems using radial basis function neural  
     network. J. Frankl. Inst. 2022, 359, 1427–1450. [CrossRef]  
102. Zhao, Y.; Chen, X.; Chen, B.; Cui, R. Sliding mode control of switched linear systems with time-varying delay and actuator faults.  
     Nonlinear Dyn. 2021, 103, 1039–1058. [CrossRef]  
103. Liu, X.; Li, X.; Liu, P.; Li, W. Backstepping control of a quadrotor UAV based on adaptive sliding mode control. J. Frankl. Inst.  
     2019, 356, 3719–3736.  
104. Alkathiri, A.A.; Zhang, X.; Liu, H.; Pan, Y. Backstepping controller for fixed-wing unmanned aerial vehicle altitude control.  
     J. Intell. Robot. Syst. 2020, 98, 99–112.  
105. Haruna, A.; Mohamed, Z.; Efe, M.Ö.; Basri, M.A.M. Dual boundary conditional integral backstepping control of a twin rotor  
     MIMO system. J. Frankl. Inst. 2017, 354, 6831–6854. [CrossRef]  
106. Zhang, Z.; Wu, H.; Zhang, M. Feedback Linearization Control for Twin Rotor MIMO System Based on Neural Network. Electronics  
     2021, 10, 682.  
107. Fritsch, O.; De Monte, P.; Buhl, M.; Lohmann, B. Quasi-static feedback linearization for the translational dynamics of a quadrotor  
     helicopter. In Proceedings of the 2012 American Control Conference (ACC), IEEE, Montreal, QC, Canada, 27–29 June 2012;  
     pp. 125–130.  
108. Lee, D.; Jin Kim, H.; Sastry, S. Feedback linearization vs. adaptive sliding mode control for a quadrotor helicopter. Int. J. Control  
     Autom. Syst. 2009, 7, 419–428. [CrossRef]  
109. Lin, Q.; Cai, Z.; Wang, Y.; Yang, J.; Chen, L. Adaptive flight control design for quadrotor UAV based on dynamic inversion  
     and neural networks. In Proceedings of the 2013 Third International Conference on Instrumentation, Measurement, Computer,  
     Communication and Control, IEEE, Shenyang, China, 21–23 September 2013; pp. 1461–1466.  
110. Mukherjee, P.; Waslander, S. Direct adaptive feedback linearization for quadrotor control. In Proceedings of the AIAA Guidance,  
     Navigation, and Control Conference, Minneapolis, MN, USA, 13–16 August 2012; p. 4917.  
111. Fang, Z.; Zhi, Z.; Jun, L.; Jian, W. Feedback linearization and continuous sliding mode control for a quadrotor UAV. In Proceedings  
     of the 2008 27th Chinese Control Conference, IEEE, Kunming, China, 16–18 July 2008, pp. 349–353.  
Sensors 2024, 24, 3286                                                                                                             48 of 51  
  
  
  
112. Zhou, Q.L.; Zhang, Y.; Rabbath, C.A.; Theilliol, D. Design of feedback linearization control and reconfigurable control allocation  
     with application to a quadrotor UAV. In Proceedings of the 2010 Conference on Control and Fault-Tolerant Systems (SysTol),  
     IEEE, Nice, France, 6–8 October 2010; pp. 371–376.  
113. Choi, Y.C.; Ahn, H.S. Nonlinear control of quadrotor for point tracking: Actual implementation and experimental tests.  
     IEEE/ASME Trans. Mechatron. 2014, 20, 1179–1192. [CrossRef]  
114. Isidori, A.; Isidori, A. Local decompositions of control systems. In Nonlinear Control Systems; Springer: Berlin/Heidelberg,  
     Germany, 1995; pp. 1–76.  
115. Wang, H.; Liu, J.; Zhou, X.; Wang, L. Robust Feedback Linearization Control for Twin Rotor MIMO System Using Sliding Mode  
     Control. Int. J. Control Autom. Syst. 2019, 17, 612–621.  
116. Krener, A. Feedback linearization. In Mathematical Control Theory; Springer: Berlin/Heidelberg, Germany, 1999; pp. 66–98.  
117. Sarma, P.; Bandyopadhyay, B. Nonlinear Zero Dynamics in Control Systems.                      In CONTROL SYSTEMS, ROBOTICS  
     AND AUTOMATION–Volume XIII: Nonlinear, Distributed, and Time Delay Systems-II; 2009; p. 24. Available online: https:  
     //books.google.com.hk/books?hl=en&lr=&id=OlN3DAAAQBAJ&oi=fnd&pg=PA24&dq=Sarma,+P.%3B+Bandyopadhyay,  
     +B.+Nonlinear+Zero+Dynamics+in+Control+Systems.&ots=OPEpgnInDM&sig=Ei8tKfAg45EPWcPCRL-f24yDNlM&redir_  
     esc=y#v=onepage&q=Sarma%2C%20P.%3B%20Bandyopadhyay%2C%20B.%20Nonlinear%20Zero%20Dynamics%20in%20  
     Control%20Systems.&f=false (accessed on 11 May 2024).  
118. Rudolph, J.; Delaleau, E. Some examples and remarks on quasi-static feedback of generalized states. Automatica 1998, 34, 993–999.  
     [CrossRef]  
119. Mistler, V.; Benallegue, A.; M’sirdi, N. Exact linearization and noninteracting control of a 4 rotors helicopter via dynamic feedback.  
     In Proceedings of the Proceedings 10th IEEE International Workshop on Robot and Human Interactive Communication. Roman  
     2001 (Cat. no. 01th8591), IEEE, Bordeaux, Paris, France, 18–21 September 2001; pp. 586–593.  
120. Wang, J.; Zhang, H.; Liu, Z. Adaptive Feedback Linearization Control for Twin Rotor MIMO System. In Proceedings of the 2020  
     25th International Conference on Mechatronics and Machine Vision in Practice (M2VIP), IEEE, Kenneth Ku, Robin Bradbeer, 2020;  
     pp. 1–5.  
121. Voos, H. Nonlinear control of a quadrotor micro-UAV using feedback-linearization. In Proceedings of the 2009 IEEE International  
     Conference on Mechatronics, IEEE, Malaga, Spain, 14–17 April 2009; pp. 1–6.  
122. Altug, E.; Ostrowski, J.P.; Mahony, R. Control of a quadrotor helicopter using visual feedback. In Proceedings of the Proceedings  
     2002 IEEE international conference on robotics and automation (Cat. No. 02CH37292), IEEE, Washington, DC, USA, 11–15 May  
     2002; Volume 1, pp. 72–77.  
123. Yang, J.; Liu, W.; Zhan, Z. Feedback Linearization Control of Twin Rotor MIMO System Based on Neural Networks. In  
     Proceedings of the 2019 9th International Conference on Information Science and Technology (ICIST), IEEE, Hulunbuir, China,  
     2–5 August 2019; pp. 379–383.  
124. Mokhtari, A.; Benallegue, A. Dynamic feedback controller of Euler angles and wind parameters estimation for a quadrotor  
     unmanned aerial vehicle. In Proceedings of the IEEE International Conference on Robotics and Automation, 2004. Proceedings.  
     ICRA’04. 2004, IEEE, New Orleans, LA, USA, 26 April–1 May 2004; Volume 3, pp. 2359–2366.  
125. Zhang, K.; Li, J. Adaptive Backstepping Feedback Linearization Control for Twin Rotor MIMO System. Control Eng. Pract. 2018,  
     79, 94–105.  
126. Liu, J.; Zhang, C.; Xu, T. Decentralized Feedback Linearization Control for Twin Rotor MIMO System. In Proceedings of the 2018  
     37th Chinese Control Conference (CCC), IEEE, Wuhan, China, 25–27 July 2018; pp. 7196–7200.  
127. Zhang, H.; Wang, J.; Liu, Z. Nonlinear Control of Twin Rotor MIMO System Using Feedback Linearization. In Proceedings of the  
     2020 IEEE 11th International Conference on Mechanical and Intelligent Manufacturing Technologies (ICMIMT), IEEE, Cape Town,  
     South Africa, 20–22 January 2020; pp. 314–319.  
128. Ma, Q.; Zhang, J.; Wang, M.; Hu, Z. Model Predictive Control for Twin Rotor MIMO System Based on Reinforcement Learning. In  
     Proceedings of the 2020 Chinese Control And Decision Conference (CCDC), IEEE, Hefei, China, 22–24 August 2020; pp. 4707–4712.  
129. Yan, C.; Wei, P. Nonlinear Model Predictive Control for Twin Rotor MIMO System with Disturbance Rejection. Control Eng. Pract.  
     2017, 64, 44–55.  
130. Köhler, J.; Kötting, P.; Soloperto, R.; Allgöwer, F.; Müller, M.A. A robust adaptive model predictive control framework for  
     nonlinear uncertain systems. Int. J. Robust Nonlinear Control 2021, 31, 8725–8749. [CrossRef]  
131. Mehndiratta, M.; Kayacan, E.; Patel, S.; Kayacan, E.; Chowdhary, G. Learning-based fast nonlinear model predictive control for  
     custom-made 3D printed ground and aerial robots. In Handbook of Model Predictive Control; Springer: Berlin/Heidelberg, Germany,  
     2019; pp. 581–605.  
132. Jia, C.; Wang, X.; Liang, Y.; Zhou, K. Robust current controller for IPMSM drives based on explicit model predictive control with  
     online disturbance observer. IEEE Access 2019, 7, 45898–45910. [CrossRef]  
133. Akpan, V.A.; Hassapis, G.D. Nonlinear model identification and adaptive model predictive control using neural networks. ISA  
     Trans. 2011, 50, 177–194. [CrossRef] [PubMed]  
134. Li, W.; Hu, Y.; Zhang, J.; Tang, Z.; Guo, J. Hybrid Model Predictive Control for Twin Rotor MIMO System. In Proceedings of the  
     2020 15th IEEE Conference on Industrial Electronics and Applications (ICIEA). IEEE, Kristiansand, Norway, 9–13 November 2020;  
     pp. 897–902.  
Sensors 2024, 24, 3286                                                                                                         49 of 51  
  
  
  
135. Wang, L.; Liu, J.; Zhou, X. Model Predictive Control with Sliding Mode Disturbance Observer for Twin Rotor MIMO System.  
     IEEE Trans. Ind. Electron. 2021, 68, 3549–3558.  
136. Bernasconi, P.; Margellos, K. Importance of Hybrid Control in Complex Systems. Annu. Rev. Control 2015, 39, 75–88.  
137. Goebel, K.; Teel, A.R.; Annaswamy, A.M. Hybrid Control Systems: Importance, Challenges, and Applications. Proc. IEEE 2012,  
     100, 29–47.  
138. Stewart, T.; Clarke, J.P. The Importance of Hybrid Control for Autonomous Systems. In Proceedings of the 2015 European  
     Control Conference (ECC), IEEE, Linz, Austria, 15–17 July 2015; pp. 2375–2380.  
139. Cassandras, C.G.; Lafortune, S. Reachability Analysis for Hybrid Systems: Theory and Methods; Springer Science & Business Media:  
     Berlin/Heidelberg, Germany, 2008.  
140. Mirafzal, B.; Ramasamy, S.; Ngo, K. Importance of Hybrid Control Strategies in Power Electronic Systems. IEEE Trans. Power  
     Electron. 2018, 33, 4731–4748.  
141. Lafortune, S. Reachability Analysis of Hybrid Systems with Linear Guard Conditions. In Proceedings of the 2008 American  
     Control Conference, IEEE, Seattle, WA, USA, 11–13 June 2008; pp. 1148–1153.  
142. Wang, L.; Shi, P.; Liu, G.P. Reachability Analysis for Switched Linear Systems with Time-Varying Delays. Automatica 2008,  
     44, 2043–2050.  
143. Li, C.; Sun, Y.; Liu, Y.; Wang, Y. A novel adaptive sliding mode control for a class of nonlinear MIMO systems with reduced-order  
     disturbance observer. IEEE Trans. Control Syst. Technol. 2022, 30, 116–128. [CrossRef]  
144. Lee, S.; Kang, C.; Park, S.; Kim, S. Adaptive Sliding Mode Control of a Multi-Link Flexible Manipulator System using FPGA.  
     IEEE Access 2021, 9, 47320–47330. [CrossRef]  
145. Dinh, T.; Trinh, Q.; Vo, D.; Nguyen, H. Adaptive sliding mode control of quadrotor unmanned aerial vehicle using backstepping  
     control and extended Kalman filter. Int. J. Control Autom. Syst. 2020, 18, 2067–2081. [CrossRef]  
146. Zhao, Y.; Xia, Y.; Xu, Y.; Zhang, J. Adaptive sliding mode control of a twin rotor MIMO system. In Proceedings of the 2017 IEEE  
     International Conference on Mechatronics and Automation (ICMA), IEEE, Takamatsu, Japan, 6–9 August 2017; pp. 1589–1594.  
     [CrossRef]  
147. Ma, S.; Hu, Y.; Liu, Y.; Gao, H. Adaptive sliding mode control for a twin rotor MIMO system with parameter uncertainties.  
     J. Control Sci. Eng. 2018, 2018, 1–10. [CrossRef]  
148. Yi, S.; Zhai, J. Adaptive second-order fast nonsingular terminal sliding mode control for robotic manipulators. ISA Trans. 2019,  
     90, 41–51. [CrossRef]  
149. Min, J.; Fang, Y.; Xu, Z. Adaptive fast terminal sliding mode control for a class of uncertain system. In Proceedings of the 2009  
     International Conference on Industrial and Information Systems, IEEE, Haikou, China, 24–25 April 2009; pp. 337–340.  
150. Nojavanzadeh, D.; Badamchizadeh, M. Adaptive fractional-order non-singular fast terminal sliding mode control for robot  
     manipulators. IET Control Theory Appl. 2016, 10, 1565–1572. [CrossRef]  
151. Nguyen, V.C.; Vo, A.T.; Kang, H.J. A finite-time fault-tolerant control using non-singular fast terminal sliding mode control and  
     third-order sliding mode observer for robotic manipulators. IEEE Access 2021, 9, 31225–31235. [CrossRef]  
152. Pan, H.; Zhang, G. Novel nonsingular fast terminal sliding mode control for a class of second-order uncertain nonlinear systems.  
     Math. Probl. Eng. 2021, 2021, 1–18. [CrossRef]  
153. Xu, Y.; Zhang, L.; Liu, H. Adaptive Backstepping Control for Twin Rotor MIMO System with Parametric Uncertainties and  
     External Disturbances. IEEE Access 2021, 9, 20933–20943.  
154. Li, W.; Li, J.; Fan, J. Adaptive Backstepping Control of a Twin-Rotor MIMO System with Input Saturation and External  
     Disturbances. IEEE Access 2019, 7, 69267–69276.  
155. Leng, Y.; Ren, L.; Yang, J. Adaptive Backstepping Control of Twin-Rotor MIMO System with Actuator Saturation and External  
     Disturbances. J. Appl. Math. 2020, 2020.  
156. Hu, H.; Qiao, J. Adaptive backstepping fast terminal sliding mode control for a class of uncertain nonlinear MIMO systems.  
     Nonlinear Dyn. 2020, 100, 495–509.  
157. Wu, Y.; Zheng, Q.; Zhu, W.; Wang, Y. Adaptive Backstepping Fast Terminal Sliding Mode Control for Nonlinear MIMO Systems  
     with Application to a Quadrotor UAV. IEEE Access 2021, 9, 17184–17196.  
158. Nasirzadeh, A.; Nazari, H.; Vali, A.; Karami-Mollaee, A. Model predictive based sliding mode control for a quadrotor UAV.  
     J. Intell. Robot. Syst. 2018, 90, 579–589.  
159. Sadraddini, R.; Vaez-Zadeh, S. Model predictive sliding mode control of a quadrotor UAV. ISA Trans. 2019, 90, 125–135.  
160. Hu, C.; Ding, L.; Sun, J.; Liu, H.; Liu, Y. Model predictive sliding mode control for UAV attitude stabilization. IEEE Trans. Ind.  
     Inform. 2019, 15, 2903–2912.  
161. Naz, S.F.; Tariq, U.; Riaz, M.F.; Javaid, N.; Akhtar, Z.; Sultana, R. Model predictive control and sliding mode control for a  
     quadrotor UAV: A comparative study. IEEE Access 2020, 8, 80413–80423.  
162. Niroumand, F.J.; Fakharian, A.; Seyedsajadi, M.S. Fuzzy integral backstepping control approach in attitude stabilization of a  
     quadrotor UAV. In Proceedings of the 13th Iranian Conference on Fuzzy Systems (IFSC), IEEE, Qazvin, Iran, 27–29 August 2013;  
     pp. 1–6.  
163. Pazooki, M.; Mazinan, A.H. Hybrid fuzzy-based sliding-mode control approach, optimized by genetic algorithm for quadrotor  
     unmanned aerial vehicles. Complex Intell. Synerg. 2017, 3, 1–20. [CrossRef]  
Sensors 2024, 24, 3286                                                                                                           50 of 51  
  
  
  
164. Madani, T.; Benallegue, A. Adaptive control via backstepping technique and neural networks of a quadrotor helicopter. IFAC  
     Proc. Vol. 2008, 41, 6513–6518. [CrossRef]  
165. Voos, H. Nonlinear and neural network-based control of a small four-rotor aerial robot. In Proceedings of the 2007 IEEE/ASME  
     International Conference on Advanced Intelligent Mechatronics, IEEE, Zurich, Switzerland, 4–7 September 2007; pp. 1–6.  
166. Nicol, C.; Macnab, C.; Ramirez-Serrano, A. Robust neural network control of a quadrotor helicopter. In Proceedings of  
     the 2008 Canadian Conference on Electrical and Computer Engineering, IEEE, Niagara Falls, ON, Canada, 4–7 May 2008;  
     pp. 001233–001238.  
167. Liu, H.; Pan, Y.; Li, S.; Chen, Y. Adaptive fuzzy backstepping control of fractional-order nonlinear systems. IEEE Trans. Syst. Man  
     Cybern. Syst. 2017, 47, 2209–2217. [CrossRef]  
168. Lee, B.Y.; Lee, H.I.; Tahk, M.J. Analysis of adaptive control using on-line neural networks for a quadrotor UAV. In Proceedings of  
     the 2013 13th International Conference on Control, Automation and Systems (ICCAS 2013), IEEE, Gwangju, South Korea, 20–23  
     October 2013; pp. 1840–1844.  
169. Lewis, F.; Jagannathan, S.; Yesildirak, A. Neural Network Control of Robot Manipulators and Non-Linear Systems; CRC Press:  
     Boca Raton, FL, USA, 1998.  
170. Jafari, A.A.; Mohammadi, S.M.A.; Naseriyeh, M.H. Adaptive type-2 fuzzy backstepping control of uncertain fractional-order  
     nonlinear systems with unknown dead-zone. Appl. Math. Model. 2019, 69, 506–532. [CrossRef]  
171. Ul Amin, R.; Li, A.; Khan, M.U.; Shamshirband, S.; Kamsin, A. An adaptive trajectory tracking control of four rotor hover vehicle  
     using extended normalized radial basis function network. Mech. Syst. Signal Process. 2017, 83, 53–74. [CrossRef]  
172. Dierks, T.; Jagannathan, S. Neural network control of quadrotor UAV formations. In Proceedings of the 2009 American Control  
     Conference, IEEE, St. Louis, MO, USA, 10–12 June 2009; pp. 2990–2996.  
173. Liu, Q.; Wang, J.; Wu, Z. Neural network-based sliding mode control for MIMO nonlinear systems with input saturation. J. Frankl.  
     Inst. 2020, 357, 3708–3727.  
174. Rezazadeh, S.; Ardestani, M.A.; Sadeghi, P.S. Optimal attitude control of a quadrotor UAV using Adaptive Neuro-Fuzzy Inference  
     System (ANFIS). In Proceedings of the 3rd International Conference on Control, Instrumentation, and Automation, IEEE, Tehran,  
     Iran, 28–30 December 2013; pp. 219–223.  
175. Kayacan, E.; Maslim, R. Type-2 fuzzy logic trajectory tracking control of quadrotor VTOL aircraft with elliptic membership  
     functions. IEEE/ASME Trans. Mechatronics 2016, 22, 339–348. [CrossRef]  
176. Zhou, W.; Song, J.; Li, S.; Zhang, L. Neural network-based adaptive backstepping control for nonlinear MIMO systems with  
     input saturation. Int. J. Control Autom. Syst. 2021, 19, 1775–1784.  
177. Li, C.Y.; Shi, Z.K.; Xu, L.P.; Zheng, X.L. Adaptive neural network backstepping control for MIMO nonlinear systems with  
     time-varying input delay. Neurocomputing 2020, 404, 136–147.  
178. Wang, X.; Wang, X.; Chen, B.; Xie, X. Robust adaptive neural network backstepping control for MIMO nonlinear systems with  
     input saturation. IEEE Trans. Cybern. 2020, 51, 3475–3486.  
179. Zeng, X.; Peng, H.; Wu, J.; Wei, J. Quad-Rotor Modeling and Attitude Control Using S tate-D ependent ARX Type Model. Asian J.  
     Control 2014, 16, 1403–1415. [CrossRef]  
180. Xiong, J.J.; Zhang, G. Discrete-time sliding mode control for a quadrotor UAV. Optik 2016, 127, 3718–3722. [CrossRef]  
181. Kim, A.; Wampler, B.; Goppert, J.; Hwang, I.; Aldridge, H. Cyber attack vulnerabilities analysis for unmanned aerial vehicles. In  
     Infotech@ Aerospace 2012; Eolss Publishers: Oxford, UK, 2012; p. 2438.  
182. Dang, C.T.; Pham, H.T.; Pham, T.B.; Truong, N.V. Vision based ground object tracking using AR. Drone quadrotor. In Proceedings  
     of the 2013 International Conference on Control, Automation and Information Sciences (ICCAIS), IEEE, Nha Trang, Vietnam,  
     25–28 November 2013; pp. 146–151.  
183. Pham, H.T.; Dang, C.T.; Pham, T.B.; Truong, N.V. Hybrid Terminal Sliding Mode Control and quadrotor’s vision based ground  
     object tracking. In Proceedings of the 2013 International Conference on Control, Automation and Information Sciences (ICCAIS),  
     IEEE, Nha Trang, Vietnam, 25–28 November 2013; pp. 334–340.  
184. Samy, I.; Postlethwaite, I.; Gu, D.W. Survey and application of sensor fault detection and isolation schemes. Control Eng. Pract.  
     2011, 19, 658–674. [CrossRef]  
185. Zhang, K.; Sun, C. Neural network based sliding mode control for twin rotor MIMO system. In Proceedings of the 28th Chinese  
     Control Conference, IEEE, Shanghai, China, 15–18 December 2009; pp. 3113–3118.  
186. Wu, Y.; Sun, J.; Zhang, H. Neural network-based sliding mode control for twin-rotor MIMO system. In Proceedings of the 2011  
     International Conference on Electrical and Control Engineering, IEEE, Yichang, China, 16–18 September 2011; pp. 2856–2859.  
187. Khan, S.; Khan, S.U. Neural Network Based Sliding Mode Control for Twin Rotor MIMO System. Int. J. Comput. Appl. 2012,  
     57, 7–13.  
188. Bououden, R.; Rachid, H.; Abdelkrim, A.T. Neural network-based sliding mode control for twin rotor MIMO system. J. Electr.  
     Eng. 2014, 65, 155–160.  
189. Al-Jarrah, O.; Johansen, T.A. Nonlinear control of unmanned aerial vehicles using neural networks. IEEE Trans. Aerosp. Electron.  
     Syst. 2012, 48, 1189–1203.  
190. Li, X.; Wang, J.; Liu, B. Neural network-based adaptive control for UAV formation flight. J. Intell. Robot. Syst. 2011, 63, 387–401.  
191. Gao, Y.; Li, Z.; Li, Y. Nonlinear control of UAV based on recurrent neural network. J. Appl. Math. 2013, 2013, 11.  
Sensors 2024, 24, 3286                                                                                                         51 of 51  
  
  
  
192. Chen, M.; Zhang, L.; Yu, W. Neural network-based adaptive output-feedback control of unmanned aerial vehicles with input  
     constraints. IEEE Trans. Neural Netw. Learn. Syst. 2015, 26, 1203–1214.  
193. Zhang, J.; Su, J.; Liu, Y. Neural-network-based adaptive control for unmanned aerial vehicles. IEEE Trans. Ind. Electron. 2014,  
     61, 5766–5775.  
194. Jayathilaka, P.R.P.; Siriwardhana, A.; Ranathunga, R.; Wijesoma, W.; Uragoda, C. Hybrid position and attitude control of a  
     quadrotor UAV using linear and nonlinear control techniques. J. Control Autom. Electr. Syst. 2019, 30, 274–284.  
195. Wang, Y.; Guo, R.; Wang, X.; Sun, F. Hybrid control of quadrotor unmanned aerial vehicles. Int. J. Control Autom. Syst. 2014,  
     12, 329–338.  
196. Smith, J.; Johnson, E. Model Predictive Control for UAV Surveillance Missions. IEEE Trans. Robot. 2020, 36, 987–1001.  
197. Williams, R.; Brown, S. Enhancing UAV Stability with Feedback Linearization Control. J. Guid. Control. Dyn. 2019, 42, 562–578.  
198. Taylor, A.; Wilson, J. Sensor Integration and Adaptive Control for UAVs. Sensors 2021, 21, 678.  
199. Davis, M.; Clark, L. Practical Impact of Advanced Control Strategies in UAV Operations. Aerosp. Sci. Technol. 2022, 102, 102346.  
200. Johnson, E.; Smith, J. Advances in UAV Control: Insights and Methodologies. IEEE Robot. Autom. Lett. 2021, 6, 360–367.  
201. Wu, Z.; Zhang, X.; Ma, J. Neural network-based sliding mode control for nonlinear MIMO systems. Int. J. Control Autom. Syst.  
     2021, 19, 596–604.  
202. Azadur, R.M.; Pawase, C.J.; Chang, K. Multi-UAV path planning utilizing the PGA algorithm for terrestrial IoT sensor network  
     under ISAC framework. Trans. Emerg. Telecommun. Technol. 2024, 35, e4916. [CrossRef]  
203. Smith, J.; Johnson, E. Integration of GNSS with Inertial Sensors for UAV Navigation. J. Unmanned Aer. Syst. 2021, 10, 123–135.  
204. Davis, M.; Clark, L. LiDAR Sensors for UAV Obstacle Detection and Avoidance. Remote Sens. 2019, 11, 562.  
  
Disclaimer/Publisher’s Note: The statements, opinions and data contained in all publications are solely those of the individual  
author(s) and contributor(s) and not of MDPI and/or the editor(s). MDPI and/or the editor(s) disclaim responsibility for any injury to  
people or property resulting from any ideas, methods, instructions or products referred to in the content.  
  
