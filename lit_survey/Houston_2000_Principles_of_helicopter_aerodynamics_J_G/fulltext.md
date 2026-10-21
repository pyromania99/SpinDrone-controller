# Principles of helicopter aerodynamics. J.G. Leishman. Cambridge University Press, The Edinburgh Building, Cambridge CB2 2RU, UK.2000. 496pp. Illustrated. £60 ISBN 0-521-66060-2.

*Extracted text. Regenerable; edit notes.md instead.*

J. Gordon Leishman Frontmatter More Information

Principles of Helicopter Aerodynamics Second Edition The helicopter is truly a unique form of aircraft and a mastery of modern aeronautical engineering that fulﬁlls a variety of civilian and military roles. The usefulness of the helicopter lies in its unique ability to take off and land vertically on almost any terrain, to hover stationary relative to the ground, and to ﬂy forward, backward, or sideways. These unique ﬂying characteristics, however, come at a price, including complex aerodynamic problems, signiﬁcant vibrations, high levels of noise, and relatively large power requirements compared to a ﬁxed-wing aircraft of the same weight. This book, written by an internationally recognized teacher and researcher in the ﬁeld, provides a thorough, modern treatment of the aerodynamic principles of helicopters and other rotating-wing vertical lift aircraft such as tilt-rotors and autogiros. The ﬁrst part of the text begins with a unique technical history of helicopter ﬂight and then covers basic methods of rotor aerodynamic analysis and related issues associated with the performance of the helicopter and its aerodynamic design. The second part is devoted to more advanced topics in helicopter aerodynamics, including airfoil ﬂows, unsteady aerodynamics, dynamic stall, rotor wakes and rotor–airframe aerodynamic interactions. The third part of the book contains chapters on autogiros and advanced methods of helicopter aerodynamic analysis. A companion chapter on the aerodynamics of wind turbines recognizes both the commonalities and differences with the aerodynamic problems found on helicopters. Every chapter is extensively illustrated and concludes with a comprehensive bibliography and a set of homework problems. Advanced undergraduate and graduate students, as well as practicing engineers and researchers, will welcome this thorough and up-to-date text on the principles of helicopter and rotating-wing aerodynamics. Dr. J. Gordon Leishman is the Minta Martin Chair of Engineering and Professor of Aerospace Engineering at the University of Maryland. He is a former aerodynamicist at Westland Helicopters and has written extensively on topics in helicopter aerodynamics. Dr. Leishman is a Fellow of the Royal Aeronautical Society and an Associate Fellow of the American Institute of Aeronautics & Astronautics. He is also the Editor-in-Chief for the Journal of the American Helicopter Society.

J. Gordon Leishman Frontmatter More Information

Cambridge Aerospace Series Editors Wei Shyy and Michael J. Rycroft

1. J.M. Rolfe and K.J. Staples (eds.): Flight Simulation

2. P. Berlin: The Geostationary Applications Satellite

3. M.J.T. Smith: Aircraft Noise

4. N.X. Vinh: Flight Mechanics of High-Performance Aircraft

5. W.A. Mair and D.L. Birdsall: Aircraft Performance

6. M.J. Abzug and E.E. Larrabee: Airplane Stability and Control

7. M.J. Sidi: Spacecraft Dynamics and Control

8. J.D. Anderson: A History of Aerodynamics

9. A.M. Cruise, J.A. Bowles, C.V. Goodall, and T.J. Patrick: Principles of Space

Instrument Design

10. G.A. Khoury and J.D. Gillett (eds.): Airship Technology

11. J. Fielding: Introduction to Aircraft Design

12. J.G. Leishman: Principles of Helicopter Aerodynamics

13. J. Katz and A. Plotkin: Low Speed Aerodynamics, 2nd Edition

14. M.J. Abzug and E.E. Larrabee: Airplane Stability and Control: A History of

the Technologies that Made Aviation Possible, 2nd Edition

15. D.H. Hodges and G.A. Pierce: Introduction to Structural Dynamics and

Aeroelasticity

16. W. Fehse: Automatic Rendezvous and Docking of Spacecraft

17. R.D. Flack: Fundamentals of Jet Propulsion with Applications

18. J.G. Leishman: Principles of Helicopter Aerodynamics, 2nd Edition

19. E.A. Baskharone: Principles of Turbomachinery in Air-Breathing Engines

J. Gordon Leishman Frontmatter More Information

Principles of Helicopter Aerodynamics Second Edition

J. GORDON LEISHMAN, D.Sc.(Eng.), Ph.D., F.R.Ae.S. University of Maryland

J. Gordon Leishman Frontmatter More Information

University Printing House, Cambridge CB2 8BS, United Kingdom One Liberty Plaza, 20th Floor, New York, NY 10006, USA

477 Williamstown Road, Port Melbourne, VIC 3207, Australia

4843/24, 2nd Floor, Ansari Road, Daryaganj, Delhi - 110002, India

79 Anson Road, #06-04/06, Singapore 079906

Cambridge University Press is part of the University of Cambridge. It furthers the University’s mission by disseminating knowledge in the pursuit of education, learning and research at the highest international levels of excellence. Information on this title: www.cambridge.org/9781107013353 © Cambridge University Press 2006 This publication is in copyright. Subject to statutory exception and to the provisions of relevant collective licensing agreements, no reproduction of any part may take place without the written permission of Cambridge University Press. First published 2000 Reprinted with corrections 2001, 2002, 2003 Second edition 2006 Reprinted 2007, 2008 First paperback edition 2017 A catalogue record for this publication is available from the British Library Library of Congress Cataloging in Publication data Leishman, J. Gordon. Principles of helicopter aerodynamics / J. Gordon Leishman. – 2nd ed. p. cm. – (Cambridge aerospace series) Includes bibliographical references and index. ISBN 0-521-85860-7 (hardback)

1. Helicopters – Aerodynamics. I. Title. II. Series.

TL716.L43 2005

629.133´352 – dc22 2005025467 ISBN 978-0-521-85860-1 Hardback ISBN 978-1-107-01335-3 Paperback Cambridge University Press has no responsibility for the persistence or accuracy of URLs for external or third-party internet websites referred to in this publication, and does not guarantee that any content on such websites is, or will remain, accurate or appropriate.

J. Gordon Leishman Frontmatter More Information

TO M Y S TU D E N T S

in appreciation of all they have taught me

J. Gordon Leishman Frontmatter More Information

Contents

Preface to the Second Edition Preface to the First Edition Acknowledgments List of Main Symbols

1 Introduction: A History of Helicopter Flight

1.1 Rising Vertically

1.2 Producing Thrust

1.3 Key Technical Problems in Attaining Vertical Flight

1.4 Early Thinking

1.5 The Hoppers

1.6 The First Hoverers

1.7 Not Quite a Helicopter

1.8 Engines: A Key Enabling Technology

1.9 On the Verge of Success

1.10 The First Successes

1.11 Toward Mass Production

1.12 Maturing Technology

1.13 Compounds, Tilt-Wings, and Tilt-Rotors

1.14 Chapter Review

1.15 Questions

Bibliography

2 Fundamentals of Rotor Aerodynamics

2.1 Introduction

2.2 Momentum Theory Analysis in Hovering Flight

2.2.1 Flow Near a Hovering Rotor

2.2.2 Conservation Laws of Aerodynamics

2.2.3 Application to a Hovering Rotor

2.3 Disk Loading and Power Loading

2.4 Induced Inﬂow Ratio

2.5 Thrust and Power Coefﬁcients

2.6 Comparison of Theory with Measured Rotor Performance

2.7 Nonideal Effects on Rotor Performance

2.8 Figure of Merit

2.9 Estimating Nonideal Effects from Rotor Measurements

2.10 Induced Tip Loss

2.11 Rotor Solidity and Blade Loading Coefﬁcient

2.12 Power Loading

page xix xxiii xxvii xxxi 1 1 4 5 6 11 17 20 23 25 28 33 40 47 49 50 51 55 55 58 59 60 61 65 66 66 68 68 70 74 74 77 80

ix

J. Gordon Leishman Frontmatter More Information

x

Contents

2.13 Momentum Analysis in Axial Climb and Descent

2.13.1 Axial Climb

2.13.2 Axial Descent

2.13.3 Region between Hover and Windmill State

2.13.4 Power Required in Axial Climbing and Descending Flight

2.13.5 Four Working States of the Rotor in Axial Flight

2.13.6 Vortex Ring State

2.13.7 Autorotation

2.14 Momentum Analysis in Forward Flight

2.14.1 Induced Velocity in Forward Flight

2.14.2 Special Case, α = 0

2.14.3 Numerical Solution to Inﬂow Equation

2.14.4 General Form of the Inﬂow Equation

2.14.5 Validity of the Inﬂow Equation

2.14.6 Rotor Power Requirements in Forward Flight

2.15 Other Applications of the Momentum Theory

2.15.1 Coaxial Rotor Systems

2.15.2 Tandem Rotor Systems

2.16 Chapter Review

2.17 Questions

Bibliography

3 Blade Element Analysis

3.1 Introduction

3.2 Blade Element Analysis in Hover and Axial Flight

3.2.1 Integrated Rotor Thrust and Power

3.2.2 Thrust Approximations

3.2.3 Torque–Power Approximations

3.2.4 Tip-Loss Factor

3.3 Blade Element Momentum Theory (BEMT)

3.3.1 Assumed Radial Distributions of Inﬂow on the Blades

3.3.2 Radial Inﬂow Equation

3.3.3 Ideal Twist

3.3.4 BEMT: Numerical Solution

3.3.5 Distributions of Inﬂow and Airloads

3.3.6 Effects of Swirl Velocity

3.3.7 The Optimum Hovering Rotor

3.3.8 Circulation Theory of Lift

3.3.9 Power Estimates for the Rotor

3.3.10 Prandtl’s Tip-Loss Function

3.3.11 Blade Design and Figure of Merit

3.3.12 BEMT in Climbing Flight

3.3.13 Further Comparisons of BEMT with Experiment

3.3.14 Compressibility Corrections to Rotor Performance

3.4 Equivalent Blade Chords and Weighted Solidity

3.4.1 Mean Wing Chords

3.4.2 Thrust Weighted Solidity

3.4.3 Power–Torque Weighted Solidity

3.4.4 Weighted Solidity of the Optimum Rotor

81 81 83 86 87 88 90 91 93 95 96 97 99 99 99 101 101 106 110 110 113 115 115 117 119 119 122 122 125 126 127 128 130 131 134 135 138 139 141 145 146 148 150 152 152 153 153 154

J. Gordon Leishman Frontmatter More Information

Contents

3.4.5 Weighted Solidities of Tapered Blades

3.4.6 Mean Lift Coefﬁcient

3.5 Blade Element Analysis in Forward Flight

3.5.1 Determining Blade Forces

3.5.2 Deﬁnition of the Approximate Induced Velocity Field

3.6 Chapter Review

3.7 Questions

Bibliography

4 Rotating Blade Motion

4.1 Introduction

4.2 Types of Rotors

4.3 Equilibrium about the Flapping Hinge

4.4 Equilibrium about the Lead–Lag Hinge

4.5 Equation of Motion for a Flapping Blade

4.6 Physical Description of Blade Flapping

4.6.1 Coning Angle

4.6.2 Longitudinal Flapping Angle

4.6.3 Lateral Flapping Angle

4.6.4 Higher Harmonics of Blade Flapping

4.7 Dynamics of Blade Flapping with a Hinge Offset

4.8 Blade Feathering and the Swashplate

4.9 Review of Rotor Reference Axes

4.10 Dynamics of a Lagging Blade with a Hinge Offset

4.11 Coupled Flap–Lag Motion

4.12 Coupled Pitch–Flap Motion

4.13 Other Types of Rotors

4.13.1 Teetering Rotor

4.13.2 Semi-Rigid or Hingeless Rotors

4.14 Introduction to Rotor Trim

4.14.1 Equations for Free-Flight Trim

4.14.2 Typical Trim Solution Procedure for Level Flight

4.15 Chapter Review

4.16 Questions

Bibliography

5 Helicopter Performance

5.1 Introduction

5.2 The International Standard Atmosphere

5.3 Hovering and Axial Climb Performance

5.4 Forward Flight Performance

5.4.1 Induced Power

5.4.2 Blade Proﬁle Power

5.4.3 Compressibility Losses and Tip Relief

5.4.4 Reverse Flow

5.4.5 Parasitic Power

5.4.6 Climb Power

5.4.7 Tail Rotor Power

5.4.8 Total Power

xi 154 155 156 156 158 166 167 169 171 171 172 174 176 178 183 183 183 185 185 186 188 190 194 196 198 199 199 200 202 204 207 209 209 211 212 212 212 215 217 218 219 220 223 225 226 226 227

J. Gordon Leishman Frontmatter More Information

xii

Contents

5.5 Performance Analysis

5.5.1 Effect of Gross Weight

5.5.2 Effect of Density Altitude

5.5.3 Lift-to-Drag Ratios

5.5.4 Climb Performance

5.5.5 Engine Fuel Consumption

5.5.6 Speed for Minimum Power

5.5.7 Speed for Maximum Range

5.5.8 Range–Payload and Endurance–Payload Relations

5.5.9 Maximum Altitude or Ceiling

5.5.10 Factors Affecting Maximum Attainable Forward Speed

5.5.11 Performance of Coaxial and Tandem Dual Rotor

Systems

5.6 Autorotational Performance

5.6.1 Autorotation in Forward Flight

5.6.2 Height–Velocity (H–V) Curve

5.6.3 Autorotation Index

5.7 Vortex Ring State (VRS)

5.7.1 Quantiﬁcation of VRS Effects

5.7.2 Implications of VRS on Flight Boundary

5.8 Ground Effect

5.8.1 Hovering Flight Near the Ground

5.8.2 Forward Flight Near the Ground

5.9 Performance in Maneuvering Flight

5.9.1 Steady Maneuvers

5.9.2 Transient Maneuvers

5.10 Factors Inﬂuencing Performance Degradation

5.11 Chapter Review

5.12 Questions

Bibliography

6 Aerodynamic Design of Helicopters

6.1 Introduction

6.2 Overall Design Requirements

6.3 Conceptual and Preliminary Design Processes

6.4 Design of the Main Rotor

6.4.1 Rotor Diameter

6.4.2 Tip Speed

6.4.3 Rotor Solidity

6.4.4 Number of Blades

6.4.5 Blade Twist

6.4.6 Blade Planform and Tip Shape

6.4.7 Airfoil Sections

6.5 Case Study: The BERP Rotor

6.6 Fuselage Aerodynamic Design Issues

6.6.1 Fuselage Drag

6.6.2 Vertical Drag and Download Penalty

6.6.3 Vertical Drag Recovery

6.6.4 Fuselage Side-Force

228 228 229 229 230 231 233 235 237 238 239 240 242 246 249 251 252 252 256 257 258 260 263 264 265 269 271 272 273 277 277 277 279 280 281 283 285 288 290 292 295 301 304 304 307 309 310

J. Gordon Leishman Frontmatter More Information

Contents

6.7 Empennage Design

6.7.1 Horizontal Stabilizer

6.7.2 Vertical Stabilizer

6.8 Role of Wind Tunnels in Aerodynamic Design

6.9 Design of Tail Rotors

6.9.1 Physical Size

6.9.2 Thrust Requirements

6.9.3 Precessional Stall Issues

6.9.4 “Pushers” versus “Tractors”

6.9.5 Design Requirements

6.9.6 Representative Tail Rotor Designs

6.10 Other Anti-Torque Devices

6.10.1 Fan-in-Fin

6.10.2 NOTAR Design

6.11 High-Speed Rotorcraft

6.11.1 Compound Helicopters

6.11.2 Tilt-Rotors

6.11.3 Other High-Speed Concepts

6.12 Smart Rotor Systems

6.13 Human-Powered Helicopter

6.14 Hovering Micro Air Vehicles

6.15 Chapter Review

6.16 Questions

Bibliography

7 Aerodynamics of Rotor Airfoils

7.1 Introduction

7.2 Helicopter Rotor Airfoil Requirements

7.3 Reynolds Number and Mach Number Effects

7.3.1 Reynolds Number

7.3.2 Concept of the Boundary Layer

7.3.3 Mach Number

7.3.4 Model Rotor Similarity Parameters

7.4 Airfoil Shape Deﬁnition

7.5 Airfoil Pressure Distributions

7.5.1 Pressure Coefﬁcient

7.5.2 Critical Pressure Coefﬁcient

7.5.3 Synthesis of Chordwise Pressure

7.5.4 Measurements of Chordwise Pressure

7.6 Aerodynamics of a Representative Airfoil Section

7.6.1 Integration of Distributed Forces

7.6.2 Pressure Integration

7.6.3 Representative Force and Moment Results

7.7 Pitching Moment and Related Issues

7.7.1 Aerodynamic Center

7.7.2 Center of Pressure

7.7.3 Effect of Airfoil Shape on Pitching Moment

7.7.4 Use of Trailing Edge Tabs

7.7.5 Reﬂexed Airfoils

xiii 311 311 312 313 314 315 315 317 318 319 320 321 321 324 325 325 327 328 330 331 334 338 338 340 347 347 348 350 350 352 357 359 360 363 363 364 365 366 368 368 370 371 374 375 377 378 381 383

J. Gordon Leishman Frontmatter More Information

xiv

Contents

7.8 Drag

7.9 Maximum Lift and Stall Characteristics

7.9.1 Effects of Reynolds Number

7.9.2 Effects of Mach Number

7.10 Advanced Rotor Airfoil Design

7.11 Representing Static Airfoil Characteristics

7.11.1 Linear Aerodynamic Models

7.11.2 Nonlinear Aerodynamic Models

7.11.3 Table Look-Up

7.11.4 Direct Curve Fitting

7.11.5 Beddoes Method

7.11.6 High Angle of Attack Range

7.12 Circulation Controlled Airfoils

7.13 Very Low Reynolds Number Airfoil Characteristics

7.14 Effects of Damage on Airfoil Performance

7.15 Chapter Review

7.16 Questions

Bibliography

8 Unsteady Airfoil Behavior

8.1 Introduction

8.2 Sources of Unsteady Aerodynamic Loading

8.3 Concepts of the Blade Wake

8.4 Reduced Frequency and Reduced Time

8.5 Unsteady Attached Flow

8.6 Principles of Quasi-Steady Thin-Airfoil Theory

8.7 Theodorsen’s Theory

8.7.1 Pure Angle of Attack Oscillations

8.7.2 Pure Plunging Oscillations

8.7.3 Pitching Oscillations

8.8 The Returning Wake: Loewy’s Problem

8.9 Sinusoidal Gust: Sears’s Problem

8.10 Indicial Response: Wagner’s Problem

8.11 Sharp-Edged Gust: Küssner’s Problem

8.12 Traveling Sharp-Edged Gust: Miles’s Problem

8.13 Time-Varying Incident Velocity

8.14 General Application of the Indicial Response Method

8.14.1 Recurrence Solution to the Duhamel Integral

8.14.2 State-Space Solution for Arbitrary Motion

8.15 Indicial Method for Subsonic Compressible Flow

8.15.1 Approximations to the Indicial Response

8.15.2 Indicial Lift from Angle of Attack

8.15.3 Indicial Lift from Pitch Rate

8.15.4 Determination of Indicial Function Coefﬁcients

8.15.5 Indicial Pitching Moment from Angle of Attack

8.15.6 Indicial Pitching Moment from Pitch Rate

8.15.7 Unsteady Axial Force and Airfoil Drag

8.15.8 State-Space Aerodynamic Model for Compressible

Flow

8.15.9 Comparison with Experiment

383 385 389 392 398 401 401 403 403 403 404 407 409 411 412 415 416 418 423 423 424 424 427 428 429 431 434 436 438 441 442 446 448 450 453 457 459 463 465 467 469 470 471 474 474 476 478 480

J. Gordon Leishman Frontmatter More Information

Contents

8.16 Nonuniform Vertical Velocity Fields

8.16.1 Exact Subsonic Linear Theory

8.16.2 Approximations to the Sharp-Edged Gust Functions

8.16.3 Response to an Arbitrary Vertical Gust

8.16.4 Blade–Vortex Interaction (BVI) Problem

8.16.5 Convecting Vertical Gusts in Subsonic Flow

8.17 Time-Varying Incident Mach Number

8.18 Unsteady Aerodynamics of Flaps

8.18.1 Incompressible Flow Theory

8.18.2 Subsonic Flow Theory

8.18.3 Comparison with Measurements

8.19 Principles of Noise Produced by Unsteady Forces

8.19.1 Retarded Time and Source Time

8.19.2 Wave Tracing

8.19.3 Compactness

8.19.4 Trace or Phase Mach Number

8.19.5 Ffowcs-Williams–Hawkins Equation

8.19.6 BVI Acoustic Model Problem

8.19.7 Comparison of Aeroacoustic Methods

8.19.8 Methods of Rotor Noise Reduction

8.20 Chapter Review

8.21 Questions

Bibliography

9 Dynamic Stall

9.1 Introduction

9.2 Flow Morphology of Dynamic Stall

9.3 Dynamic Stall in the Rotor Environment

9.4 Effects of Forcing Conditions on Dynamic Stall

9.5 Modeling of Dynamic Stall

9.5.1 Semi-Empirical Models of Dynamic Stall

9.5.2 Capabilities of Dynamic Stall Modeling

9.5.3 Future Modeling Goals with Semi-Empirical Models

9.6 Torsional Damping

9.7 Effects of Sweep Angle on Dynamic Stall

9.8 Effect of Airfoil Shape on Dynamic Stall

9.9 Three-Dimensional Effects on Dynamic Stall

9.10 Time-Varying Velocity Effects on Dynamic Stall

9.11 Prediction of In-Flight Airloads

9.12 Stall Control

9.13 Chapter Review

9.14 Questions

Bibliography

10 Rotor Wakes and Blade Tip Vortices

10.1 Introduction

10.2 Flow Visualization Techniques

10.2.1 Natural Condensation Effects

10.2.2 Smoke Flow Visualization

10.2.3 Density Gradient Methods

xv 483 483 484 487 488 490 492 492 493 497 500 502 504 505 506 507 508 510 513 515 516 517 519 525 525 527 529 531 535 536 541 543 545 547 551 553 556 557 559 560 561 562 567 567 568 568 569 570

J. Gordon Leishman Frontmatter More Information

xvi

Contents

10.3 Characteristics of the Rotor Wake in Hover

10.3.1 General Features

10.3.2 Wake Geometry in Hover

10.4 Characteristics of the Rotor Wake in Forward Flight

10.4.1 Wake Boundaries

10.4.2 Blade–Vortex Interactions (BVIs)

10.5 Other Characteristics of Rotor Wakes

10.5.1 Periodicity versus Aperiodicity

10.5.2 Vortex Perturbations and Instabilities

10.6 Detailed Structure of the Tip Vortices

10.6.1 Velocity Field

10.6.2 Models for the Tip Vortex

10.6.3 Vorticity Diffusion Effects and Vortex Core Growth

10.6.4 Correlation of Rotor Tip Vortex Data

10.6.5 Flow Rotation Effects on Turbulence Inside Vortices

10.7 Vortex Models of the Rotor Wake

10.7.1 Biot–Savart Law

10.7.2 Vortex Segmentation

10.7.3 Governing Equations for the Convecting Vortex Wake

10.7.4 Prescribed Wake Models for Hovering Flight

10.7.5 Prescribed Vortex Wake Models for Forward Flight

10.7.6 Free-Vortex Wake Analyses

10.8 Aperiodic Wake Developments

10.8.1 Wake Stability Analysis

10.8.2 Flow Visualization of Transient Wake Problems

10.8.3 Dynamic Inﬂow

10.8.4 Time-Marching Free-Vortex Wakes

10.8.5 Simulation of Carpenter & Friedovich Problem

10.9 General Dynamic Inﬂow Models

10.10 Descending Flight and the Vortex Ring State

10.11 Wake Developments in Maneuvering Flight

10.12 Chapter Review

10.13 Questions

Bibliography

11 Rotor–Airframe Interactional Aerodynamics

11.1 Introduction

11.2 Rotor–Fuselage Interactions

11.2.1 Effects of the Fuselage on Rotor Performance

11.2.2 Time-Averaged Effects on the Airframe

11.2.3 Unsteady Rotor–Fuselage Interactions

11.2.4 Fuselage Side-Forces

11.2.5 Modeling of Rotor–Fuselage Interactions

11.3 Rotor–Empennage Interactions

11.3.1 Airloads on the Horizontal Tail

11.3.2 Modeling of Rotor–Empennage Interactions

11.4 Rotor–Tail Rotor Interactions

11.5 Chapter Review

11.6 Questions

Bibliography

572 572 573 575 577 578 582 582 582 584 585 586 592 594 595 598 599 601 602 604 607 614 627 627 630 631 633 633 635 638 640 645 646 647 655 655 657 658 662 666 673 674 676 679 680 682 685 686 687

J. Gordon Leishman Frontmatter More Information

Contents

xvii

12 Autogiros and Gyroplanes

692

12.1 Introduction

12.2 The Curious Phenomenon of Autorotation

12.3 Review of Autorotational Physics

12.4 Rolling Rotors: The Dilemma of Asymmetric Lift

12.5 Innovation of the Flapping and Lagging Hinges

12.6 Prerotating the Rotor

12.7 Autogiro Theory Meets Practice

12.8 Vertical Flight Performance of the Autogiro

12.9 Forward Flight Performance of the Autogiro

12.10 Comparison of Autogiro Performance with the Helicopter

12.11 Airfoils for Autogiros

12.12 NACA Research on Autogiros

12.13 Giving Better Control: Orientable Rotors

12.14 Improving Performance: Jump and Towering Takeoffs

12.15 Ground and Air Resonance

12.16 Helicopters Eclipse Autogiros

12.17 Renaissance of the Autogiro?

12.18 Chapter Review

12.19 Questions

Bibliography

13 Aerodynamics of Wind Turbines

13.1 Introduction

13.2 History of Wind Turbine Development

13.3 Power in the Wind

13.4 Momentum Theory Analysis for a Wind Turbine

13.4.1 Power and Thrust Coefﬁcients for a Wind Turbine

13.4.2 Theoretical Maximum Efﬁciency

13.5 Representative Power Curve for a Wind Turbine

13.6 Elementary Wind Models

13.7 Blade Element Model for the Wind Turbine

13.8 Blade Element Momentum Theory for a

Wind Turbine

13.8.1 Effect of Number of Blades

13.8.2 Effect of Viscous Drag

13.8.3 Tip-Loss Effects

13.8.4 Tip Losses and Other Viscous Losses

13.8.5 Effects of Stall

13.9 Airfoils for Wind Turbines

13.10 Yawed Flow Operation

13.11 Vortex Wake Considerations

13.12 Unsteady Aerodynamic Effects on Wind Turbines

13.12.1 Tower Shadow

13.12.2 Dynamic Stall and Stall Delay

13.13 Advanced Aerodynamic Modeling Requirements

13.14 Chapter Review

13.15 Questions

Bibliography

692 693 694 699 700 701 702 704 705 708 709 710 712 713 715 716 717 719 720 720 723 723 724 726 727 729 730 731 733 735 738 742 742 743 745 747 747 750 751 757 760 761 763 764 765 767

J. Gordon Leishman Frontmatter More Information

xviii

14 Computational Methods for Helicopter Aerodynamics

14.1 Introduction

14.2 Fundamental Governing Equations of Aerodynamics

14.2.1 Navier–Stokes Equations

14.2.2 Euler Equations

14.3 Vorticity Transport Equations

14.4 Vortex Methods

14.5 Boundary Layer Equations

14.6 Potential Equations

14.7 Surface Singularity Methods

14.8 Thin Airfoil Theory

14.9 Lifting-Line Blade Model

14.10 Applications of Advanced Computational Methods

14.10.1 Unsteady Airfoil Performance

14.10.2 Tip Vortex Formation

14.10.3 CFD Modeling of the Rotor Wake

14.10.4 Airframe Flows

14.10.5 Vibrations and Acoustics

14.10.6 Ground Effect

14.10.7 Vortex Ring State

14.11 Comprehensive Rotor Analyses

14.12 Chapter Review

14.13 Questions

Bibliography Appendix Index

Contents 771 771 772 773 776 777 779 780 783 783 786 787 790 790 794 797 798 801 803 803 805 808 809 810 815 817

J. Gordon Leishman Frontmatter More Information

Preface to the Second Edition

The reader will see many changes to this second and much enlarged edition of Principles of Helicopter Aerodynamics. However, the goal in writing the book remains the same, to give a reasonably adequate description of background theory and present an up-to-date treatment of the subject of helicopter aerodynamics, or at least as up to date as it can be. With the life span of a modern engineering textbook being measured in a few years, it seemed appropriate to regularly update the book with both revised and new material that, in part, reﬂect both current areas of topical interest as well as areas of ongoing fundamental research. The revised text will continue to appeal both to those learning the ﬁeld anew and to those who are practicing engineers. The text requires some familiarity with basic aerodynamics and mathematical concepts, although no prior background in rotating-wing aerodynamics is assumed. Both new and updated questions at the end of each chapter will satisfy instructors who have used the ﬁrst edition of the book in the classroom and have found the solutions manual useful in their teaching. Work on revising the ﬁrst edition and compiling the second edition started early in 2003, although planning, gathering materials, and writing started well before then. The new material was ﬁnally deﬁned late in 2003 and some of it was taught in my “Helicopter Aerodynamics II” class during the spring of 2004. In this second edition I have taken the opportunity to thoroughly revise the original text for content and clarity, as well as to ﬁnally weed out the remaining evasive typographical errors that made it into the third printing of the ﬁrst edition. Any errors that do remain are entirely of my own doing. Many revisions to the text have been made based on my own rereading during the last ﬁve years, as well as on the feedback and suggestions received from many readers, a lot of them students. In the ﬁrst instance, I have put back some important parts of the original manuscript that were edited out of the ﬁrst published edition. This by itself has provided much better detail and continuity between sections and chapters, as well as giving a fresh new look to the book. A few original sections have been deemed duplicative or redundant with the addition of the new chapters and so have been edited out completely. Other parts of the text have been moved to the new chapters. Most of the original ﬁgures have been revised or updated, or have been reordered in a more logical sequence. The introductory chapter on the development of the helicopter has been completely revised. I have taken the opportunity to thoroughly check historical events, dates, and other details, which are often conﬂicting between different publications. Tracing original documents back over ninety years in some cases has proved challenging, but I have been helped in this task by many willing individuals. I am particularly grateful to Roger Connor of the National Air and Space Museum for sharing his extensive knowledge of helicopters and VTOL aircraft. I have also added new material to this chapter that has strengthened the description of the technological advances needed for vertical ﬂight. The addition of new material on engines has helped put the development of the helicopter into a better perspective when compared to the development of other aircraft. Chapters 2 and 3 describe the fundamental aerodynamic characteristics of helicopter rotors, and these two chapters have seen the fewest changes in the second edition. Even here, however, several additions xix

J. Gordon Leishman Frontmatter More Information

xx

Preface to the Second Edition

have been made to better introduce the fundamentals of helicopter ﬂight and to provide an essential primer for the more advanced materials contained in the subsequent chapters of the book. Chapter 4 on rotating blade motion has seen the addition of sections on teetering and semi-rigid rotors to complement the discussion on the articulated form of rotor. A discussion of pitch–ﬂap coupling and an introductory section on ground resonance helps round out the revised chapter. Finally, the discussion on the methods of solving for rotor trim has expanded to complete this chapter. Chapter 5 is on helicopter performance. Initially I focused on a better description of the international standard atmosphere and why it is used in performance work. The addition of a section on compressibility losses on the rotor operating at higher forward speeds helps better explain the impact of this deleterious effect on helicopter performance. A new section on engines and the issues of speciﬁc fuel consumption connect better to a revised section on ﬂight range and endurance. There is also a new section on maximum altitude (ceiling) performance. Performance issues in rapidly descending ﬂight have received some recent attention and this seems to be a topic that is very poorly understood from a fundamental standpoint. To this end, revised and expanded sections on autorotational ﬂight and the vortex ring state should help cover the ﬁeld. Finally, a section on maneuvering ﬂight performance helps round out Chapter 5. The ﬁrst part of the book concludes with Chapter 6, which addresses helicopter aerodynamic design issues. This chapter has seen an expanded consideration of airframe download and other performance penalties. A new section on the use of wind tunnel testing in the evaluation of rotor air loads and helicopter performance helps put the difﬁculties of accurate ﬂight performance estimation into proper perspective. The issues of maneuvering ﬂight are considered again through a discussion of precessional stall effects, both on the main rotor and the tail rotor. Four new sections have been added to Chapter 6. The ﬁrst is on “smart” rotor systems, which may be one approach whereby the performance of the helicopter could be signiﬁcantly improved. It remains to be seen, however, if the practical issues in building and certifying a cost-competitive smart rotor system can be overcome. Second, the ideas of a human-powered helicopter are introduced. This is a problem that continues to interest generation after generation of students, although, as the discussion shows, the physiological limits of humans severely limit the possibilities of ever achieving successful human-powered vertical ﬂight. Nevertheless, it remains a problem that will continue to hold great fascination and it is worthy of consideration from both an aerodynamics and a vehicle design perspective. Third, there is a new section on micro air vehicles (MAVs). The ﬂight of MAVs involves low Reynolds number issues for which there is little existing knowledge of their aerodynamics, and so they require special considerations in their performance estimation. Fourth, a brief section on performance degradation issues associated with rotor and airframe icing completes this chapter. The second part of the book starts in Chapter 7 with a revised discussion on airfoil sections used for helicopter applications. The chapter is enhanced by a more thorough and up-to-date discussion on advanced airfoil design. The addition of a section on circulation controlled airfoils reﬂects renewed interest in this concept for alternative rotorcraft concepts. A short section on very low Reynolds number airfoil characteristics complements the discussion on MAVs found in Chapter 6. Finally, a section on the aerodynamic degradation of helicopter rotor airfoils from ballistic damage completes this chapter. Chapter 8 is a fairly large chapter on unsteady aerodynamics, and the existing material has seen several changes. A new section on the unsteady aerodynamics of airfoils with trailing edge ﬂaps recognizes their use on modern rotors for vibration and noise reduction and perhaps as a primary means of rotor control by replacing the swashplate. The addition of a new section on rotor aeroacoustics in this chapter helps make the connection between unsteady aerodynamic forces and the

J. Gordon Leishman Frontmatter More Information

Preface to the Second Edition

xxi

often obtrusive noise that seems to plague the modern helicopter. This section also provides a primer for students and aerodynamicists learning the ﬁeld of acoustics anew, as well as lending a bridge to practicing acousticians who need greater than superﬁcial knowledge about helicopter related acoustic problems. Chapter 10 on rotor wakes has seen the addition of new material throughout. This includes a fresh discussion on tip vortex modeling, including the effects of turbulence within the vortex core. The understanding of helicopters in maneuvering ﬂight has received much recent attention by the technical community, so a section on time-accurate wake modeling seemed a natural addition to the book. Chapters 11 through 14 are all new chapters. Chapter 11 is on rotor–airframe interactional aerodynamics that contains material left over from the draft of the ﬁrst edition. Because the helicopter as a system must function properly and predictably throughout its operational ﬂight envelope, an understanding of component-interaction aerodynamics is essential to the successful design of the modern helicopter. Several sections on this topic from the original text have now been parsed out and integrated with the new material that forms this chapter. Chapter 12 describes the technological development of autogiros and gyroplanes, and recognizes the fundamental role that the autogiro played in the development of the helicopter. This chapter also contains the essential aerodynamic theory of the autogiro. The need for this chapter also reﬂects a renaissance of interest in the commercialization of a modern gyroplane, for which new engineers must be suitably knowledgeable. Chapter 13 is on the aerodynamics of wind turbines. It may seem surprising to some to ﬁnd a chapter on wind turbines in a book on helicopters, but the aerodynamics of wind turbines have many similarities to the aerodynamics of helicopters, yet also with important differences. There are so relatively few books on wind turbines and those describing the aerodynamics are frequently out of print. Environmental concerns about global warming and the storage of waste from nuclear power plants has seen an increased emphasis on the use of wind energy, which is likely to see much more rapid and expansive use in years to come. The performance of wind turbines can be analyzed by many of the same methods used for helicopters, and the cross fertilization of expertise in the two areas will hopefully help foster a new understanding of both ﬁelds. Finally, Chapter 14 explains the basis of modern computational methods for rotor and helicopter analysis, and these are put into context with the capabilities classical methods and modern engineering approaches. Dr. Richard Brown of Imperial College at the University of London contributed heavily the writing of this chapter. To some, the development of the ﬁeld of computational ﬂuid dynamics (CFD) by itself is held out to be the “Holy Grail” for the helicopter aerodynamicist. But this is a very misleading perspective because CFD does not, by itself, hold the answer to all of the various problems found on helicopters. The answer lies more in the successful integration of advanced forms of aerodynamic analysis into other disciplines of analysis. It is also unwise for other approaches to be abandoned in the shorter term while CFD matures to an accepted level of capability, given that this could still be decades off, despite more optimistic claims. These CFD models require continuous and careful validation, both against more complete solutions and/or analytical results and against detailed experimental measurements. This is one reason why wind tunnel testing of helicopters and subsystems will continue to be essential to better understand and predict the capabilities of the helicopter in response to speciﬁc aerodynamic phenomena. The future offers many opportunities for new research focused toward the development of more innovative computational models with greater predictive capabilities for helicopter applications. Only then can the problems that limit the performance and capabilities of the helicopter be understood and mitigated.

J. Gordon Leishman Frontmatter More Information

Preface to the First Edition

This book is a college-level analytical and applied level exposition of the aerodynamic principles of helicopters and other rotating-wing vertical lift aircraft. It is written for students who have no background in rotating-wing aerodynamics but have had at least two semesters of basic aerodynamics at the undergraduate level and possibly one course at graduate level. The material covered has grown mainly out of two graduate-level courses in “Helicopter Aerodynamics” that I have taught at the University of Maryland since 1988. I have also taught a somewhat more general senior-level undergraduate course in “Helicopter Theory” about every other year, which is centered around the ﬁrst half of this book. These courses have been offered as part of the formal curriculum in the Center for Rotorcraft Education and Research, which was originally founded in 1982, partly through the efforts of Professor Alfred Gessow. It is now nearly ﬁfty years since Alfred Gessow and Gary Myers’ well-known book The Aerodynamics of the Helicopter was ﬁrst published. I am pleased to record in the preface to this book that in his status as Professor Emeritus, Alfred Gessow continues to be active in activities at the University of Maryland and also within government and professional organizations. As a testimony to his life-long dedication to education and research in helicopter technology, the Rotorcraft Center at the University of Maryland has been recently named in his honor. In the institutions where formal courses in helicopter technology have been taught, either at the undergraduate or graduate level, my experience is that they have been well received and very popular with the students. What is often most attractive to students is the highly multidisciplinary nature of helicopter engineering problems. Therefore, an introductory course in helicopters provides a good capstone to the aerospace engineering curriculum. Another factor for most students who have taken a helicopter course is the realization that so much more remains to be learned about fundamental aerodynamics, especially as it applies to rotating-wing aircraft. This is reﬂected in the experience levels in predicting the aerodynamics and overall behavior of helicopters before their ﬁrst ﬂight, which are less than desirable. Consequently, it is fair to say that the various aerodynamic problems associated with helicopters and other new forms of rotary-wing aircraft probably provide the scientists and engineers of the future with some of the most outstanding research challenges to be found in the ﬁeld of theoretical and applied aerodynamics. Serious work on this book project started about three years ago and was motivated primarily by my students. It has grown out of a relatively informal collection of classroom notes and research papers and an overwhelming need to synthesize both older and newer information on the subject of helicopter aerodynamics into one coherent volume. With the ever increasing content of new research material on helicopters and the increasing proportion of recent research material being included in the course, especially in the second semester, the development of a formal textbook was really a logical step. Even since 1980, when Wayne Johnson’s excellent book Helicopter Theory was published, progress in understanding helicopter aerodynamics and other related ﬁelds has been remarkable. A glance at the content of recent proceedings of the Annual Forum of the American Helicopter Society or the European Rotorcraft Forum shows the scope and depth of new work being xxiii

J. Gordon Leishman Frontmatter More Information

xxiv

Preface to the First Edition

conducted today, which continues despite tightly controlled budgets. This has been fueled, in part, by the great advances in computer technology, which has fostered ambitious new analytical and numerical approaches to solving helicopter technology problems. Some of these approaches now come under the banner of computational ﬂuid dynamics (CFD) using numerical solutions to the Euler and Navier–Stokes equations. While these techniques are not yet mature, CFD methods have begun to provide new insight into the complicated aerodynamic problems associated with helicopters that were previously intractable with existing mathematical methods or were limited by available experimental techniques. As these new numerical methods continue to mature and become better validated, the ﬁrst decade of the twenty-ﬁrst century will see an increasing use of CFD tools in the design of new and improved helicopters. The past twenty years has also marked a revolution in the experimental studies of helicopter aerodynamics, where advances in ﬂow diagnostic and other instrumentation has allowed measurements on rotors to be made with a ﬁdelity that was considered impossible just a few years ago. The complex nature of the problems found on helicopters means that both experiment and theory must continue to go hand in hand to forge a better understanding of the whole. This will result in the development of new rotating-wing aircraft with better performance, lower vibration, better reliability and maintainability, and with lower direct and operational costs. The modern spirit of international cooperation in research and development makes the years ahead in the twenty-ﬁrst century very exciting. As I have already mentioned, a signiﬁcant part of the content of this book has been focused toward students at the graduate level who are learning the principles of helicopter aerodynamics and are exploring the tools available to approach the vigorous, multidisciplinary research and development of the modern helicopter. In planning the content of this book, I have organized the material in two main parts. The ﬁrst part will be appropriate for a one-semester course in helicopter aerodynamics for senior undergraduate and ﬁrst-year graduate students. This material essentially represents a thorough introduction to fundamental helicopter problems, basic methods of rotor analysis, issues associated with helicopter performance, and conceptual design issues. I have attempted to follow the spirit of Gessow & Myers’s book, where theory is supported throughout by liberal references to experimental observations and measurements. In this regard, rediscovering the less well-known early NACA and RAE technical literature on the subject of helicopter aerodynamics proved to be one of the most satisfying aspects of writing this book. The rapid progress made in understanding the problems of the helicopter during the period between 1930 and 1950, and the ingenuity shown in both the experimental and analytical work, are quite remarkable. The second part of the book gives a more advanced treatment of more detailed aspects of helicopter aerodynamics, again with emphasis on physical concepts and basic methods of analysis. This part will appeal more to those students who plan to conduct research in helicopter aerodynamics and related ﬁelds or who are already practicing engineers in industry and government laboratories. However, I do hope that practicing engineers will relish in the opportunity to revisit the ﬁrst part of the book to review the basics and also to review the inherent assumptions and limitations of the fundamental concepts and methods. These are so often taken for granted, but they form the backbone of many modern forms of helicopter analysis. Because I have always found a need to bring “industrial practice” into the classroom, I have tried to incorporate engineering practice as well as some of my own industrial experience into the second part of the book. For this, I thank my former colleagues at Westland Helicopters for sharing their knowledge with me. Like most textbooks, the ﬁnal product has turned out to be not exactly what was originally planned. Along the way, topics have been added, parts of the text rearranged, and some other

J. Gordon Leishman Frontmatter More Information

Preface to the First Edition

xxv

topics deleted. Also, in light of the reviews of the preliminary manuscript, new ﬁgures have been added, many were modiﬁed, and others deleted. While more than 400 ﬁgures were originally prepared, less than 275 ﬁnally made it to the ﬁnished book. In the interests of space and publication costs, two chapters have been left out completely. These were “Interactional Aerodynamics” and “Advanced Computational Aerodynamic Techniques.” Both topics are referred to, albeit brieﬂy, throughout the book, but to include them would have made the ﬁnal size of the book prohibitively long. There have also been such rapid recent advances in these areas in the past few years that they are best left for a second edition. The list of key references for each chapter is extensive but by no means complete, and the reader is encouraged to follow through with the references contained in each publication, as required. Problems are provided at the end of each chapter and have been drawn on timeproven homework and examination questions. Solutions have been provided in a companion instructors’ solution manual. Chapter 1 introduces the helicopter through its technical history. This chapter grew out of my personal research into the Berliner helicopter experiments that were conducted during the early 1920s at the College Park Airport, which is close to the University of Maryland. Whereas in the technical development of ﬁxed-wing aircraft it is possible to point to several key historical events, it becomes quickly apparent that things are much less clear in the development of the helicopter. There are already many authoritative publications that detail the historical development of the helicopter, so I have tried to approach the discussion on more of a technical theme and to put the background and difﬁculties in understanding the aerodynamics of vertical ﬂight into broader perspective. This introduction is followed in Chapter 2 by an analysis of hovering and axial ﬂight using the Rankine–Froude momentum theory. The basic momentum theory concept was extended to forward ﬂight by Glauert and others from the RAE at Farnborough, inspired not by the helicopter, but by the success of Cierva’s Autogiro. It is shown that many of the important performance and operational characteristics of the helicopter can be deduced from Glauert’s extension of the basic momentum theory. The blade element and combined blade-element momentum theory is discussed in Chapter 3. These ideas were developed in the 1940s and provide the foundation for a more modern treatment of the aerodynamics of rotors. On the basis of certain assumptions, important information on blade design, such as optimum or ideal shapes for the blade planform and blade twist, can be deduced from the combined blade element momentum theory. Because helicopter blades have articulation, in that they can ﬂap and lag about hinges located near the root of each blade, it is not possible to understand the behavior of the helicopter solely from an aerodynamics perspective. Therefore, in Chapter 4 a discussion of rigid blade motion leads naturally into an understanding of the issues associated with rotor response to the changing aerodynamic loads and also to rotor control. Also introduced here are the ideas of rotor trim; that is, the pilot’s control inputs required to enable equilibrium ﬂight of the helicopter. Chapter 5 gives an introduction to helicopter performance and operational issues such as climbing and descending ﬂight, including the autorotative state and ﬂight near the ground. The ﬁrst part of the book concludes in Chapter 6, which reviews issues associated with the conceptual aerodynamic design of helicopters, including the main rotor, the fuselage and empennage, and the tail rotor. Although it might have been more appropriate to place this chapter at the end of the book, it provides a good bridge between the fundamentals and results to many problems that are still more of a research nature for which experimental research is incomplete and predictive capabilities are not yet mature. Chapter 7 starts the second part of the book with an important practical review of basic airfoil aerodynamics, including boundary layer and viscous aerodynamics, and the role of compressibility. This is followed by applications of these concepts to understanding some

J. Gordon Leishman Frontmatter More Information

xxvi

Preface to the First Edition

of the special requirements and characteristics of rotor airfoils. Again, liberal reference to experimental measurements makes the present treatment relatively unique amongst textbooks on helicopters. Chapters 8 and 9 comprise a comprehensive discussion on unsteady aerodynamics, with emphasis on the relevance to helicopter problems. Classical techniques of unsteady aerodynamics, including Theodorsen’s theory and Loewy’s theory, the indicial response method, and dynamic inﬂow, are reviewed in Chapter 8. Extensions of some of these methods to the compressible ﬂow problems found on helicopters are described, with validation with experimental measurements where possible. Indicial methods are treated in some detail because they form the foundation for many modern methods of helicopter analysis and are not covered in any previous helicopter text. Chapter 9 discusses the problem of dynamic stall, which is known to be a barrier to attaining high speed forward ﬂight with a conventional helicopter. Engineering methods of dynamic stall prediction are also reviewed, along with some examples of the general predictive capability to be found with these models. The physical nature of helicopter rotor wakes, both in hovering and forward ﬂight, are discussed in Chapter 10. Nearly all of what we know about rotor wakes comes from empirical observations, and this has led to the development of well-validated mathematical models of the rotor wake using vortex techniques. Chapter 10 concludes with a brief discussion on interactional aerodynamics. Although many the problems of rotor aerodynamics can be studied by considering the rotor in isolation to the fuselage, tail rotor and empennage, aerodynamic interactions between the components lead to many problems that are not yet fully understood. A word about systems of units is in order. In the preparation of this book, I have found it necessary to use both the British (Imperial) and metric (SI) systems. Any preference for one over the other is done simply for the sake of convenience, and I think there is really little reason to change units for the sake of standardization in the text. As aerospace engineers, and particularly helicopter engineers, we are used to working with both systems and even with mixed units in the same breath, and so most readers will have no problems with this approach. For the foreseeable future, students will have to learn to become fully conversant in both systems. Where I have felt it would be helpful, units in both systems are stated. For convenience, a table of conversion factors is also included in the appendix. Having said all this and made my excuses, I hope this edition of the book will be judged, especially by the practicing engineer, on what it contains and not on what else could potentially have been included. Like everything in aviation, the ﬁnal product is always a compromise and is always under a continuous state of revision, development, and improvement. As a ﬁnal comment, it seems appropriate to quote Igor Sikorsky who has said: “At that time [1908] aeronautics was neither an industry nor even a science . . . it was an art, I might say, a passion. Indeed at that time it was a miracle.” As we stand now at the turn of the new millennium, I’m sure that if he was alive Igor Sikorsky would have agreed that the past century has indeed seen many miracles, both in aeronautics and in helicopter technology. The new century will almost certainly see more.

J. Gordon Leishman Frontmatter More Information

Acknowledgments

This book is the product of an opportunity afforded to me by the University of Maryland. The research and writing of the ﬁrst edition of this book was unique experience and has led to many lessons. It was gratifying to see how well the ﬁrst edition was received, with feedback being received from the four corners of the world. The ﬁrst edition took the best part of 2,500 desk hours, mainly during evenings and weekends and spread over three years. In this regard, I am forever grateful to my wife, Alice Marie Leishman, for her love and understanding, and for providing me continuous support where writing, proofreading, drawing ﬁgures, and plotting graphs meant many long nights and all too short weekends. Preparing this second edition required about 1,300 desk hours, spread over about eighteen months, but ultimately even with the beneﬁt of experience and hindsight, it proved to be no lesser a task than the ﬁrst edition. Special acknowledgment is due to a great many people, both on and off the University of Maryland campus. I am most grateful for the council of my colleagues at the University of Maryland, Professors Alfred Gessow (now deceased), Inderjit Chopra, Roberto Celi, James Baeder, Christopher Cadou, and Dr. Vengalattore Nagaraj. Professor Gessow had read substantial parts of the ﬁrst edition and offered many useful suggestions for improvement. Before he passed away in May of 2002, Alfred Gessow and I had many signiﬁcant discussions about the autogiro and the role it played in the fundamental development of the helicopter, from the point of view of both engineering theory and practice. Chapter 12 in this second edition reﬂects the spirit of our discussions, and I dedicate this chapter to his memory. Al Gessow’s knowledge and ﬁrst-hand experience working at NACA Langley during the 1940s and 1950s in the technical development of early helicopters was also invaluable to me. Professors Inderjit Chopra and Roberto Celi provided good suggestions for Chapter 4 and kindly allowed me to use some of their own course material on blade motion and rotor trim. Professor James Baeder read Chapter 8 on unsteady aerodynamics and Chapter 14 on computational methods for helicopter applications, and I am grateful to him and his students for their willingness to help me with ﬁgure preparation. Dr. Chris Cadou read the revised parts of Chapter 1, and I enjoyed our engaging conversations on early engine development and the history of aircraft technology in general. Dr. Nagaraj read the revised Chapters 5 and 6 and offered more good suggestions on helicopter design issues. I am particularly grateful to Dr. Richard Brown of Imperial College at the University of London who is responsible for a substantial part of the writing in Chapter 14 of the second edition. This chapter on computational methods proved to be one of the most satisfying to write, although it took the longest amount of time out of the four new chapters with numerous iterations required to ﬁnally deﬁne the appropriate content. Dr. Brown also provided useful feedback for improvement on the other three new chapters. I am indebted to my graduate students at the University of Maryland, both past and current, who have both directly and indirectly contributed to the content of both the ﬁrst and second editions of this book. The current members of my research group, namely Shreyas Ananthan, Sandeep Gupta, Arun Jose, Robin Preator, and Manikandan Ramasamy, xxvii

J. Gordon Leishman Frontmatter More Information

xxviii

Acknowledgments

enthusiastically read many parts of the second edition and offered useful advice and constructive criticism from a student’s perspective. Many of the students in my classes proofread drafts of revised chapters, and I am particularly grateful to the group of Moble Benedict, Brandon Bush, Mamta Jangid, Nitin Gupta, Vinit Gupta, Vinod Lakshminarayan, and Eric Schroeder for their careful reading and checking of equations. Sandeep Gupta did much to help me reﬁne the material in Chapter 13 on wind turbines. Yik-Loon Lee and Karthikeyan Duraisamy spent time with me explaining the intricacies of advanced computational methods and grid generation techniques, and I appreciate them lending me some of the results from their own research for Chapter 14 in the second edition of this book. I am grateful to Dr. Mahendra Bhagwat (now at US Army/NASA Ames) for running the prescribedand free-vortex wake solutions and for allowing me to adapt his analysis of the tip vortex aging problem in Chapter 10. Dr. Preston Martin (also now at US Army/NASA Ames) took several of the rotor wake visualization images for Chapter 10 and did much to help me ﬁnd references and other material on airfoils and airfoil design for Chapter 7. Jacob Park (now at Bell Helicopter Textron) ran several of the rotor wake calculations, sifted through lots of experimental results on rotor wakes, and organized the data for a good number of the ﬁgures in Chapter 10. Gregory Pugliese read many of the draft chapters for the ﬁrst edition, made endless trips to the library to hunt down reports, cross checked more than several hundred references, and helped me turn more than a few of my many crude sketches into professional looking ﬁgures. Other former students who have directly or indirectly contributed material used in this book include Dr. Ashish Bagai (Sikorsky), Dr. Peter Bi (US Navy NSWC), Dr. Gilbert Crouse (Da Vinci Technologies), Dr. Berend van der Wall (DLR), Mark Daghir (Boeing), Erwin Moederesheim, Joe Tyler, Alan Coyne (Sikorsky), Col. Keith Robinson (US Army), Greg Pugliese, and Dan Grifﬁths (Sikorsky). A special thanks to Dr. Ashish Bagai and Dr. Mahendra Bhagwat for their generosity in allowing me to adapt some of their results and graphics for Chapter 10. Dr. Peter Bi provided me with good feedback on the new content included in Chapter 11. Off the university campus there are a great many people and colleagues who provided advice during the writing of both the ﬁrst and second editions. I remain indebted to the reviewers of my original book proposal, who were able to make good suggestions on what should be included and what should be left out, particularly Prof. Edward Smith and Dr. Andrew Lemnios. The draft second edition saw many more critical eyes, and I was humbled by the care and thoroughness that each reviewer took over their respective chapters. All of their comments were duly taken into account and they led to a better book as a result. Many other people made suggestions on the originally published text or recommended topics for the second edition. Some colleagues read draft copies of the new chapters, while still others sent me hard-to-ﬁnd reports, computer ﬁles of experimental data, ﬁgures, photographs, or other information that was used or adapted for the book. In particular, I appreciate the contributions of Douglas Baldwin, Tom Beddoes, Dr. John Berry, Dr. Bill Bousman, Dr. Richard Brown, Marshall Buhl, Tim Candsdale, Dr. Mark Chafﬁn, Prof. Muguru Chandrasekhara, Dr. Bruce Charnov, Dr. Colin Coleman, Roger Connor, Shawn Coyle, Prof. George Done, Lee-Jay Fingersh, Prof. Peretz Friedmann, Susan Gorton, Dr. Richard Green, David Groen, Jay Groen, Robert Hansford, Franklin Harris, Jennifer Henderson, Michael Hirschberg, Dr. Stuart Houston, Dr. Wayne Johnson, Markus Krekel, Andrew Line, James Mayﬁeld, Dr. Kenneth McAlister, Dr. Patrick Moriarty, Dr. Khanh Nguyen, James Mayﬁeld, Dr. Reinert Müller, Prof. Gareth Padﬁeld, John Perry, Prof. Ganesh Rajagopalan, Dr. Rex Rivolo, Dr. Scott Schreck, Prof. Michael Sellig, Sergei Sikorsky, Dr. Roger Strawn, Prof. Edward Smith, Dr. James Tangler, Dr. Chee Tung, Dr. Hal Youngren, Dr. Daniel Wachspress, and Dr. David Wood. A great number of people, too

J. Gordon Leishman Frontmatter More Information

Acknowledgments

xxix

numerous to mention, sent me letters or e-mail about the ﬁrst edition giving me comments for improvement, pointing out remaining typographical errors, or suggesting areas for further clariﬁcation. Instructors who have used the book and the solutions manual for their classes gave me excellent feedback, which was duly taken into consideration. As they will see in this second edition of the book, most of their suggestions were enthusiastically incorporated and their critiques are most gratefully acknowledged. Thanks are due to the helicopter companies, who were kind enough to send me photographs of their various helicopters or give me permission to publish those that I found in publicity materials and on their web sites. In particular, I acknowledge Madelyn Bush and Jack Satterﬁeld of Boeing Helicopters, Kevin Hale of Bell Helicopter Textron, David Long of Kaman Aircraft Corporation, Eurocopter, and the Public Affairs Department at GKN Westland (now Agusta-Westland). Other photographs were obtained from NASA, the National Renewable Energy Laboratory, Sandia National Laboratories, and the photo archive at Patuxent Naval Air Station. I am also grateful to the staff of the National Air and Space Museum for giving permission to publish some of the historical photographs included in Chapter 1 and to Brian Riddle of the Royal Aeronautical Society in London for ﬁnding some early published papers and reports on autogiros and helicopters. I want to express my sincere gratitude to the staff at Cambridge University Press for their help and support during the writing and publication process of both the ﬁrst and second editions of this book. Florence Padgett was the editor for the ﬁrst edition, and my thanks extend also to Ellen Carlin for her help with the various editing and production issues. Peter Gordon was the editor for the second edition, and I am grateful for his enthusiastic support and encouragement throughout. Finally, my thanks again to the staff at TechBooks for their help in the production of the second edition of this book.

