<!-- Slide number: 1 -->

![](PicturePlaceholder2.jpg)
# MTE 481 - PRELIMINARY DESIGN PRESENTATION
10/22/2025
Capstone Group 52
Chanuth Weeraratna, Samuel Mpoloka, Adesh Partap Singh, Joseph Bagheri, Ameen Aydan
Photo credit: @bruce.digital

<!-- Slide number: 2 -->
# Presentation Content

Needs Identification
Problem Definition
Design Specification (Criterion, Constraints, Objectives)
Alternate Design Concepts
Selection Matrix
Applicable Patent
Timeline (Gantt chart)
MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  2

<!-- Slide number: 3 -->
# Needs identification
Hands Free Mobility
MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  3

### Notes:
- Close your eyes and imagine, youre a wheelchair user. What kinds of tasks would be easier for you?

<!-- Slide number: 4 -->
# NEEDS IDENTIFICATION

![](PicturePlaceholder8.jpg)
Wheelchair users engaged in day-to-day tasks are fundamentally limited by the need to use their hands for both propulsion and steering.
What activities could always be more enjoyable or easier with access to both hands?
E.g. Holding your kids' hands, opening a door, carrying groceries, etc

![](Picture16.jpg)
Anecdotally:
Adrit Batra from Laurier enjoys street hockey as a hobby but finds that his arms frequently fatigue, and injury is common in wheelchair sports.
The “building up through repetition” nature of propulsion places loads that the shoulder was never optimized to bear over years. (Barry Mason, 2020)
Constant switching between tasks also creates a steeper learning curve for many wheelchair sports, which was a deterrent for Adrit Batra before he took up the hobby.
MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  4

### Notes:
Maybe talk about the degrees of freedom a normal play would have? Why does increasing them make the game more enjoyable? Include a diagram of a human moving in all 3 DOFs.

<!-- Slide number: 5 -->
# NEEDS IDENTIFICATION
The Hidden Cost

A systematic review found that shoulder pain is the most frequent musculoskeletal complaint in wheelchair users, driven largely by overuse, tendinopathy, and rotator cuff tears (Shawn Song, 2022).
Among wheelchair athletes, shoulder complaints are extremely common — reported prevalence ranges from 16% up to 76% across studies. (Omar W Heyward, 2017)
In a study of 296 full-time manual wheelchair users, those playing sports more than 1–2×/week had 75.7% occurrence of rotator cuff tears, vs. 36.3% in non-sport users. (Michael Akbar, 2015)

![A diagram of a rotator cuff tear AI-generated content may be incorrect.](ContentPlaceholder12.jpg)
MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  5

<!-- Slide number: 6 -->
# Needs Statement
Hands-free wheelchair for sports could open another avenue of enjoyment and reduce the rates of injury for wheelchair users engaged in hockey, basketball, or tennis, among other sports.
MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  6

### Notes:
Make it more enjoyable for wheelchair sports by reducing the learning curve, giving a more authentic experience, and reduction in the injuries that occur. Yes, there exists handsfree solutions but non of them are widely adopted and specifically doesn’t serve the domain of sports. By starting in the sports domain, we can focus on a more performance-based wheelchair.

There’s a need to reduce frequent injury occurrence to make wheelchair sports more enjoyable and reduce participation reluctance.

<!-- Slide number: 7 -->
# Problem Definition

MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  7

<!-- Slide number: 8 -->
# PROBLEM DEFINITION

Wheelchair users who are frequently engaged with sports must use their arms to propel, steer and handle the ball.
The intensity and frequency of these movements leave the conventional wheelchair user vulnerable to injuries.
The wheelchair also constrains the user's agility and natural gameplay, creating a larger learning curve and a reluctance to take up the sport.
MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  8

<!-- Slide number: 9 -->
# PROBLEM DEFINITION

![](PicturePlaceholder2.jpg)
Will focus on basketball as the environment and objects of interaction are easy to define and much narrower in focus
MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  9

<!-- Slide number: 10 -->
# USER REQUIREMENTS
Vague, non-measurable requirements tied to the user and count towards the selection matrix

Human-machine interface doesn’t require input from the hands or arms
Method of operation should not interfere with gameplay
E.g. the user takes their eyes off the ball
Mimic the full range of motion of an able-bodied player: 2 degrees of translation and 1 degree of rotation
‘mimic natural gameplay” problem def’n
Control method should account for a varying range of disabilities
Operation should be simple and intuitive
Including user control sensitivity adjustment
Wheelchair should be safe for the user to operate and those around them
Failsafe measures
Speed limiter
Passive and/or active protection from collisions
Emergency stop
MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  10

### Notes:
Presented to a wheel chair user who also had something to say about this. Maybe remove user preference
Should be able to seat anyone and be comfortable
Comfort of use rating.
Must be testable in controlled conditions
Slalom run is a great metric for determining the overall maneuverability of the wheelchair.
Need to consider the environment and the user:
What is a safe stopping distance?
What about the collisions expected during normal game play?
Operation of the wheelchair must be simple and intuitive so that normal people can use it!
Subjective form of measurement? (NASA tlx)
Measurable: Time to understand (time taken to achieve 80% of max control proficiency). Match that of the hoverboard?
Maybe it’s more important that the control is fast enough for game environments?

<!-- Slide number: 11 -->
# Engineering Requirements
Criterion #1

Weight capacity should accommodate 50th to 95th percentile man (Weight Percentile Calculator)
| 50th – percentile weight | Approx 80kg |
| --- | --- |
| 95th –percentile weight | Approx 110kg |
MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  11

<!-- Slide number: 12 -->
# Engineering Requirements
CDC Anthropometric Reference Data

![](Picture16.jpg)

![](Picture18.jpg)

Criterion #2
The wheelchair itself must not get in the way when handling game objects (e.g. basketballs, hockey stick)
Justified as the length outside a cross section to touch a ball
90th – percentile hip breadth is 36.82 cm - Objective
50th – percentile grip reach is 74.95 cm - Constraint

Basketball
Cross section Radius = Arm length - basketball diameter
Cross Section Radius
MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  12

### Notes:
Label figures
https://multisite.eos.ncsu.edu/www-ergocenter-ncsu-edu/wp-content/uploads/sites/18/2016/06/Anthropometric-Detailed-Data-Tables.pdf

Diagram of the cross-sectional area of human

If it’s meant for anything – why are you designing, it around basketball?

<!-- Slide number: 13 -->
# Engineering Requirements
Criterion #3

| Criterion | Constraint | Objective |
| --- | --- | --- |
| Max Speed | 2.3 m/s | 4.75 m/s |
| Acceleration | 0.7 m/s/s | 1.3 m/s/s |
| Angular velocity | Implied in above criteria |  |
| Stopping distance |  |  |
More aggressive performance specifications are better for use in a basketball sports
Metrics shown are meant for all directions of travel
Justified in the following studies
Dynamics of wheelchair basketball (K D Coutts, 1992)
Drag and sprint performance of wheelchair basketball players (K D Coutts, 1992)
Low profile, omni-directional stool
What has already been achieved
MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  13

### Notes:
Show a dude spinning in a wheelchair

Look at a slalom test to determine how fast to turn

<!-- Slide number: 14 -->
# Engineering Requirements

Criterion #4
Criterion #5
A faster response time is better
Human reaction time ≈ 220 ms
Basketball player reaction ≈ 180 ms
Sensor  Controller  Actuator
latency should aim for 40ms

Operating time should aim for the duration of a full game of basketball (40 mins)
This metric will have a minimum of 12 mins per basketball game quarter
MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  14

### Notes:
Response time might not be a relevant metric – need to inquire with tha gangFind sources for criterion 5

<!-- Slide number: 15 -->
# Engineering Requirements – Criteria Table
| # | Criterion | Constraint | Objective |
| --- | --- | --- | --- |
| 1 | Weight | 80kg | 110kg |
| 2 | Footprint Radius | 60cm | 40cm |
| 3 | Max Speed | 2.3 m/s | 4.75 m/s |
| 3 | Acceleration | 0.7 m/s/s | 1.3 m/s/s |
| 4 | Sensor to actuation latency | 40ms | - |
| 5 | Total Operating Time | 12mins | 40mins |
MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  15

### Notes:

<!-- Slide number: 16 -->
# Alternative Designs & Selection Matrix
MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  16

<!-- Slide number: 17 -->
# ALTERNATE DESIGNS CONSIDERED
Alternative 1: Omni-directional Ball Drive System

![](Picture10.jpg)
Description: Single large ball mechanism providing 360° movement capability through body weight shifting
Engineering Strengths: True omni-directional movement, intuitive control, compact design, minimal external protrusions
Engineering Weaknesses: Complex mechanical system, potential stability challenges, high manufacturing cost, maintenance requirements
MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  17

<!-- Slide number: 18 -->
# ALTERNATE DESIGNS CONSIDERED
Alternative 2: Segway-Style Self-Balancing Platform

![A green and black vehicle with a black wheel AI-generated content may be incorrect.](Picture6.jpg)
Description: Two-wheel self-balancing system with gyroscopic stabilization and lean-to-steer control
Engineering Strengths: Proven technology, responsive control, excellent stability, established manufacturing
Engineering Weaknesses: Limited to forward/backward movement, requires continuous power, complex electronics, limited sports compatibility
MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  18

<!-- Slide number: 19 -->
# ALTERNATE DESIGNS CONSIDERED

![](Picture3.jpg)
Alternative 3: Mecanum Wheel System
Description: Four wheels with rollers at 45° angles enabling omni-directional movement through differential drive
Engineering Strengths: Precise control, good load distribution, proven in robotics, modular design
Engineering Weaknesses: Complex wheel design, maintenance intensive, limited to smooth surfaces, high manufacturing cost
MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  19

<!-- Slide number: 20 -->
# ALTERNATE DESIGNS CONSIDERED

![](Picture6.jpg)
Alternative 4: Track-Based System with Pivot Mechanism
Description: Continuous track system with central pivot for rotation, tank-style movement with independent track control
Engineering Strengths: Excellent traction, simple control logic, robust design, low maintenance
Engineering Weaknesses: Limited to 2 DOF movement, turning radius limitations, high friction, poor surface compatibility

MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  20

<!-- Slide number: 21 -->
# DESIGN SELECTION WEIGHTING JUSTIFICATION
| Criteria | Weight | Importance | Rationale |
| --- | --- | --- | --- |
| Hands-Free Operation | 25% | Primary requirement for sports applications and day-to-day tasks | Enables simultaneous equipment handling during gameplay |
| Non-Interfering Controls | 20% | Critical for equipment handling and gameplay | Control method must not distract from sports performance |
| Design Complexity | 20% | Essential for manufacturability and maintenance | Simpler designs reduce fabrication costs and improve reliability |
| Inherit Stability | 15% | Important for safety and user confidence | Natural stability reduces control complexity and power requirements |
| Safety Operation | 10% | Fundamental requirement for user protection | System must maintain stability during collisions |
| Intuitive Operation | 5% | Important for user adoption and training | Natural control methods reduce learning curve |
| Full Mobility Range | 5% | Desired but secondary to core functionality | Omni-directional movement provides competitive advantage |
MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  21

<!-- Slide number: 22 -->
# DESIGN SELECTION MATRIX
| Criteria | Weight | Ball Drive | Segway | Mecanum | Track System |
| --- | --- | --- | --- | --- | --- |
| Hands-Free Operation | 25% | 9 | 8 | 9 | 7 |
| Non-Interfering Controls | 20% | 9 | 7 | 8 | 7 |
| Design Complexity | 20% | 7 | 6 | 5 | 8 |
| Inherent Stabilit | 15% | 8 | 9 | 7 | 9 |
| Safety Operation | 10% | 8 | 9 | 8 | 9 |
| Intuitive Operation | 5% | 9 | 8 | 7 | 8 |
| Full Mobility Range | 5% | 10 | 6 | 10 | 6 |
| Weighted Score |  | 8.3 | 7.5 | 7.7 | 7.7 |
MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  22

<!-- Slide number: 23 -->
# Relevant Patents
Joseph and Adesh
MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  23

<!-- Slide number: 24 -->
# PATENT REVIEW

![A drawing of a tower AI-generated content may be incorrect.](ContentPlaceholder2.jpg)

![A drawing of a multi-dir device AI-generated content may be incorrect.](PicturePlaceholder23.jpg)

![A black and white drawing of a computer device AI-generated content may be incorrect.](Picture12.jpg)
MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  24

<!-- Slide number: 25 -->
# LOW-PROFILE AND HIGH-LOAD BALL-BALANCING ROLLING SYSTEM

![A diagram of a machine AI-generated content may be incorrect.](Picture8.jpg)
This patent, US12,048,655B2, describes a low-profile, high-load, a hands-free ball-balancing omnidirectional rolling system designed by the University of Illinois.
It is designed as an intuitive and self-balancing mobility device for a person with lower-limb disabilities.
The system offers omnidirectional motion (360 degrees) and advanced driving assistance features like obstacle avoidance and semi-autonomous navigation.
This patent shows the constraints needed to:
maintain stability while being hands-free.
ensure safety in case of power shut down.
This patent also shows a potential room for innovation:
In increasing the motor speeds for sports usage.
Alternative motor contact angles on the sphere

![A drawing of a chair AI-generated content may be incorrect.](Picture1.jpg)
MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  25

<!-- Slide number: 26 -->
# Single wheeled robot Balancing robot, Patent: US 7,847,504B2
This patent shows the constraints needed to:​

Move the ball using an inverted mouse drive and pulley system
Demonstrates useful initial opposite motion needed to get the angular tilt needed to go in that direction
Good orientation of motor contact to simplify motion

This patent also shows a potential room for innovation:​

Changing the design to meet a wheelchair application

![A drawing of a tower AI-generated content may be incorrect.](ContentPlaceholder2.jpg)

![A drawing of a machine AI-generated content may be incorrect.](ContentPlaceholder6.jpg)
MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  26

### Notes:

<!-- Slide number: 27 -->
# BALL - BALANCING ROBOT AND DRIVE ASSEMBLY

![A drawing of a multi-dir device AI-generated content may be incorrect.](PicturePlaceholder23.jpg)
The application discloses a Ball-Balancing Robot (BBR) and its drive assembly designed for dynamic stability and mobility.
The key innovation is the drive assembly, which utilizes three omniwheel assemblies positioned at 120° radial spacings with their planes of rotation being mutually-orthogonal.
This orthogonal arrangement effectively decouples the wheel motion for independent control and reduces the chance of slippage by contacting the ball in the mid-latitude range (near the equator).

This patent shows the constraints needed to:​​

​Properly control and stabilizes the robot with the control system displayed
Reduce angular slippage using a particular tilt in the roller wheels
Enable twist motion as well
​
This patent also shows a potential room for:
Changing the contact angle of the motor driving wheels on the ball further to simplify control system design
MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  27

### Notes:

<!-- Slide number: 28 -->
# Timeline
Gantt chart
MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  28

<!-- Slide number: 29 -->

![A screenshot of a calendar AI-generated content may be incorrect.](Picture3.jpg)
MTE 481 - PRELIMINARY DESIGN PRESENTATION
PAGE  29