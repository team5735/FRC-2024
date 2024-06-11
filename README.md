## This repo contains code for the FRC Team 5735, The Control Freaks, 2024 robot, Squab.

Squab was built for the 2024 FIRST Robotics Competition game, Crescendo, in which robots were tasked with moving 14" foam rings around a field, to be shot or placed into various scoring areas.

Squab was build atop the WPILib Command-Based framework, with documentation [here](https://docs.wpilib.org/en/stable/index.html).

---
Squab contains the following major subsystems:
- A swerve drivetrain
  -	built with Swerve Drive Specialties mk4i
  -	controlled with CTRE's swerve system
- An under-the-bumper intake for notes
   - built with pully belts to smoothly pull the note
   - utilizing a Beam-Break sensor for note tracking
- A wrist-mounted shooter/feeder
  - built with compliant wheels on the shooter and foam rollers for the feeder
  - utilizing a Beam-Break sensor for note tracking
  - utilizing a PWM Through-Bore Encoder for angle feedback
  - controlled with PID Feedback for velocity on the shooter
  - controlled with PID Feedback and Arm Feedforward for angle precision on the wrist
- Two 3-stage climbers
  - built to be linearly moved by winch rotation
  - built with springs to hold the arms rigid
  - utilizing Hall Effect sensors to prevent climber damage
- A LimeLight for vision processing and tracking
