# DECODE (2025-2026) Competition Season

Welcome to our team's official repository for the **FIRST Tech Challenge: DECODE** 2025-2026 competition season. This repository contains the source code for our robot controller, featuring advanced pathing, vision processing, and modular subsystem design.

## Our Codebase

Our code is organized into modular subsystems and mathematical models to ensure reliability and ease of tuning.

### Core Technology Stack

*   **[Pedro Pathing](https://github.com/pedropathing/PedroPathing):** Used for high-performance autonomous navigation and path-following.
*   **GoBilda Pinpoint Localizer:** Integrated with Pedro Pathing for high-accuracy 3-wheel odometry and position tracking.
*   **Limelight 3A Vision:** Leveraged for real-time AprilTag localization, field target alignment, and automatic obelisk pattern detection.
*   **Physics-Grounded Shooter Model:** A custom `ShooterModel` class that uses both theoretical physics (flywheel radius, launch angle, η=0.55 efficiency) and linear regression for precise launcher RPM control.

### Key Subsystems

*   **Mecanum Drivetrain:** A 4-motor drive system optimized for mobility and precise alignment using vision feedback.
*   **Dual-Flywheel Shooter:** Powered by high-speed motors (`left_launcher`, `right_launcher`) and fed by continuous rotation servos (`left_feeder`, `right_feeder`).
*   **Vision-Assisted Aiming:** Real-time distance estimation based on Limelight target area, feeding directly into our shooter's power model.

## OpModes

### Autonomous
Our autonomous programs utilize **Pedro Pathing** for consistent and repeatable scoring.
*   **DECODE Competition Auto BLUE:** Features initial intake, obelisk pattern reading, and precise shot sequences (e.g., RLL/LRL/LLR).
*   **PedroAutonomousBC:** A "Blue Close" scoring path focused on speed and sample delivery.

### TeleOp
*   **Aim + Shoot TeleOp:** A vision-centric drive mode where the robot automatically aligns to targets and calculates optimal shooter RPM based on distance.
*   **ManualAndVisionTeleOp:** Full control for the driver with optional vision assistance for crucial scoring moments.

## Development & Tuning

We use several specialized modes for robot calibration:
*   **CalibrationMode:** Used for gathering data points for our distance-to-area and distance-to-RPM models.
*   **Pedro Pathing Tuning:** Full suite of tuning OpModes (Lateral, Heading, Drive, etc.) for maximizing our follower's accuracy.

## Resources & Credits

This project is built on the [FTC SDK](https://github.com/FIRST-Tech-Challenge/FtcRobotController). 
Special thanks to the authors of the following libraries:
*   **Baron Henderson (20077 The Indubitables):** For Pedro Pathing.
*   **Scott's Bots (10158):** For early Pedro tuning contributions.

---
*Good luck to all teams in the DECODE season!*
