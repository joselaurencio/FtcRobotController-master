# TeamCode Module Technical Guide

Welcome to the **TeamCode** module. This is where the competition-ready logic for our robot resides. 

## 🏗️ Code Architecture

We follow a modular approach to separate hardware control, mathematical modeling, and high-level autonomous logic.

### 🔌 Subsystems
*   **`Drivetrain.java`**: Handles Mecanum drive logic, including alignment to Limelight targets.
*   **`Shooter.java`**: Manages the dual-flywheel launcher (`left_launcher`, `right_launcher`) and the CRServo feeders.

### 📐 Math & Models
*   **`ShooterModel.java`**: A utility class containing our physics-based and regression-based shooter power models. It converts distance (cm) to target RPM.
    *   **Physics Mode**: Uses standard projectile equations with a calibrated efficiency factor (η=0.55).
    *   **Linear Mode**: Uses a regression model (`3.67626 * d + 1746.15732`) for simplified tuning.

### 👁️ Vision
*   **`LimelightVision.java`**: A wrapper for the Limelight 3A sensor.
    *   **Distance Estimation**: Uses a power regression on the target area (`ta`).
    *   **Alignment**: Provides `tx` and `ty` for horizontal and vertical centering.
    *   **Obelisk Detection**: Custom logic for identifying game-specific patterns.

## 📍 Navigation & Pathing

We use **Pedro Pathing** for all autonomous movements. 
*   **`Constants.java`**: Contains PIDF coefficients for translational, heading, and drive movements, as well as GoBilda Pinpoint localizer offsets.
*   **`Tuning.java`**: The main interface for adjusting our follower's coefficients on the fly.

## 🚀 Competition OpModes

*   **`DecodeSimpleAutoBLUE.java` / `DecodeSimpleAutoRED.java`**: Full autonomous state machines that handle intaking, navigation, pattern reading, and scoring.
*   **`AimAndShootTeleOp.java`**: TeleOp mode with automated vision-based aiming and power calculation.
*   **`PedroAutonomousBC.java` / `PedroAutonomousRC.java`**: Optimized scoring paths using Bezier curves.

## 🛠️ How to Add New OpModes

1.  Create a new class in `org.firstinspires.ftc.teamcode.opmodes`.
2.  Extend `LinearOpMode` or `OpMode`.
3.  Add the `@TeleOp` or `@Autonomous` annotation.
4.  Remember to include a `@Disabled` annotation if the code is still in progress.

---
*Maintained by the Software Team.*
