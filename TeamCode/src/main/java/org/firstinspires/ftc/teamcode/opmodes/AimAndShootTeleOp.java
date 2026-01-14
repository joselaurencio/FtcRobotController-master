package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.LimelightVision;
import org.firstinspires.ftc.teamcode.math.ShooterModel;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp(name = "Aim + Shoot (Drivetrain)")
public class AimAndShootTeleOp extends OpMode {

    private LimelightVision vision;
    private Shooter shooter;
    private Drivetrain drivetrain;

    @Override
    public void init() {
        vision = new LimelightVision(hardwareMap);
        shooter = new Shooter(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);

        telemetry.addLine("Aim + Shoot TeleOp Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ===============================
        // UPDATE VISION (ONCE PER LOOP)
        // ===============================
        vision.update();

        if (vision.hasTarget()) {

            // ===============================
            // VISION VALUES
            // ===============================
            double tx = vision.getTx();
            double ty = vision.getTy();
            double ta = vision.getTa();

            // Distance handling (matches video)
            double distancePose = vision.getDistanceMeters();
            double distanceTrig = vision.getTrigDistanceMeters();
            double distanceUsed = vision.getBestDistanceMeters();

            // ===============================
            // SHOOTER CALCULATION
            // ===============================
            double targetRPM = ShooterModel.distanceToRPM(distanceUsed);

            // ===============================
            // CONTROL
            // ===============================
            drivetrain.alignToTarget(tx);
            shooter.setRPM(targetRPM);

            // ===============================
            // TELEMETRY — LIMELIGHT
            // ===============================
            telemetry.addLine("=== LIMELIGHT ===");
            telemetry.addData("Has Target", true);
            telemetry.addData("tx (deg)", "%.2f", tx);
            telemetry.addData("ty (deg)", "%.2f", ty);
            telemetry.addData("ta", "%.2f", ta);

            // ===============================
            // TELEMETRY — APRILTAG POSE
            // ===============================
            telemetry.addLine("=== APRILTAG POSE ===");
            telemetry.addData("BotPose", vision.getBotPose());

            // ===============================
            // TELEMETRY — DISTANCE
            // ===============================
            telemetry.addLine("=== DISTANCE ===");
            telemetry.addData("Distance (Pose) m", "%.3f", distancePose);
            telemetry.addData("Distance (Trig) m", "%.3f", distanceTrig);
            telemetry.addData("Distance Used m", "%.3f", distanceUsed);
            telemetry.addData(
                    "Distance Source",
                    vision.usingAprilTagDistance() ? "AprilTag Pose" : "Trig (ty)"
            );

            // ===============================
            // TELEMETRY — SHOOTER
            // ===============================
            telemetry.addLine("=== SHOOTER ===");
            telemetry.addData("Target RPM", "%.1f", targetRPM);
            telemetry.addData("Actual RPM", "%.1f", shooter.getRPM());

        } else {

            drivetrain.stop();

            telemetry.addLine("=== LIMELIGHT ===");
            telemetry.addData("Has Target", false);
            telemetry.addLine("Waiting for target...");
        }

        telemetry.update();
    }
}
