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

        vision.update();

        if (vision.hasTarget()) {

            double tx = vision.getTx();
            double ty = vision.getTy();

            // ✅ Correct distance method
            double distance = vision.getBestDistanceMeters();

            // Shooter model
            double targetRPM = ShooterModel.distanceToRPM(distance);

            // Control
            drivetrain.alignToTarget(tx);
            shooter.setRPM(targetRPM);

            // ===============================
            // TELEMETRY
            // ===============================
            telemetry.addLine("=== LIMELIGHT ===");
            telemetry.addData("tx (deg)", "%.2f", tx);
            telemetry.addData("ty (deg)", "%.2f", ty);
            telemetry.addData("ta", "%.2f", vision.getTa());

            telemetry.addLine("=== DISTANCE ===");
            telemetry.addData("Distance (m)", "%.3f", distance);

            telemetry.addLine("=== SHOOTER ===");
            telemetry.addData("Target RPM", "%.1f", targetRPM);
            telemetry.addData("Actual RPM", "%.1f", shooter.getRPM());

        } else {
            drivetrain.stop();
            telemetry.addLine("No target detected");
        }

        telemetry.update();
    }
}
