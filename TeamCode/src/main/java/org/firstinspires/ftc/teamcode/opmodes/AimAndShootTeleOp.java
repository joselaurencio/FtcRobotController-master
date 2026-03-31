package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.LimelightVision;
import org.firstinspires.ftc.teamcode.math.ShooterModel;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp(name = "Aim + Shoot (Area Distance)", group = "TeleOp")
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
        telemetry.addLine("Distance Method: Target Area (Power Model)");
        telemetry.update();
    }

    @Override
    public void loop() {

        // Update Limelight once per loop
        vision.update();

        shooter.setRPM(6000);

        if (vision.hasTarget()) {

            // Vision measurements
            double tx = vision.getTx();
            double ta = vision.getTa();

            // Distance from calibrated area regression
            double distance = vision.getDistanceFromArea();

            // Convert distance → RPM
            double targetRPM = ShooterModel.distanceToRPM(distance,true) *.1;

            // Align drivetrain and spin shooter
            //drivetrain.alignToTarget(tx);
            //shooter.setRPM(targetRPM);

            // =====================
            // TELEMETRY
            // =====================
            telemetry.addLine("=== LIMELIGHT ===");
            telemetry.addData("Has Target", true);
            telemetry.addData("tx (deg)", "%.2f", tx);
            telemetry.addData("ta", "%.4f", ta);

            telemetry.addLine("=== DISTANCE ===");
            telemetry.addData("Distance", "%.2f", distance);

            telemetry.addLine("=== SHOOTER ===");
            telemetry.addData("Target RPM", "%.1f", targetRPM);
            telemetry.addData("Actual RPM", "%.1f", shooter.getRPM());
            telemetry.addData("At Speed", shooter.atSpeed());

        } else {

            // No target → stop rotation + shooter safely
            drivetrain.stop();
            shooter.setRPM(0);

            telemetry.addLine("=== LIMELIGHT ===");
            telemetry.addData("Has Target", false);
            telemetry.addLine("Waiting for AprilTag...");
        }

        telemetry.update();
    }
}
