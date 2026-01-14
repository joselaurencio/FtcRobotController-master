package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@TeleOp(name = "Sensor: Limelight Distance Test", group = "Sensor")
public class AprilTagLimelightTest extends OpMode {

    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {

        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {

            double ta = llResult.getTa();
            double tx = llResult.getTx();
            double distanceCm = getDistanceFromTag(ta);

            telemetry.addData("Has Target", true);
            telemetry.addData("ta", "%.4f", ta);
            telemetry.addData("tx (deg)", "%.2f", tx);
            telemetry.addData("Distance (cm)", distanceCm);
        } else {
            telemetry.addData("Has Target", false);
        }

        telemetry.update();
    }

    public double getDistanceFromTag(double ta) {
        double scale = 41061.19;
        double distance = scale/ta;
        return distance;
    }
}
