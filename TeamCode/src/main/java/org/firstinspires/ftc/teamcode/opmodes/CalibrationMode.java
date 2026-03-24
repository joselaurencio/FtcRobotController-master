package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.vision.LimelightVision;

import java.util.ArrayList;

@TeleOp(name = "CALIBRATION MODE", group = "TEST")
public class CalibrationMode extends OpMode {

    // ===== DRIVE =====
    private DcMotor leftFront, rightFront, leftBack, rightBack;

    // ===== VISION =====
    private LimelightVision limelight;

    // ===== DATA STORAGE =====
    ArrayList<Double> baseOffsets = new ArrayList<>();
    ArrayList<Double> leftOffsets = new ArrayList<>();
    ArrayList<Double> rightOffsets = new ArrayList<>();
    ArrayList<Double> distances = new ArrayList<>();

    // ===== BUTTON EDGE DETECTION =====
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFront = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBack = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBack = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        limelight = new LimelightVision(hardwareMap);

        telemetry.addLine("Calibration Mode Ready");
    }

    @Override
    public void loop() {

        limelight.update();

        // ===== DRIVE =====
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        mecanumDrive(forward, strafe, rotate);

        double tx = 0;
        double distance = 0;

        if (limelight.hasTarget()) {
            tx = limelight.getTx();
            distance = limelight.getDistanceFromArea();
        }

        // ===== RECORD BASE OFFSET =====
        if (gamepad1.a && !lastA && limelight.hasTarget()) {
            baseOffsets.add(tx);
            distances.add(distance);
        }

        // ===== RECORD LEFT SHOOTER =====
        if (gamepad1.b && !lastB && limelight.hasTarget()) {
            leftOffsets.add(tx);
        }

        // ===== RECORD RIGHT SHOOTER =====
        if (gamepad1.x && !lastX && limelight.hasTarget()) {
            rightOffsets.add(tx);
        }

        // ===== CLEAR DATA =====
        if (gamepad1.y && !lastY) {
            baseOffsets.clear();
            leftOffsets.clear();
            rightOffsets.clear();
            distances.clear();
        }

        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;

        // ===== CALCULATIONS =====
        double avgBase = average(baseOffsets);
        double avgLeft = average(leftOffsets);
        double avgRight = average(rightOffsets);

        double suggestedLeftOffset = avgBase - avgLeft;
        double suggestedRightOffset = avgRight - avgBase;

        // ===== TELEMETRY =====
        telemetry.addLine("=== LIVE DATA ===");
        telemetry.addData("tx", tx);
        telemetry.addData("distance (cm)", distance);

        telemetry.addLine("\n=== RECORDED ===");
        telemetry.addData("Base Samples", baseOffsets.size());
        telemetry.addData("Left Samples", leftOffsets.size());
        telemetry.addData("Right Samples", rightOffsets.size());

        telemetry.addLine("\n=== RESULTS ===");
        telemetry.addData("BASE_OFFSET", avgBase);
        telemetry.addData("LEFT_OFFSET", suggestedLeftOffset);
        telemetry.addData("RIGHT_OFFSET", suggestedRightOffset);

        telemetry.addLine("\n=== CONTROLS ===");
        telemetry.addLine("A = Save BASE");
        telemetry.addLine("B = Save LEFT");
        telemetry.addLine("X = Save RIGHT");
        telemetry.addLine("Y = Clear");

        telemetry.update();
    }

    // ===== DRIVE METHOD =====
    void mecanumDrive(double forward, double strafe, double rotate) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        double lf = (forward + strafe + rotate) / denominator;
        double rf = (forward - strafe - rotate) / denominator;
        double lb = (forward - strafe + rotate) / denominator;
        double rb = (forward + strafe - rotate) / denominator;

        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
    }

    // ===== AVERAGE HELPER =====
    double average(ArrayList<Double> list) {
        if (list.size() == 0) return 0;
        double sum = 0;
        for (double v : list) sum += v;
        return sum / list.size();
    }
}