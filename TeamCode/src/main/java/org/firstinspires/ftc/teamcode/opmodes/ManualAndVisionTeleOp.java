package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.vision.LimelightVision;

@TeleOp(name = "Manual + Vision Shooter", group = "Competition")
public class ManualAndVisionTeleOp extends OpMode {

    private Shooter shooter;
    private LimelightVision limelight;

    // =============================
    // Intake Hardware
    // =============================
    private DcMotor intake;
    private CRServo leftFeeder;
    private CRServo rightFeeder;

    private static final double INTAKE_POWER = 1.0;
    private static final double FEED_POWER = 1.0;
    private static final double STOP_POWER = 0.0;
    private static final double FEED_TIME = 0.75;

    private ElapsedTime leftFeedTimer = new ElapsedTime();
    private ElapsedTime rightFeedTimer = new ElapsedTime();

    private boolean leftFeeding = false;
    private boolean rightFeeding = false;

    private boolean intakeOn = false;
    private boolean intakeReverse = false;
    private boolean lastA = false;
    private boolean lastB = false;

    // =============================
    // Shooter Modes
    // =============================
    private enum Mode {
        MANUAL,
        VISION
    }

    private Mode currentMode = Mode.MANUAL;

    private static final double MANUAL_CLOSE_RPM = 1200;
    private static final double MANUAL_FAR_RPM = 1350;

    private boolean lastY = false;
    private boolean lastDpadUp = false;
    private boolean lastX = false;

    @Override
    public void init() {

        shooter = new Shooter(hardwareMap);
        limelight = new LimelightVision(hardwareMap);

        // Intake hardware names MUST match config
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeederServo");

        intake.setPower(0);
        leftFeeder.setPower(STOP_POWER);
        rightFeeder.setPower(STOP_POWER);

        telemetry.addLine("Manual + Vision + Intake Ready");
    }

    @Override
    public void loop() {

        limelight.update();

        // =============================
        // SHOOTER MODE SWITCHING
        // =============================

        if (gamepad1.y && !lastY) {
            currentMode = Mode.MANUAL;
            shooter.setRPM(MANUAL_CLOSE_RPM);
        }

        if (gamepad1.dpad_up && !lastDpadUp) {
            currentMode = Mode.MANUAL;
            shooter.setRPM(MANUAL_FAR_RPM);
        }

        if (gamepad1.x && !lastX) {
            currentMode = Mode.VISION;
        }

        lastY = gamepad1.y;
        lastDpadUp = gamepad1.dpad_up;
        lastX = gamepad1.x;

        // =============================
        // VISION MODE LOGIC
        // =============================

        if (currentMode == Mode.VISION && limelight.hasTarget()) {
            double distance = limelight.getDistanceFromArea();
            double requiredRPM = shooter.getRPM(); // assuming this already calculates internally
            shooter.setRPM(requiredRPM);
        }

        // =============================
        // SHOOTER TRIGGER CONTROL (FIXED)
        // =============================

        if (gamepad1.right_trigger > 0.3) {
            // forward spin
            // RPM already set by mode selection
        } else if (gamepad1.left_trigger > 0.3) {
            shooter.setRPM(-4000); // reverse clear
        } else {
            shooter.setRPM(0);
        }

        // =============================
        // INTAKE TOGGLE (A)
        // =============================

        if (gamepad1.a && !lastA) {
            intakeOn = !intakeOn;
        }
        lastA = gamepad1.a;

        if (intakeOn) {
            intake.setPower(-INTAKE_POWER);
        } else {
            intake.setPower(0);
        }

        if (gamepad1.b && !lastB) {
            intakeReverse = !intakeReverse;
        }
        lastB = gamepad1.b;

        if (intakeReverse) {
            intake.setPower(INTAKE_POWER);
        } else {
            intake.setPower(0);
        }

        // =============================
        // FEEDERS (TIMED BURST)
        // =============================

        // Left feeder
        if (gamepad1.left_bumper && !leftFeeding) {
            leftFeeding = true;
            leftFeedTimer.reset();
            leftFeeder.setPower(FEED_POWER);
        }

        if (leftFeeding && leftFeedTimer.seconds() > FEED_TIME) {
            leftFeeder.setPower(STOP_POWER);
            leftFeeding = false;
        }

        // Right feeder
        if (gamepad1.right_bumper && !rightFeeding) {
            rightFeeding = true;
            rightFeedTimer.reset();
            rightFeeder.setPower(FEED_POWER);
        }

        if (rightFeeding && rightFeedTimer.seconds() > FEED_TIME) {
            rightFeeder.setPower(STOP_POWER);
            rightFeeding = false;
        }

        // =============================
        // TELEMETRY
        // =============================

        telemetry.addData("Mode", currentMode);
        telemetry.addData("Shooter RPM", shooter.getRPM());
        telemetry.addData("Intake On", intakeOn);
        telemetry.addData("Left Feeding", leftFeeding);
        telemetry.addData("Right Feeding", rightFeeding);

        if (limelight.hasTarget()) {
            telemetry.addData("Distance", limelight.getDistanceFromArea());
        }
    }
}
