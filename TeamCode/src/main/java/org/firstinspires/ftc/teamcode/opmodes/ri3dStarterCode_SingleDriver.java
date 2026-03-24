package org.firstinspires.ftc.teamcode.opmodes;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.vision.LimelightVision;
import org.firstinspires.ftc.teamcode.math.ShooterModel;

@TeleOp(name = "DECODE Ri3D (Single Driver)", group = "StarterBot") // ===== CHANGED =====
public class ri3dStarterCode_SingleDriver extends OpMode {

    // ================= AUTO ALIGN PD =================
    private double kP_align = 0.02;
    private double kD_align = 0.003;
    private double previousAlignError = 0;

    private final double ALIGN_DEADBAND  = 0.5;
    private final double MAX_ALIGN_POWER = 0.4;

    // ================= OFFSET =================
    private final double BASE_OFFSET     = -1.0;
    private final double DISTANCE_FACTOR =  0.002;
    private final double SHOOTER_OFFSET  =  0.8;

    final double FEED_TIME_SECONDS = 5;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;

    final double RPM_CLOSE_TARGET = 2000;
    final double RPM_CLOSE_MIN = 1900;

    final double LAUNCHER_CLOSE_TARGET_VELOCITY = RPM_CLOSE_TARGET * 28 / 60.0;
    final double LAUNCHER_CLOSE_MIN_VELOCITY = RPM_CLOSE_MIN * 28 / 60.0;

    final double RPM_FAR_TARGET = 3200;
    final double RPM_FAR_MIN = 3000;

    final double LAUNCHER_FAR_TARGET_VELOCITY = RPM_FAR_TARGET * 28 / 60.0;
    final double LAUNCHER_FAR_MIN_VELOCITY = RPM_FAR_MIN * 28 / 60.0;

    double launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY;
    double launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;

    final double LEFT_POSITION = 0.2962;
    final double RIGHT_POSITION = 0;

    private boolean lastBack = false;

    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotorEx leftLauncher, rightLauncher;
    private DcMotor intake;
    private CRServo leftFeeder, rightFeeder;
    private Servo diverter;

    private LimelightVision limelight;

    ElapsedTime leftFeederTimer = new ElapsedTime();
    ElapsedTime rightFeederTimer = new ElapsedTime();

    private enum LaunchState { IDLE, ALIGN, SPIN_UP, LAUNCH, LAUNCHING }
    private LaunchState leftLaunchState, rightLaunchState;

    private enum DiverterDirection { LEFT, RIGHT }
    private DiverterDirection diverterDirection = DiverterDirection.LEFT;

    private enum IntakeState { ON, OFF }
    private IntakeState intakeState = IntakeState.OFF;

    private enum ShooterMode { MANUAL, VISION }
    private ShooterMode shooterMode = ShooterMode.VISION;

    private enum LauncherDistance { CLOSE, FAR }
    private LauncherDistance launcherDistance = LauncherDistance.CLOSE;

    double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;

    @Override
    public void init() {
        leftLaunchState = LaunchState.IDLE;
        rightLaunchState = LaunchState.IDLE;

        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftLauncher = hardwareMap.get(DcMotorEx.class, "left_launcher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "right_launcher");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        diverter = hardwareMap.get(Servo.class, "diverter");

        limelight = new LimelightVision(hardwareMap);

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
    }

    @Override
    public void loop() {

        limelight.update();

        // ===== SINGLE DRIVER DRIVE (gamepad1 instead of gamepad2) =====
        double forward = gamepad1.left_stick_y;   // ===== CHANGED =====
        double strafe  = -gamepad1.left_stick_x;  // ===== CHANGED =====
        double rotate  = -gamepad1.right_stick_x; // ===== CHANGED =====

        // ===== ALIGN LOCK =====
        boolean isAligning = (leftLaunchState == LaunchState.ALIGN || rightLaunchState == LaunchState.ALIGN);

        if (!isAligning) {
            mecanumDrive(forward, strafe, rotate);
        } else {
            mecanumDrive(forward, strafe, 0); // ===== NEW ===== allow move but not rotate
        }

        // ===== AUTO ALIGN (trigger) =====
        if (gamepad1.right_trigger > 0.5 && limelight.hasTarget()) {

            double error = -limelight.getTx();
            double derivative = error - previousAlignError;

            double output = (error * kP_align) + (derivative * kD_align);

            if (Math.abs(error) < ALIGN_DEADBAND) output = 0;

            output = Math.max(-MAX_ALIGN_POWER, Math.min(MAX_ALIGN_POWER, output));

            mecanumDrive(0, 0, output);

            previousAlignError = error;
        }

        // ===== SHOOTING =====
        launchLeft(gamepad1.right_bumper);
        launchRight(gamepad1.left_bumper);

        // ===== INTAKE =====
        if (gamepad1.a) intake.setPower(1);
        else if (gamepad1.x) intake.setPower(-1);
        else intake.setPower(0);

        // ===== TELEMETRY =====
        telemetry.addData("Left State", leftLaunchState);
        telemetry.addData("Right State", rightLaunchState);

        if (limelight.hasTarget()) {
            telemetry.addData("tx", limelight.getTx());
        }
    }

    void mecanumDrive(double f, double s, double r) {
        double d = Math.max(Math.abs(f) + Math.abs(s) + Math.abs(r), 1);

        leftFrontDrive.setPower((f + s + r) / d);
        rightFrontDrive.setPower((f - s - r) / d);
        leftBackDrive.setPower((f - s + r) / d);
        rightBackDrive.setPower((f + s - r) / d);
    }

    double getTargetOffset(boolean left) {
        double d = limelight.hasTarget() ? limelight.getDistanceFromArea() : 0;
        double offset = BASE_OFFSET + d * DISTANCE_FACTOR;
        return left ? offset - SHOOTER_OFFSET : offset + SHOOTER_OFFSET;
    }

    boolean isAligned(boolean left) {
        if (!limelight.hasTarget()) return false;
        return Math.abs(limelight.getTx() - getTargetOffset(left)) < ALIGN_DEADBAND;
    }

    void launchLeft(boolean shoot) {
        switch (leftLaunchState) {

            case IDLE:
                if (shoot && limelight.hasTarget()) leftLaunchState = LaunchState.ALIGN;
                break;

            case ALIGN:
                double err = limelight.getTx() - getTargetOffset(true);
                double power = -err * kP_align;
                mecanumDrive(0, 0, power);

                if (isAligned(true)) leftLaunchState = LaunchState.SPIN_UP;
                break;

            case SPIN_UP:
                leftLauncher.setVelocity(launcherTarget);
                rightLauncher.setVelocity(launcherTarget);
                if (leftLauncher.getVelocity() > launcherMin)
                    leftLaunchState = LaunchState.LAUNCH;
                break;

            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                leftFeederTimer.reset();
                leftLaunchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (leftFeederTimer.seconds() > FEED_TIME_SECONDS) {
                    leftFeeder.setPower(0);
                    leftLaunchState = LaunchState.IDLE;
                }
                break;
        }
    }

    void launchRight(boolean shoot) {
        switch (rightLaunchState) {

            case IDLE:
                if (shoot && limelight.hasTarget()) rightLaunchState = LaunchState.ALIGN;
                break;

            case ALIGN:
                double err = limelight.getTx() - getTargetOffset(false);
                double power = -err * kP_align;
                mecanumDrive(0, 0, power);

                if (isAligned(false)) rightLaunchState = LaunchState.SPIN_UP;
                break;

            case SPIN_UP:
                leftLauncher.setVelocity(launcherTarget);
                rightLauncher.setVelocity(launcherTarget);
                if (rightLauncher.getVelocity() > launcherMin)
                    rightLaunchState = LaunchState.LAUNCH;
                break;

            case LAUNCH:
                rightFeeder.setPower(FULL_SPEED);
                rightFeederTimer.reset();
                rightLaunchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (rightFeederTimer.seconds() > FEED_TIME_SECONDS) {
                    rightFeeder.setPower(0);
                    rightLaunchState = LaunchState.IDLE;
                }
                break;
        }
    }
}