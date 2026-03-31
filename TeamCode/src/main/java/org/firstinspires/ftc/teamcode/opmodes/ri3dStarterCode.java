package org.firstinspires.ftc.teamcode.opmodes;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.vision.LimelightVision;
import org.firstinspires.ftc.teamcode.math.ShooterModel;

@TeleOp(name = "Ri3D FINAL FIXED", group = "Comp")
public class ri3dStarterCode extends OpMode {

    // ================= ALIGN PD =================
    private double kP_align = 0.06;
    private double kD_align = 0.002;
    private double previousAlignError = 0;

    private final double ALIGN_DEADBAND  = 0.15;
    private final double MAX_ALIGN_POWER = 0.5;

    // ================= SHOOTER =================
    private double launcherTargetTPS = 0;
    private double launcherMin = 0;
    private boolean useLeftModel = true;

    final double FEED_TIME_SECONDS = 1.2;

    // ================= HARDWARE =================
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotorEx leftLauncher, rightLauncher;
    private DcMotor intake1, intake2;
    private CRServo leftFeeder, rightFeeder;
    private Servo diverter;

    private LimelightVision limelight;

    ElapsedTime leftTimer = new ElapsedTime();
    ElapsedTime rightTimer = new ElapsedTime();

    // ================= STATES =================
    private enum LaunchState { IDLE, ALIGN, SPIN_UP, LAUNCH, LAUNCHING }
    private LaunchState leftState = LaunchState.IDLE;
    private LaunchState rightState = LaunchState.IDLE;

    // ================= INIT =================
    @Override
    public void init() {

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftLauncher  = hardwareMap.get(DcMotorEx.class, "left_launcher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "right_launcher");

        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");

        leftFeeder  = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        diverter = hardwareMap.get(Servo.class, "diverter");

        limelight = new LimelightVision(hardwareMap);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        intake1.setDirection(DcMotor.Direction.FORWARD);
        intake2.setDirection(DcMotor.Direction.REVERSE);

        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300,0,0,10));
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300,0,0,10));

        telemetry.addLine("READY");
    }

    // ================= LOOP =================
    @Override
    public void loop() {

        limelight.update();

        // ===== DRIVE =====
        double f = -gamepad2.left_stick_y;
        double s = gamepad2.left_stick_x;
        double r = gamepad2.right_stick_x;
        mecanumDrive(f,s,r);

        // ===== SHOOTER TARGET =====
        if (limelight.hasTarget()) {
            double d = limelight.getDistanceFromArea();
            double rpm = useLeftModel ?
                    ShooterModel.distanceToRPM(d,true) :
                    ShooterModel.distanceToRPM(d,false);

            launcherTargetTPS = rpm * 28.0 / 60.0;
            launcherMin = launcherTargetTPS * 0.95;
        }

        // ===== SHOOT REQUEST =====
        launchLeft(gamepad1.right_bumper);
        launchRight(gamepad1.left_bumper);

        // ===== TELEMETRY =====
        telemetry.addData("Mode", useLeftModel ? "LEFT" : "RIGHT");
        telemetry.addData("Target RPM", launcherTargetTPS * 60 / 28);
        telemetry.addData("Left RPM", leftLauncher.getVelocity()*60/28);
        telemetry.addData("Right RPM", rightLauncher.getVelocity()*60/28);

        if (limelight.hasTarget()) {
            telemetry.addData("Distance", limelight.getDistanceFromArea());
            telemetry.addData("TX", limelight.getTx());
        }
    }

    // ================= ALIGN OFFSET FROM YOUR TABLE =================
    double getTargetOffset(boolean left, double d) {

        if (left) {
            if (d < 70) return -1.0;
            if (d < 90) return -3.0;
            if (d < 110) return -4.5;
            if (d < 130) return -2.5;
            return -2.0;
        } else {
            if (d < 70) return 1.0;
            if (d < 90) return 2.0;
            if (d < 110) return 6.5;
            if (d < 130) return 1.6;
            return 1.0;
        }
    }

    // ================= LEFT SHOOT =================
    void launchLeft(boolean press) {

        switch (leftState) {

            case IDLE:
                if (press && limelight.hasTarget()) {
                    useLeftModel = true;
                    leftState = LaunchState.ALIGN;
                }
                break;

            case ALIGN:
                double d = limelight.getDistanceFromArea();
                double target = getTargetOffset(true, d);
                double error = limelight.getTx() - target;

                double power = -(error * kP_align + (error - previousAlignError) * kD_align);
                power = Math.max(-MAX_ALIGN_POWER, Math.min(MAX_ALIGN_POWER, power));

                mecanumDrive(0,0,power);
                previousAlignError = error;

                if (Math.abs(error) < ALIGN_DEADBAND) {
                    mecanumDrive(0,0,0);
                    leftState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                leftLauncher.setVelocity(launcherTargetTPS);
                rightLauncher.setVelocity(launcherTargetTPS);

                if (leftLauncher.getVelocity() > launcherMin) {
                    leftState = LaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                leftFeeder.setPower(1);
                leftTimer.reset();
                leftState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (leftTimer.seconds() > FEED_TIME_SECONDS) {
                    leftFeeder.setPower(0);
                    leftState = LaunchState.IDLE;
                }
                break;
        }
    }

    // ================= RIGHT SHOOT =================
    void launchRight(boolean press) {

        switch (rightState) {

            case IDLE:
                if (press && limelight.hasTarget()) {
                    useLeftModel = false;
                    rightState = LaunchState.ALIGN;
                }
                break;

            case ALIGN:
                double d = limelight.getDistanceFromArea();
                double target = getTargetOffset(false, d);
                double error = limelight.getTx() - target;

                double power = -(error * kP_align + (error - previousAlignError) * kD_align);
                power = Math.max(-MAX_ALIGN_POWER, Math.min(MAX_ALIGN_POWER, power));

                mecanumDrive(0,0,power);
                previousAlignError = error;

                if (Math.abs(error) < ALIGN_DEADBAND) {
                    mecanumDrive(0,0,0);
                    rightState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                leftLauncher.setVelocity(launcherTargetTPS);
                rightLauncher.setVelocity(launcherTargetTPS);

                if (rightLauncher.getVelocity() > launcherMin) {
                    rightState = LaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                rightFeeder.setPower(1);
                rightTimer.reset();
                rightState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (rightTimer.seconds() > FEED_TIME_SECONDS) {
                    rightFeeder.setPower(0);
                    rightState = LaunchState.IDLE;
                }
                break;
        }
    }

    // ================= DRIVE =================
    void mecanumDrive(double f, double s, double r) {
        double denom = Math.max(Math.abs(f)+Math.abs(s)+Math.abs(r),1);

        leftFrontDrive.setPower((f+s+r)/denom);
        rightFrontDrive.setPower((f-s-r)/denom);
        leftBackDrive.setPower((f-s+r)/denom);
        rightBackDrive.setPower((f+s-r)/denom);
    }
}