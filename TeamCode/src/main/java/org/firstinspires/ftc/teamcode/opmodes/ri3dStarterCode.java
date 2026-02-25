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
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp(name = "DECODE Ri3D", group = "StarterBot")
//@Disabled
public class ri3dStarterCode extends OpMode {
    // ================= AUTO ALIGN PD =================
    private double kP_heading = 0.02;   // Tune this
    private double kD_heading = 0.003;  // Tune this

    private double previousHeadingError = 0;

    private final double HEADING_DEADBAND = 1.0; // degrees
    private final double MAX_ROTATE_POWER = 0.6; // safety clamp
    // ================================================
    final double FEED_TIME_SECONDS = .8; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    final double RPM_CLOSE_TARGET = 2000;
    final double RPM_CLOSE_MIN = 1900;

    final double LAUNCHER_CLOSE_TARGET_VELOCITY = RPM_CLOSE_TARGET * 28 / 60.0; //in ticks/second for the close goal.
    final double LAUNCHER_CLOSE_MIN_VELOCITY = RPM_CLOSE_MIN * 28 / 60.0; //minimum required to start a shot for close goal.

    final double RPM_FAR_TARGET = 3200;
    final double RPM_FAR_MIN = 3000;

    final double LAUNCHER_FAR_TARGET_VELOCITY = RPM_FAR_TARGET * 28 / 60.0; //in ticks/second for the close goal.
    final double LAUNCHER_FAR_MIN_VELOCITY = RPM_FAR_MIN * 28 / 60.0; //minimum required to start a shot for close goal.

    double launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY; //These variables allow
    double launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;

    final double LEFT_POSITION = 0.2962; //the left and right position for the diverter servo
    final double RIGHT_POSITION = 0;
    private boolean lastBack = false;
    private Shooter shooter;
    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx leftLauncher = null;
    private DcMotorEx rightLauncher = null;
    private DcMotor intake = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    private Servo diverter = null;

    private LimelightVision limelight;

    ElapsedTime leftFeederTimer = new ElapsedTime();
    ElapsedTime rightFeederTimer = new ElapsedTime();


    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchState leftLaunchState;
    private LaunchState rightLaunchState;

    private enum DiverterDirection {
        LEFT,
        RIGHT
    }
    private DiverterDirection diverterDirection = DiverterDirection.LEFT;

    private enum IntakeState {
        ON,
        OFF;
    }

    private enum ShooterMode {
        MANUAL,
        VISION
    }
    private ShooterMode shooterMode = ShooterMode.VISION;
    private IntakeState intakeState = IntakeState.OFF;

    private enum LauncherDistance {
        CLOSE,
        FAR;
    }

    private LauncherDistance launcherDistance = LauncherDistance.CLOSE;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
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


        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);



        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        leftLauncher.setZeroPowerBehavior(BRAKE);
        rightLauncher.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        limelight.update();
        // Continuously update RPM in Vision mode
        if (shooterMode == ShooterMode.VISION && limelight.hasTarget()) {

            double distanceCm = limelight.getDistanceFromArea();
            double distanceMeters = distanceCm / 100.0;

            double rpm = ShooterModel.distanceToRPM(distanceCm);

            double ticksPerSecond = rpm * 28.0 / 60.0;

            launcherTarget = ticksPerSecond;
            launcherMin = ticksPerSecond * 0.95;
        }
// Toggle vision mode with BACK button
        if (gamepad1.back && !lastBack) {
            if (shooterMode == ShooterMode.MANUAL) {
                shooterMode = ShooterMode.VISION;
            } else {
                shooterMode = ShooterMode.MANUAL;
            }
        }
        lastBack = gamepad1.back;

        double forward = gamepad2.right_stick_x;
        double strafe = -gamepad2.left_stick_x;
        double rotate = gamepad2.left_stick_y;

// ===== AUTO ALIGN BUTTON =====
        boolean autoAlignActive = gamepad1.right_trigger > 0.5;

        if (autoAlignActive && limelight.hasTarget()) {

            double currentYaw = limelight.getTx(); // MUST return degrees (left/right offset)
            double error = 0 - currentYaw; // target is centered = 0

            double derivative = error - previousHeadingError;

            double output = (error * kP_heading) + (derivative * kD_heading);

            // Deadband to prevent jitter
            if (Math.abs(error) < HEADING_DEADBAND) {
                output = 0;
            }

            // Clamp rotation power
            output = Math.max(-MAX_ROTATE_POWER, Math.min(MAX_ROTATE_POWER, output));

            rotate = output;

            previousHeadingError = error;

        } else {
            previousHeadingError = 0; // reset when not aligning
        }

        // Now drive with modified rotate
        mecanumDrive(forward, strafe, rotate);

        if (gamepad1.y) {
            leftLauncher.setVelocity(launcherTarget);
            rightLauncher.setVelocity(launcherTarget);
        }


        if (gamepad1.dpadDownWasPressed()) {
            switch (diverterDirection){
                case LEFT:
                    diverterDirection = DiverterDirection.RIGHT;
                    diverter.setPosition(RIGHT_POSITION);
                    break;
                case RIGHT:
                    diverterDirection = DiverterDirection.LEFT;
                    diverter.setPosition(LEFT_POSITION);
                    break;
            }
        }

        if (gamepad1.aWasPressed()){
            switch (intakeState){
                case ON:
                    intakeState = IntakeState.OFF;
                    intake.setPower(0);
                    break;
                case OFF:
                    intakeState = IntakeState.ON;
                    intake.setPower(1);
                    break;
            }
        }

        if (gamepad1.xWasPressed()){
            switch (intakeState){
                case ON:
                    intakeState = IntakeState.OFF;
                    intake.setPower(-1);
                    break;
                case OFF:
                    intakeState = IntakeState.ON;
                    intake.setPower(0);
                    break;
            }
        }

        if (gamepad1.dpadUpWasPressed()) {
            switch (launcherDistance) {
                case CLOSE:
                    launcherDistance = LauncherDistance.FAR;
                    launcherTarget = LAUNCHER_FAR_TARGET_VELOCITY;
                    launcherMin = LAUNCHER_FAR_MIN_VELOCITY;
                    break;
                case FAR:
                    launcherDistance = LauncherDistance.CLOSE;
                    launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY;
                    launcherMin = LAUNCHER_CLOSE_MIN_VELOCITY;
                    break;
            }
        }

        /*
         * Now we call our "Launch" function.
         */
        launchLeft(gamepad1.leftBumperWasPressed());
        launchRight(gamepad1.rightBumperWasPressed());

        /*
         * Show the state and motor powers
         */

        final double leftLauncherV = 60 * leftLauncher.getVelocity();
        final double leftLauncherV2 = leftLauncherV/28;

        final double rightLauncherV = 60 * rightLauncher.getVelocity();
        final double rightLauncherV2 = rightLauncherV/28;

        telemetry.addData("Shooter Mode", shooterMode);
        telemetry.addData("Auto Align Active", gamepad1.right_trigger > 0.5);
        if (limelight.hasTarget()) {
            telemetry.addData("Tag Yaw", limelight.getTx());
        }

        telemetry.addData("Vision Target TPS", launcherTarget);
        if (limelight.hasTarget()) {
            telemetry.addData("Distance (cm)", limelight.getDistanceFromArea());
        }

        telemetry.addData("Left Launch State", leftLaunchState);
        telemetry.addData("Right Launch State", rightLaunchState);
        telemetry.addData("launch distance", launcherDistance);
        telemetry.addData("Left Launcher Velocity", leftLauncherV2);
        telemetry.addData("Right Launcher Velocity", rightLauncherV2);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    void mecanumDrive(double forward, double strafe, double rotate){

        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);


        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

//        leftFrontDrive.setPower(leftFrontPower);
//        rightFrontDrive.setPower(rightFrontPower);
//        leftBackDrive.setPower(leftBackPower);
//        rightBackDrive.setPower(rightBackPower);



    }

    void launchLeft(boolean shotRequested) {
        switch (leftLaunchState) {
            case IDLE:
                if (shotRequested) {
                    leftLaunchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                leftLauncher.setVelocity(launcherTarget);
                rightLauncher.setVelocity(launcherTarget);
                if (leftLauncher.getVelocity() > launcherMin) {
                    leftLaunchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                leftFeederTimer.reset();
                leftLaunchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (leftFeederTimer.seconds() > FEED_TIME_SECONDS) {
                    leftLaunchState = LaunchState.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }

    void launchRight(boolean shotRequested) {
        switch (rightLaunchState) {
            case IDLE:
                if (shotRequested) {
                    rightLaunchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                leftLauncher.setVelocity(launcherTarget);
                rightLauncher.setVelocity(launcherTarget);
                if (rightLauncher.getVelocity() > launcherMin) {
                    rightLaunchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                rightFeeder.setPower(FULL_SPEED);
                rightFeederTimer.reset();
                rightLaunchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (rightFeederTimer.seconds() > FEED_TIME_SECONDS) {
                    rightLaunchState = LaunchState.IDLE;
                    rightFeeder.setPower(STOP_SPEED);
                }
                break;
        }
    }
}