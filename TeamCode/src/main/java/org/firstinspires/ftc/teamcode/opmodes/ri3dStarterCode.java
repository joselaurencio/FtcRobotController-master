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
    // kP_align: proportional gain — tune this first. Increase until robot snaps to target.
    // kD_align: derivative gain  — tune second. Increase to damp oscillation near target.
    private double kP_align = .08;
    private double kD_align = 0.003;
    private double previousAlignError = 0; // used for derivative term in both shooters

    private final double ALIGN_DEADBAND  = 0.2;  // degrees — ignore error smaller than this
    private final double MAX_ALIGN_POWER = 0.7;  // clamp rotation to prevent overshoot
    // ==================================================

    // ================= DYNAMIC OFFSET TUNING =================
    // BASE_OFFSET:      horizontal bias when camera/robot aren't perfectly centered (degrees)
    // DISTANCE_FACTOR:  how much the offset shifts per cm farther from target
    // SHOOTER_OFFSET:   geometry compensation — left shooter aims left, right aims right
    //
    // To replace with a lookup table later, change getTargetOffset() only.
    private final double BASE_OFFSET     = -3.0;   // degrees
    private final double DISTANCE_FACTOR =  0.002; // degrees per cm
    private final double SHOOTER_OFFSET  =  .1;   // degrees, applied ± per shooter side
    // =========================================================

    final double FEED_TIME_SECONDS = 1.5;   // feeder servos run this long per shot
    final double STOP_SPEED        = 0.0;
    final double FULL_SPEED        = 1.0;

    final double RPM_CLOSE_TARGET = 2000;
    final double RPM_CLOSE_MIN    = 1900;

    final double LAUNCHER_CLOSE_TARGET_VELOCITY = RPM_CLOSE_TARGET * 28 / 60.0;
    final double LAUNCHER_CLOSE_MIN_VELOCITY    = RPM_CLOSE_MIN    * 28 / 60.0;

    final double RPM_FAR_TARGET = 3200;
    final double RPM_FAR_MIN    = 3000;

    final double LAUNCHER_FAR_TARGET_VELOCITY = RPM_FAR_TARGET * 28 / 60.0;
    final double LAUNCHER_FAR_MIN_VELOCITY    = RPM_FAR_MIN    * 28 / 60.0;

    double launcherTarget = LAUNCHER_CLOSE_TARGET_VELOCITY;
    double launcherMin    = LAUNCHER_CLOSE_MIN_VELOCITY;

    final double LEFT_POSITION  = 0.2962;
    final double RIGHT_POSITION = 0;

    private boolean lastBack = false;

    // ===== Hardware =====
    private DcMotor   leftFrontDrive  = null;
    private DcMotor   rightFrontDrive = null;
    private DcMotor   leftBackDrive   = null;
    private DcMotor   rightBackDrive  = null;
    private DcMotorEx leftLauncher    = null;
    private DcMotorEx rightLauncher   = null;
    private DcMotor   intake          = null;
    private CRServo   leftFeeder      = null;
    private CRServo   rightFeeder     = null;
    private Servo     diverter        = null;

    private LimelightVision limelight;

    ElapsedTime leftFeederTimer  = new ElapsedTime();
    ElapsedTime rightFeederTimer = new ElapsedTime();

    // ===== State Enums =====

    private enum LaunchState {
        IDLE,
        ALIGN,      // rotate to dynamic target offset before spinning up
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchState leftLaunchState;
    private LaunchState rightLaunchState;

    private enum DiverterDirection { LEFT, RIGHT }
    private DiverterDirection diverterDirection = DiverterDirection.LEFT;

    private enum IntakeState { ON, OFF }
    private IntakeState intakeState = IntakeState.OFF;

    private enum ShooterMode { MANUAL, VISION }
    private ShooterMode shooterMode = ShooterMode.VISION;

    private enum LauncherDistance { CLOSE, FAR }
    private LauncherDistance launcherDistance = LauncherDistance.CLOSE;

    // Drive power storage (for telemetry)
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    // =========================================================
    // INIT
    // =========================================================
    @Override
    public void init() {
        leftLaunchState  = LaunchState.IDLE;
        rightLaunchState = LaunchState.IDLE;

        leftFrontDrive  = hardwareMap.get(DcMotor.class,   "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class,   "right_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class,   "left_back_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class,   "right_back_drive");
        leftLauncher    = hardwareMap.get(DcMotorEx.class, "left_launcher");
        rightLauncher   = hardwareMap.get(DcMotorEx.class, "right_launcher");
        intake          = hardwareMap.get(DcMotor.class,   "intake");
        leftFeeder      = hardwareMap.get(CRServo.class,   "left_feeder");
        rightFeeder     = hardwareMap.get(CRServo.class,   "right_feeder");
        diverter        = hardwareMap.get(Servo.class,     "diverter");

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
        leftLauncher.setZeroPowerBehavior(BRAKE);
        rightLauncher.setZeroPowerBehavior(BRAKE);

        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10));
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10));

        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {}

    // =========================================================
    // MAIN LOOP
    // =========================================================
    @Override
    public void loop() {

        limelight.update();

        // ===== Vision Mode: continuously update launcher velocity target =====
        if (shooterMode == ShooterMode.VISION && limelight.hasTarget()) {
            double distanceCm     = limelight.getDistanceFromArea();
            double rpm            = ShooterModel.distanceToRPM(distanceCm); // replace with ShooterModel.distanceToRPM(distanceCm) when ready
            double ticksPerSecond = rpm * 28.0 / 60.0;
            launcherTarget = ticksPerSecond;
            launcherMin    = ticksPerSecond * 0.95;
        }

        // ===== Toggle Vision/Manual mode with BACK button =====
        if (gamepad1.back && !lastBack) {
            shooterMode = (shooterMode == ShooterMode.MANUAL)
                    ? ShooterMode.VISION
                    : ShooterMode.MANUAL;
        }
        lastBack = gamepad1.back;

        // ===== Driver input =====
        double forward = gamepad2.left_stick_y;
        double strafe  = -gamepad2.left_stick_x;
        double rotate  = -gamepad2.right_stick_x;

        // ===== Manual auto-align override (right trigger) =====
        // This is separate from the launcher ALIGN state — it lets the driver
        // hold the trigger to continuously rotate toward target center.
        boolean autoAlignActive = gamepad1.right_trigger > 0.5;

        if (autoAlignActive && limelight.hasTarget()) {
            double currentYaw = limelight.getTx();
            double error      = -currentYaw; // aim for tx = 0
            double derivative = error - previousAlignError;
            double output     = (error * kP_align) + (derivative * kD_align);

            if (Math.abs(error) < ALIGN_DEADBAND) output = 0;
            output = Math.max(-MAX_ALIGN_POWER, Math.min(MAX_ALIGN_POWER, output));

            rotate             = output;
            previousAlignError = error;
        } else {
            previousAlignError = 0;
        }

        mecanumDrive(forward, strafe, rotate);

        // ===== Manual flywheel spin-up (hold Y) =====
        if (gamepad1.y) {
            leftLauncher.setVelocity(launcherTarget);
            rightLauncher.setVelocity(launcherTarget);
        }

        // ===== Diverter toggle =====
        if (gamepad1.dpadDownWasPressed()) {
            switch (diverterDirection) {
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

        // ===== Intake toggle (A = forward, X = reverse) =====
        if (gamepad1.aWasPressed()) {
            switch (intakeState) {
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

        if (gamepad1.xWasPressed()) {
            switch (intakeState) {
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

        // ===== Distance preset toggle =====
        if (gamepad1.dpadUpWasPressed()) {
            switch (launcherDistance) {
                case CLOSE:
                    launcherDistance = LauncherDistance.FAR;
                    launcherTarget   = LAUNCHER_FAR_TARGET_VELOCITY;
                    launcherMin      = LAUNCHER_FAR_MIN_VELOCITY;
                    break;
                case FAR:
                    launcherDistance = LauncherDistance.CLOSE;
                    launcherTarget   = LAUNCHER_CLOSE_TARGET_VELOCITY;
                    launcherMin      = LAUNCHER_CLOSE_MIN_VELOCITY;
                    break;
            }
        }

        // ===== Launch state machines =====
        // Right bumper = left shooter, Left bumper = right shooter
        launchLeft(gamepad1.rightBumperWasPressed());
        launchRight(gamepad1.leftBumperWasPressed());

        // ===== Telemetry =====
        final double leftRPM  = leftLauncher.getVelocity()  * 60 / 28;
        final double rightRPM = rightLauncher.getVelocity() * 60 / 28;

        telemetry.addData("Shooter Mode",       shooterMode);
        telemetry.addData("Launcher Distance",  launcherDistance);
        telemetry.addData("Auto Align Active",  autoAlignActive);
        telemetry.addData("Left Launch State",  leftLaunchState);
        telemetry.addData("Right Launch State", rightLaunchState);
        telemetry.addData("Left RPM",           leftRPM);
        telemetry.addData("Right RPM",          rightRPM);
        telemetry.addData("Vision Target TPS",  launcherTarget);

        if (limelight.hasTarget()) {
            telemetry.addData("Limelight tx",       limelight.getTx());
            telemetry.addData("Distance (cm)",      limelight.getDistanceFromArea());
            telemetry.addData("L targetOffset",     getTargetOffset(true));
            telemetry.addData("R targetOffset",     getTargetOffset(false));
        }
    }

    @Override
    public void stop() {}

    // =========================================================
    // MECANUM DRIVE
    // =========================================================
    void mecanumDrive(double forward, double strafe, double rotate) {
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower  = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower   = (forward - strafe + rotate) / denominator;
        rightBackPower  = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    // =========================================================
    // DYNAMIC TARGET OFFSET
    // =========================================================
    /**
     * Returns the ideal Limelight tx value the robot should align to before firing.
     *
     * Formula: BASE_OFFSET + (distanceCm * DISTANCE_FACTOR) ± SHOOTER_OFFSET
     *
     *   BASE_OFFSET:      accounts for camera/robot not being perfectly centered
     *   DISTANCE_FACTOR:  parallax compensation — offset grows slightly at range
     *   SHOOTER_OFFSET:   left flywheel sits left of center, right sits right;
     *                     each needs a slightly different aim angle
     *
     * To switch to a lookup table, replace the formula here only — no state
     * machine changes required.
     *
     * @param isLeftShooter true = left flywheel, false = right flywheel
     * @return target tx in degrees
     */
    double getTargetOffset(boolean isLeftShooter) {
        double distanceCm = (limelight != null && limelight.hasTarget())
                ? limelight.getDistanceFromArea()
                : 0;

        double offset = BASE_OFFSET + (distanceCm * DISTANCE_FACTOR);
        offset += isLeftShooter ? -SHOOTER_OFFSET : SHOOTER_OFFSET;
        return offset;
    }

    // =========================================================
    // ALIGNMENT CHECK
    // =========================================================
    /**
     * Returns true when tx is within ALIGN_DEADBAND of the dynamic target offset.
     *
     * @param isLeftShooter determines which target offset to check against
     */
    public boolean isAligned(boolean isLeftShooter) {
        if (limelight == null || !limelight.hasTarget()) return false;
        double targetOffset = getTargetOffset(isLeftShooter);
        return Math.abs(limelight.getTx() - targetOffset) < ALIGN_DEADBAND;
    }

    // =========================================================
    // LEFT LAUNCH STATE MACHINE
    // State flow: IDLE → ALIGN → SPIN_UP → LAUNCH → LAUNCHING → IDLE
    // =========================================================
    void launchLeft(boolean shotRequested) {
        switch (leftLaunchState) {

            case IDLE:
                // Only enter ALIGN if a shot was requested and we have a vision target
                if (shotRequested && limelight.hasTarget()) {
                    previousAlignError = 0; // reset derivative accumulator
                    leftLaunchState = LaunchState.ALIGN;
                }
                break;

            case ALIGN:
                // If target is lost mid-align, abort safely
                if (!limelight.hasTarget()) {
                    mecanumDrive(0, 0, 0);
                    leftLaunchState = LaunchState.IDLE;
                    break;
                }

                double leftTargetOffset = getTargetOffset(true); // left shooter offset
                double leftTx           = limelight.getTx();
                double leftError        = leftTx - leftTargetOffset;
                double leftDeriv        = leftError - previousAlignError;

                // PD controller — negate so positive error drives robot toward target
                double leftPower = -(leftError * kP_align + leftDeriv * kD_align);

                // Deadband: stop fighting tiny residual error
                if (Math.abs(leftError) < ALIGN_DEADBAND) leftPower = 0;

                // Safety clamp
                leftPower = Math.max(-MAX_ALIGN_POWER, Math.min(MAX_ALIGN_POWER, leftPower));

                mecanumDrive(0, 0, leftPower);
                previousAlignError = leftError;

                // Inline alignment telemetry — useful during tuning
                telemetry.addData("[L ALIGN] tx",           leftTx);
                telemetry.addData("[L ALIGN] targetOffset", leftTargetOffset);
                telemetry.addData("[L ALIGN] error",        leftError);
                telemetry.addData("[L ALIGN] power",        leftPower);

                if (isAligned(true)) {
                    mecanumDrive(0, 0, 0);
                    previousAlignError = 0;
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
                    leftFeeder.setPower(STOP_SPEED);
                    leftLaunchState = LaunchState.IDLE;
                }
                break;
        }
    }

    // =========================================================
    // RIGHT LAUNCH STATE MACHINE
    // State flow: IDLE → ALIGN → SPIN_UP → LAUNCH → LAUNCHING → IDLE
    // =========================================================
    void launchRight(boolean shotRequested) {
        switch (rightLaunchState) {

            case IDLE:
                if (shotRequested && limelight.hasTarget()) {
                    previousAlignError = 0;
                    rightLaunchState = LaunchState.ALIGN;
                }
                break;

            case ALIGN:
                if (!limelight.hasTarget()) {
                    mecanumDrive(0, 0, 0);
                    rightLaunchState = LaunchState.IDLE;
                    break;
                }

                double rightTargetOffset = getTargetOffset(false); // right shooter offset
                double rightTx           = limelight.getTx();
                double rightError        = rightTx - rightTargetOffset;
                double rightDeriv        = rightError - previousAlignError;

                double rightPower = -(rightError * kP_align + rightDeriv * kD_align);

                if (Math.abs(rightError) < ALIGN_DEADBAND) rightPower = 0;

                rightPower = Math.max(-MAX_ALIGN_POWER, Math.min(MAX_ALIGN_POWER, rightPower));

                mecanumDrive(0, 0, rightPower);
                previousAlignError = rightError;

                telemetry.addData("[R ALIGN] tx",           rightTx);
                telemetry.addData("[R ALIGN] targetOffset", rightTargetOffset);
                telemetry.addData("[R ALIGN] error",        rightError);
                telemetry.addData("[R ALIGN] power",        rightPower);

                if (isAligned(false)) {
                    mecanumDrive(0, 0, 0);
                    previousAlignError = 0;
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
                    rightFeeder.setPower(STOP_SPEED);
                    rightLaunchState = LaunchState.IDLE;
                }
                break;
        }
    }
}
