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
    private double kP_align = .08;
    private double kD_align = 0.003;
    private double previousAlignError = 0;

    private final double ALIGN_DEADBAND  = 0.2;
    private final double MAX_ALIGN_POWER = 0.7;
    // ==================================================

    // ================= DYNAMIC OFFSET TUNING =================
    private final double BASE_OFFSET     = -3.0;
    private final double DISTANCE_FACTOR =  0.002;
    private final double SHOOTER_OFFSET  =  .1;
    // =========================================================

    final double FEED_TIME_SECONDS = 1.5;
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

    private boolean lastBack   = false;
    private boolean lastGP2RB  = false;   // edge detect for GP2 right bumper (manual left shot)
    private boolean lastGP2LB  = false;   // edge detect for GP2 left  bumper (manual right shot)

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

    ElapsedTime leftFeederTimer        = new ElapsedTime();
    ElapsedTime rightFeederTimer       = new ElapsedTime();

    // ===== State Enums =====

    private enum LaunchState {
        IDLE,
        ALIGN,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    private LaunchState leftLaunchState;
    private LaunchState rightLaunchState;

    // =========================================================
    // MANUAL (NO-ALIGN) STATE MACHINE
    // =========================================================
    // Mirrors LaunchState but skips ALIGN entirely.
    // Used for GP2 bumper fallback shots — great for lookup table
    // data collection and shooter verification without vision.
    //
    //   GP2 RIGHT BUMPER → manual LEFT  shot (spin up left launcher,  fire left  feeder)
    //   GP2 LEFT  BUMPER → manual RIGHT shot (spin up right launcher, fire right feeder)
    //
    // The velocity target is read from Limelight distance → ShooterModel
    // at the moment the button is pressed (same formula as VISION mode),
    // so you get real distance-based data even without alignment.
    // If Limelight has no target, it falls back to launcherTarget.
    // =========================================================
    private enum ManualLaunchState { IDLE, SPIN_UP, LAUNCH, LAUNCHING }
    private ManualLaunchState manualLeftState  = ManualLaunchState.IDLE;
    private ManualLaunchState manualRightState = ManualLaunchState.IDLE;

    // Velocity snapshot taken when the manual shot is triggered.
    // Stored separately so a Limelight dropout mid-shot doesn't change the speed.
    private double manualLeftTarget  = 0;
    private double manualRightTarget = 0;

    private enum DiverterDirection { LEFT, RIGHT }
    private DiverterDirection diverterDirection = DiverterDirection.LEFT;

    private enum IntakeState { ON, OFF }
    private IntakeState intakeState = IntakeState.OFF;

    private enum ShooterMode { MANUAL, VISION }
    private ShooterMode shooterMode = ShooterMode.VISION;

    private enum LauncherDistance { CLOSE, FAR }
    private LauncherDistance launcherDistance = LauncherDistance.CLOSE;

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

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);

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

        rightFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
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
            double rpm            = ShooterModel.distanceToRPM(distanceCm);
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
        boolean autoAlignActive = gamepad1.right_trigger > 0.5;

        if (autoAlignActive && limelight.hasTarget()) {
            double currentYaw = limelight.getTx();
            double error      = -currentYaw;
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

        // ===== Vision launch state machines (GP1 bumpers) =====
        // Right bumper = left shooter, Left bumper = right shooter
        launchLeft(gamepad1.rightBumperWasPressed());
        launchRight(gamepad1.leftBumperWasPressed());

        // ===== Manual (no-align) launch state machines (GP2 bumpers) =====
        // GP2 Right bumper = manual left shot, GP2 Left bumper = manual right shot
        boolean gp2rb = gamepad2.right_bumper;
        boolean gp2lb = gamepad2.left_bumper;
        manualLaunchLeft( gp2rb && !lastGP2RB);
        manualLaunchRight(gp2lb && !lastGP2LB);
        lastGP2RB = gp2rb;
        lastGP2LB = gp2lb;

        // ===== Telemetry =====
        final double leftRPM  = leftLauncher.getVelocity()  * 60 / 28;
        final double rightRPM = rightLauncher.getVelocity() * 60 / 28;

        telemetry.addData("Shooter Mode",        shooterMode);
        telemetry.addData("Launcher Distance",   launcherDistance);
        telemetry.addData("Auto Align Active",   autoAlignActive);
        telemetry.addData("Left Launch State",   leftLaunchState);
        telemetry.addData("Right Launch State",  rightLaunchState);
        telemetry.addData("Manual Left State",   manualLeftState);
        telemetry.addData("Manual Right State",  manualRightState);
        telemetry.addData("Left RPM",            leftRPM);
        telemetry.addData("Right RPM",           rightRPM);
        telemetry.addData("Vision Target TPS",   launcherTarget);

        if (limelight.hasTarget()) {
            telemetry.addData("Limelight tx",      limelight.getTx());
            telemetry.addData("Distance (cm)",     limelight.getDistanceFromArea());
            telemetry.addData("L targetOffset",    getTargetOffset(true));
            telemetry.addData("R targetOffset",    getTargetOffset(false));
        }

        // Manual shot data — always show so you can log it during testing
        telemetry.addData("Manual L target TPS", manualLeftTarget);
        telemetry.addData("Manual R target TPS", manualRightTarget);
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
    public boolean isAligned(boolean isLeftShooter) {
        if (limelight == null || !limelight.hasTarget()) return false;
        double targetOffset = getTargetOffset(isLeftShooter);
        return Math.abs(limelight.getTx() - targetOffset) < ALIGN_DEADBAND;
    }

    // =========================================================
    // LEFT LAUNCH STATE MACHINE (vision, with alignment)
    // State flow: IDLE → ALIGN → SPIN_UP → LAUNCH → LAUNCHING → IDLE
    // =========================================================
    void launchLeft(boolean shotRequested) {
        switch (leftLaunchState) {

            case IDLE:
                if (shotRequested && limelight.hasTarget()) {
                    previousAlignError = 0;
                    leftLaunchState = LaunchState.ALIGN;
                }
                break;

            case ALIGN:
                if (!limelight.hasTarget()) {
                    mecanumDrive(0, 0, 0);
                    leftLaunchState = LaunchState.IDLE;
                    break;
                }

                double leftTargetOffset = getTargetOffset(true);
                double leftTx           = limelight.getTx();
                double leftError        = leftTx - leftTargetOffset;
                double leftDeriv        = leftError - previousAlignError;
                double leftPower        = -(leftError * kP_align + leftDeriv * kD_align);

                if (Math.abs(leftError) < ALIGN_DEADBAND) leftPower = 0;
                leftPower = Math.max(-MAX_ALIGN_POWER, Math.min(MAX_ALIGN_POWER, leftPower));

                mecanumDrive(0, 0, leftPower);
                previousAlignError = leftError;

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
    // RIGHT LAUNCH STATE MACHINE (vision, with alignment)
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

                double rightTargetOffset = getTargetOffset(false);
                double rightTx           = limelight.getTx();
                double rightError        = rightTx - rightTargetOffset;
                double rightDeriv        = rightError - previousAlignError;
                double rightPower        = -(rightError * kP_align + rightDeriv * kD_align);

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

    // =========================================================
    // MANUAL LEFT LAUNCH STATE MACHINE (no alignment)
    // =========================================================
    // GP2 RIGHT BUMPER triggers this.
    // Spins up BOTH launchers to the distance-based velocity,
    // then fires the LEFT feeder for FEED_TIME_SECONDS.
    // No robot rotation — driver aims manually.
    // Use this to build your RPM lookup table:
    //   stand at a known distance, press GP2 RB, read "Manual L target TPS"
    //   from telemetry, convert → RPM = TPS * 60 / 28.
    // =========================================================
    void manualLaunchLeft(boolean shotRequested) {
        switch (manualLeftState) {

            case IDLE:
                if (shotRequested) {
                    // Snapshot velocity at trigger time — uses Limelight distance
                    // if available, otherwise falls back to current launcherTarget.
                    if (limelight.hasTarget()) {
                        double distanceCm     = limelight.getDistanceFromArea();
                        double rpm            = ShooterModel.distanceToRPM(distanceCm);
                        manualLeftTarget      = rpm * 28.0 / 60.0;
                    } else {
                        manualLeftTarget = launcherTarget;
                    }
                    manualLeftState = ManualLaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                leftLauncher.setVelocity(manualLeftTarget);
                rightLauncher.setVelocity(manualLeftTarget);

                // Use the same 95% threshold as the vision path
                if (leftLauncher.getVelocity() > manualLeftTarget * 0.95) {
                    manualLeftState = ManualLaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                leftFeederTimer.reset();
                manualLeftState = ManualLaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (leftFeederTimer.seconds() > FEED_TIME_SECONDS) {
                    leftFeeder.setPower(STOP_SPEED);
                    // Let flywheels coast down naturally
                    leftLauncher.setVelocity(0);
                    rightLauncher.setVelocity(0);
                    manualLeftState = ManualLaunchState.IDLE;
                }
                break;
        }
    }

    // =========================================================
    // MANUAL RIGHT LAUNCH STATE MACHINE (no alignment)
    // =========================================================
    // GP2 LEFT BUMPER triggers this.
    // Spins up BOTH launchers to the distance-based velocity,
    // then fires the RIGHT feeder for FEED_TIME_SECONDS.
    // =========================================================
    void manualLaunchRight(boolean shotRequested) {
        switch (manualRightState) {

            case IDLE:
                if (shotRequested) {
                    if (limelight.hasTarget()) {
                        double distanceCm     = limelight.getDistanceFromArea();
                        double rpm            = ShooterModel.distanceToRPM(distanceCm);
                        manualRightTarget     = rpm * 28.0 / 60.0;
                    } else {
                        manualRightTarget = launcherTarget;
                    }
                    manualRightState = ManualLaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                leftLauncher.setVelocity(manualRightTarget);
                rightLauncher.setVelocity(manualRightTarget);

                if (rightLauncher.getVelocity() > manualRightTarget * 0.95) {
                    manualRightState = ManualLaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                rightFeeder.setPower(FULL_SPEED);
                rightFeederTimer.reset();
                manualRightState = ManualLaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (rightFeederTimer.seconds() > FEED_TIME_SECONDS) {
                    rightFeeder.setPower(STOP_SPEED);
                    leftLauncher.setVelocity(0);
                    rightLauncher.setVelocity(0);
                    manualRightState = ManualLaunchState.IDLE;
                }
                break;
        }
    }
}