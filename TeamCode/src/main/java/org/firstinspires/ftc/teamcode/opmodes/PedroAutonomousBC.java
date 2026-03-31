package org.firstinspires.ftc.teamcode.opmodes; // make sure this aligns with class location

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.ShooterModel;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.vision.LimelightVision;

@Autonomous(name = "Score Auto - Blue Close", group = "Examples")
public class PedroAutonomousBC extends OpMode {

    // =========================================================================
    // PedroPathing
    // =========================================================================
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(28.5, 128, Math.toRadians(180));
    private final Pose scorePose = new Pose(60,   85,  Math.toRadians(135));
    private final Pose intake1pose = new Pose(16.82285714285714,   83.5,  Math.toRadians(135));

    private Path toScore;

    // =========================================================================
    // Hardware
    // =========================================================================
    private DcMotorEx leftLauncher  = null;
    private DcMotorEx rightLauncher = null;
    private CRServo   leftFeeder    = null;
    private CRServo   rightFeeder   = null;

    private LimelightVision limelight;

    // =========================================================================
    // Constants
    // =========================================================================
    private static final double FEED_TIME_SECONDS = 0.8;
    private static final double STOP_SPEED        = 0.0;
    private static final double FULL_SPEED        = 1.0;
    private static final int    TOTAL_SHOTS       = 3;

    // =========================================================================
    // Launcher State Machine
    // =========================================================================
    private enum LaunchState { IDLE, SPIN_UP, LAUNCH, LAUNCHING }
    private LaunchState launchState = LaunchState.IDLE;

    private double      launcherTarget = 0;
    private double      launcherMin    = 0;
    private int         shotsFired     = 0;

    private ElapsedTime feederTimer = new ElapsedTime();

    // =========================================================================
    // buildPaths()
    // =========================================================================
    private void buildPaths() {
        toScore = new Path(new BezierLine(startPose, scorePose));
        toScore.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading(), intake1pose.getHeading());
    }

    // =========================================================================
    // Launcher helpers
    // =========================================================================

    /** Call once to kick off a single shot cycle. */
    private void requestShot() {
        if (launchState == LaunchState.IDLE) {
            launchState = LaunchState.SPIN_UP;
        }
    }

    /** Returns true when the launcher FSM is back to IDLE (shot complete). */
    private boolean launcherIdle() {
        return launchState == LaunchState.IDLE;
    }

    private void stopLaunchers() {
        leftLauncher.setVelocity(0);
        rightLauncher.setVelocity(0);
    }

    /**
     * Must be called every loop tick.
     * Handles spin-up → feed → stop for a single shot, then returns to IDLE.
     * Fires BOTH feeders simultaneously (left and right launchers shoot together).
     */
    private void updateLauncher() {
        switch (launchState) {
            case IDLE:
                break;

            case SPIN_UP:
                leftLauncher.setVelocity(launcherTarget);
                rightLauncher.setVelocity(launcherTarget);
                // Advance once BOTH launchers are up to speed
                if (leftLauncher.getVelocity()  > launcherMin
                        && rightLauncher.getVelocity() > launcherMin) {
                    launchState = LaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                    shotsFired++;
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }

    // =========================================================================
    // Path FSM
    //
    //  State map:
    //   0  → Drive to scoring position
    //   1  → Wait to arrive → update limelight, set RPM from distance
    //   2  → Fire shot; wait for it to finish; repeat until 3 shots done
    //   3  → Stop launchers — DONE
    // =========================================================================
    private void autonomousPathUpdate() {
        updateLauncher(); // runs every tick

        switch (pathState) {

            case 0:
                // Begin driving to the scoring position
                follower.followPath(toScore);
                setPathState(1);
                break;

            case 1:
                // Wait to arrive, keep polling limelight so RPM is ready
                limelight.update();
                if (!follower.isBusy()) {
                    // Grab distance from AprilTag and convert to launcher velocity
                    if (limelight.hasTarget()) {
                        double distanceCm     = limelight.getDistanceFromArea();
                        double rpm            = ShooterModel.distanceToRPM(distanceCm,true);
                        launcherTarget        = rpm * 28.0 / 60.0;
                        launcherMin           = launcherTarget * 0.95;
                    }
                    // Advance to shooting even if no tag yet — target will use last known value
                    setPathState(2);
                }
                break;

            case 2:
                // Keep limelight fresh in case we want to update between shots
                limelight.update();
                if (limelight.hasTarget()) {
                    double distanceCm  = limelight.getDistanceFromArea();
                    double rpm         = ShooterModel.distanceToRPM(distanceCm,true);
                    launcherTarget     = rpm * 28.0 / 60.0;
                    launcherMin        = launcherTarget * 0.95;
                }

                if (shotsFired < TOTAL_SHOTS) {
                    // Queue next shot whenever the launcher is idle
                    requestShot();
                } else {
                    // All 3 shots done
                    setPathState(3);
                }
                break;

            case 3:
                stopLaunchers();
                setPathState(-1); // DONE
                break;

            default:
                break; // -1: finished, do nothing
        }
    }

    private void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    // =========================================================================
    // OpMode Lifecycle
    // =========================================================================

    @Override
    public void init() {
        pathTimer   = new Timer();
        opmodeTimer = new Timer();

        follower      = Constants.createFollower(hardwareMap);
        leftLauncher  = hardwareMap.get(DcMotorEx.class, "left_launcher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "right_launcher");
        leftFeeder    = hardwareMap.get(CRServo.class,   "left_feeder");
        rightFeeder   = hardwareMap.get(CRServo.class,   "right_feeder");
        limelight     = new LimelightVision(hardwareMap);

        leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLauncher.setZeroPowerBehavior(BRAKE);
        rightLauncher.setZeroPowerBehavior(BRAKE);
        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10));
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10));

        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        final double leftRPM  = leftLauncher.getVelocity()  * 60.0 / 28.0;
        final double rightRPM = rightLauncher.getVelocity() * 60.0 / 28.0;

        telemetry.addData("Path State",       pathState);
        telemetry.addData("Launch State",     launchState);
        telemetry.addData("Shots Fired",      shotsFired + " / " + TOTAL_SHOTS);
        telemetry.addData("Target RPM",       launcherTarget * 60.0 / 28.0);
        telemetry.addData("Left RPM",         leftRPM);
        telemetry.addData("Right RPM",        rightRPM);
        telemetry.addData("Has AprilTag",     limelight.hasTarget());
        if (limelight.hasTarget()) {
            telemetry.addData("Distance (cm)", limelight.getDistanceFromArea());
        }
        telemetry.addData("Opmode Time (s)",  opmodeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    @Override
    public void stop() {
        stopLaunchers();
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
    }
}