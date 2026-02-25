package org.firstinspires.ftc.teamcode.opmodes;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.ShooterModel;
import org.firstinspires.ftc.teamcode.vision.LimelightVision;

@Autonomous(name = "DECODE Competition Auto BLUE", group = "Competition")
public class DecodeSimpleAutoBLUE extends OpMode {

    DcMotor leftFront, rightFront, leftBack, rightBack, intake;
    DcMotorEx leftLauncher, rightLauncher;
    CRServo leftFeeder, rightFeeder;

    LimelightVision limelight;

    ElapsedTime timer = new ElapsedTime();

    double launcherTarget;
    double launcherMin;

    int obeliskPattern = -1;
    int shotIndex = 0;
    String shotSequence = "";

    final double FEED_TIME = 1.5;
    final double SHOT_DELAY = 3.0;

    final double ALIGN_kP = 0.02;
    final double ALIGN_DEADBAND = 1.0;

    enum State {
        DRIVE_BACK,
        TURN_LEFT,
        READ_PATTERN,
        TURN_RIGHT,
        SPIN_UP,
        SHOOT,
        WAIT_BETWEEN,
        DONE
    }

    State state = State.DRIVE_BACK;

    @Override
    public void init() {

        leftFront = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFront = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBack = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBack = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftLauncher = hardwareMap.get(DcMotorEx.class, "left_launcher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "right_launcher");

        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        limelight = new LimelightVision(hardwareMap);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLauncher.setZeroPowerBehavior(BRAKE);
        rightLauncher.setZeroPowerBehavior(BRAKE);

        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(BRAKE);

    }

    @Override
    public void start() {
        timer.reset();
        intake.setPower(1.0);  // RUN WHOLE AUTO
    }

    @Override
    public void loop() {

        limelight.update();

        switch (state) {

            case DRIVE_BACK:
                mecanumDrive(0, 0, 0.5);
                if (timer.seconds() > 1.8) { // tune for 130cm
                    mecanumDrive(0,0,0);
                    state = State.TURN_LEFT;
                    timer.reset();
                }
                break;

            case TURN_LEFT:
                mecanumDrive(0.5, 0, 0);
                if (timer.seconds() > 0.8) {
                    mecanumDrive(0,0,0);
                    state = State.READ_PATTERN;
                    timer.reset();
                }
                break;

            case READ_PATTERN:
                obeliskPattern = limelight.getObeliskNumber();

                if (obeliskPattern == 21) shotSequence = "RLL"; // GPP
                if (obeliskPattern == 22) shotSequence = "LRL"; // PGP
                if (obeliskPattern == 23) shotSequence = "LLR"; // PPG

                state = State.TURN_RIGHT;
                timer.reset();
                break;

            case TURN_RIGHT:
                mecanumDrive(-0.5, 0, 0);
                if (timer.seconds() > 0.75) {
                    mecanumDrive(0,0,0);
                    state = State.SPIN_UP;
                }
                break;

            case SPIN_UP:

                if (limelight.hasTarget()) {

                    double distance = limelight.getDistanceFromArea();
                    double rpm = ShooterModel.distanceToRPM(distance) * 1; // boost to prevent undershoot

                    double tps = rpm * 28.0 / 60.0;

                    launcherTarget = tps;
                    launcherMin = tps * 0.97;

                    leftLauncher.setVelocity(launcherTarget);
                    rightLauncher.setVelocity(launcherTarget);

                    if (leftLauncher.getVelocity() > launcherMin) {
                        state = State.SHOOT;
                    }
                }
                break;

            case SHOOT:

                if (shotIndex < 3) {

                    char shot = shotSequence.charAt(shotIndex);

                    if (shot == 'L') {fireLeft();}
                    if (shot == 'R') {fireRight();}

                    shotIndex++;
                    timer.reset();
                    state = State.WAIT_BETWEEN;
                } else {
                    state = State.DONE;
                }
                break;

            case WAIT_BETWEEN:
                if (timer.seconds() > SHOT_DELAY) {
                    state = State.SPIN_UP; // re-evaluate distance before next shot
                }
                break;

            case DONE:
                mecanumDrive(0,0,0);
                leftLauncher.setVelocity(0);
                rightLauncher.setVelocity(0);
                intake.setPower(0);
                mecanumDrive(0, 0, 0.5);
                if (timer.seconds() > .5) { // tune for 130cm
                    mecanumDrive(0,0,0);
                    timer.reset();
                }
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Pattern", obeliskPattern);
        telemetry.addData("Shot Sequence", shotSequence);
        if (limelight.hasTarget())
            telemetry.addData("Distance", limelight.getDistanceFromArea());
    }

    void mecanumDrive(double forward, double strafe, double rotate) {

        double denominator = Math.max(Math.abs(forward)
                + Math.abs(strafe)
                + Math.abs(rotate), 1);

        leftFront.setPower((forward + strafe + rotate)/denominator);
        rightFront.setPower((forward - strafe - rotate)/denominator);
        leftBack.setPower((forward - strafe + rotate)/denominator);
        rightBack.setPower((forward + strafe - rotate)/denominator);
    }

    void fireLeft() {
        leftFeeder.setPower(1.0);
        timer.reset();
        while (timer.seconds() < FEED_TIME) {}
        leftFeeder.setPower(0);
    }

    void fireRight() {
        rightFeeder.setPower(1.0);
        timer.reset();
        while (timer.seconds() < FEED_TIME) {}
        rightFeeder.setPower(0);
    }
}