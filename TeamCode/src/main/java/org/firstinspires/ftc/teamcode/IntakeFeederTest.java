package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Intake + Feeder Test", group = "Test")
public class IntakeFeederTest extends OpMode {

    // =========================
    // Constants
    // =========================
    private static final double INTAKE_POWER = .8;
    private static final double FEED_POWER = 1.0;
    private static final double STOP_POWER = 0.0;
    private static final double FEED_TIME = 0.75; // seconds

    // =========================
    // Hardware
    // =========================
    private DcMotor intakeMotor;
    private CRServo leftFeederServo;
    private CRServo rightFeederServo;

    private ElapsedTime leftTimer = new ElapsedTime();
    private ElapsedTime rightTimer = new ElapsedTime();

    // =========================
    // Intake State
    // =========================
    private enum IntakeState {
        OFF,
        ON
    }

    private IntakeState intakeState = IntakeState.OFF;

    // =========================
    // Feeder State
    // =========================
    private boolean leftFeeding = false;
    private boolean rightFeeding = false;

    @Override
    public void init() {

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        leftFeederServo = hardwareMap.get(CRServo.class, "leftFeederServo");
        rightFeederServo = hardwareMap.get(CRServo.class, "rightFeederServo");

        intakeMotor.setPower(0);

        leftFeederServo.setPower(STOP_POWER);
        rightFeederServo.setPower(STOP_POWER);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {

        // =========================
        // INTAKE TOGGLE (A button)
        // =========================
        if (gamepad1.a) {
            if (intakeState == IntakeState.OFF) {
                intakeState = IntakeState.ON;
                intakeMotor.setPower(-INTAKE_POWER);
            } else {
                intakeState = IntakeState.OFF;
                intakeMotor.setPower(0);
            }

        }

        // =========================
        // INTAKE REVERSE (Hold B)
        // =========================
        if (gamepad1.b) {
            intakeMotor.setPower(INTAKE_POWER);
        } else if (intakeState == IntakeState.ON) {
            intakeMotor.setPower(-INTAKE_POWER);
        }

        // =========================
        // LEFT FEEDER
        // =========================
        if (gamepad1.left_bumper && !leftFeeding) {
            leftFeeding = true;
            leftTimer.reset();
            leftFeederServo.setPower(FEED_POWER);
        }

        if (leftFeeding && leftTimer.seconds() > FEED_TIME) {
            leftFeederServo.setPower(STOP_POWER);
            leftFeeding = false;
        }

        // =========================
        // RIGHT FEEDER
        // =========================
        if (gamepad1.right_bumper && !rightFeeding) {
            rightFeeding = true;
            rightTimer.reset();
            rightFeederServo.setPower(FEED_POWER);
        }

        if (rightFeeding && rightTimer.seconds() > FEED_TIME) {
            rightFeederServo.setPower(STOP_POWER);
            rightFeeding = false;
        }

        // =========================
        // Telemetry
        // =========================
        telemetry.addData("Intake State", intakeState);
        telemetry.addData("Left Feeding", leftFeeding);
        telemetry.addData("Right Feeding", rightFeeding);
    }
}
