package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * ============================================================
 *  MOTOR DIRECTION TEST — DECODE Ri3D
 * ============================================================
 *
 *  PURPOSE:
 *    Tells you exactly which motors need to be REVERSED and
 *    whether the intake direction is correct — without needing
 *    to look at the robot while it's running.
 *
 *  HOW TO USE:
 *    1. Put the robot on the ground with WHEELS FREE TO SPIN.
 *    2. Run this OpMode.
 *    3. Follow the on-screen instructions on the Driver Hub.
 *    4. Press each button, watch the wheel/intake, answer Y/N.
 *    5. Read the final VERDICT on the telemetry screen.
 *
 *  CONTROLS:
 *    Each button spins ONE motor at low power for a short burst.
 *    After each test the telemetry tells you what it SHOULD do.
 *    You confirm whether it actually did that.
 *
 *    Gamepad 1:
 *      LEFT  BUMPER  → Test LEFT  FRONT drive motor
 *      RIGHT BUMPER  → Test RIGHT FRONT drive motor
 *      LEFT  TRIGGER → Test LEFT  BACK  drive motor
 *      RIGHT TRIGGER → Test RIGHT BACK  drive motor
 *      A             → Test INTAKE (should pull IN)
 *      B             → Test INTAKE reverse (should push OUT)
 *      Y             → Show FINAL VERDICT / recommended fixes
 *      BACK          → Reset all results
 *
 *  READING RESULTS:
 *    ✅ CORRECT   — motor is fine, no change needed
 *    ❌ REVERSE   — go into your main code and flip this motor's
 *                   direction (FORWARD → REVERSE or vice versa)
 *    ⏳ NOT TESTED — you haven't tested this motor yet
 *
 * ============================================================
 */
@TeleOp(name = "Motor Direction Test", group = "TEST")
public class MotorDirectionTest extends OpMode {

    // ===== Hardware =====
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor intake;

    // ===== Test power — low enough to be safe, high enough to see clearly =====
    private static final double TEST_POWER    = 0.35;
    private static final double TRIGGER_THRESHOLD = 0.5;

    // ===== Test result states =====
    private enum Result { NOT_TESTED, CORRECT, NEEDS_REVERSE }

    private Result leftFrontResult  = Result.NOT_TESTED;
    private Result rightFrontResult = Result.NOT_TESTED;
    private Result leftBackResult   = Result.NOT_TESTED;
    private Result rightBackResult  = Result.NOT_TESTED;
    private Result intakeResult     = Result.NOT_TESTED;

    // ===== Which motor is currently being tested =====
    private enum ActiveTest { NONE, LEFT_FRONT, RIGHT_FRONT, LEFT_BACK, RIGHT_BACK, INTAKE_IN, INTAKE_OUT }
    private ActiveTest activeTest = ActiveTest.NONE;

    // ===== Button edge detection =====
    private boolean lastLB    = false;
    private boolean lastRB    = false;
    private boolean lastA     = false;
    private boolean lastB     = false;
    private boolean lastY     = false;
    private boolean lastBack  = false;
    private boolean lastLT    = false;
    private boolean lastRT    = false;

    // ===== Show verdict screen =====
    private boolean showVerdict = false;

    // ===== Timer to auto-stop motor after short burst =====
    private long testStartTime = 0;
    private static final long TEST_DURATION_MS = 1200; // run each motor for 1.2 seconds

    // =========================================================
    // INIT
    // =========================================================
    @Override
    public void init() {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake          = hardwareMap.get(DcMotor.class, "intake");

        // Set all directions to FORWARD so we're testing raw wiring, not code offsets.
        // This way the test tells you exactly what to put in your main code.
        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stopAllMotors();
        telemetry.addLine("Motor Direction Test READY");
        telemetry.addLine("Press PLAY to start.");
        telemetry.update();
    }

    // =========================================================
    // MAIN LOOP
    // =========================================================
    @Override
    public void loop() {

        boolean lb   = gamepad1.left_bumper;
        boolean rb   = gamepad1.right_bumper;
        boolean lt   = gamepad1.left_trigger  > TRIGGER_THRESHOLD;
        boolean rt   = gamepad1.right_trigger > TRIGGER_THRESHOLD;
        boolean a    = gamepad1.a;
        boolean b    = gamepad1.b;
        boolean y    = gamepad1.y;
        boolean back = gamepad1.back;

        // ===== BUTTON PRESS DETECTION =====

        // LEFT FRONT — left bumper
        if (lb && !lastLB) {
            startTest(ActiveTest.LEFT_FRONT);
        }

        // RIGHT FRONT — right bumper
        if (rb && !lastRB) {
            startTest(ActiveTest.RIGHT_FRONT);
        }

        // LEFT BACK — left trigger
        if (lt && !lastLT) {
            startTest(ActiveTest.LEFT_BACK);
        }

        // RIGHT BACK — right trigger
        if (rt && !lastRT) {
            startTest(ActiveTest.RIGHT_BACK);
        }

        // INTAKE IN — A
        if (a && !lastA) {
            startTest(ActiveTest.INTAKE_IN);
        }

        // INTAKE OUT — B
        if (b && !lastB) {
            startTest(ActiveTest.INTAKE_OUT);
        }

        // SHOW VERDICT — Y
        if (y && !lastY) {
            stopAllMotors();
            activeTest  = ActiveTest.NONE;
            showVerdict = !showVerdict;
        }


        // RESET — BACK
        if (back && !lastBack) {
            resetAll();
        }

        lastLB   = lb;
        lastRB   = rb;
        lastLT   = lt;
        lastRT   = rt;
        lastA    = a;
        lastB    = b;
        lastY    = y;
        lastBack = back;

        // ===== RUN ACTIVE TEST =====
        runActiveTest();

        // ===== DISPLAY =====
        if (showVerdict) {
            displayVerdict();
        } else {
            displayTestScreen();
        }

        telemetry.update();
    }

    // =========================================================
    // START A TEST
    // =========================================================
    private void startTest(ActiveTest test) {
        stopAllMotors();
        activeTest    = test;
        showVerdict   = false;
        testStartTime = System.currentTimeMillis();
    }

    // =========================================================
    // RUN ACTIVE TEST — applies power and auto-stops after duration
    // =========================================================
    private void runActiveTest() {
        long elapsed = System.currentTimeMillis() - testStartTime;

        if (activeTest == ActiveTest.NONE) return;

        if (elapsed > TEST_DURATION_MS) {
            // Auto-stop and record result as pending user confirmation
            stopAllMotors();
            // Don't reset activeTest — keep it so telemetry shows what was just tested
            return;
        }

        // Apply power to the one motor being tested
        switch (activeTest) {
            case LEFT_FRONT:
                leftFrontDrive.setPower(TEST_POWER);
                break;
            case RIGHT_FRONT:
                rightFrontDrive.setPower(TEST_POWER);
                break;
            case LEFT_BACK:
                leftBackDrive.setPower(TEST_POWER);
                break;
            case RIGHT_BACK:
                rightBackDrive.setPower(TEST_POWER);
                break;
            case INTAKE_IN:
                intake.setPower(TEST_POWER);
                break;
            case INTAKE_OUT:
                intake.setPower(-TEST_POWER);
                break;
        }
    }

    // =========================================================
    // DISPLAY — MAIN TEST SCREEN
    // =========================================================
    private void displayTestScreen() {

        long elapsed = System.currentTimeMillis() - testStartTime;
        boolean motorRunning = (activeTest != ActiveTest.NONE) && (elapsed <= TEST_DURATION_MS);

        telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        telemetry.addLine("  MOTOR DIRECTION TEST");
        telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        telemetry.addLine("");

        // ── Active test prompt ──
        if (motorRunning) {
            telemetry.addLine(">>> MOTOR RUNNING — WATCH IT <<<");
            switch (activeTest) {
                case LEFT_FRONT:
                    telemetry.addLine("Testing: LEFT FRONT DRIVE");
                    telemetry.addLine("Expected: wheel spins FORWARD");
                    telemetry.addLine("(robot would move forward-left)");
                    break;
                case RIGHT_FRONT:
                    telemetry.addLine("Testing: RIGHT FRONT DRIVE");
                    telemetry.addLine("Expected: wheel spins FORWARD");
                    telemetry.addLine("(robot would move forward-right)");
                    break;
                case LEFT_BACK:
                    telemetry.addLine("Testing: LEFT BACK DRIVE");
                    telemetry.addLine("Expected: wheel spins FORWARD");
                    telemetry.addLine("(robot would move forward-left)");
                    break;
                case RIGHT_BACK:
                    telemetry.addLine("Testing: RIGHT BACK DRIVE");
                    telemetry.addLine("Expected: wheel spins FORWARD");
                    telemetry.addLine("(robot would move forward-right)");
                    break;
                case INTAKE_IN:
                    telemetry.addLine("Testing: INTAKE (IN direction)");
                    telemetry.addLine("Expected: rollers pull INWARD");
                    telemetry.addLine("(game elements pulled INTO robot)");
                    break;
                case INTAKE_OUT:
                    telemetry.addLine("Testing: INTAKE (OUT direction)");
                    telemetry.addLine("Expected: rollers push OUTWARD");
                    telemetry.addLine("(game elements pushed OUT of robot)");
                    break;
            }
            telemetry.addLine("");
        } else if (activeTest != ActiveTest.NONE) {
            // Motor just stopped — ask for confirmation
            telemetry.addLine(">>> DID IT SPIN CORRECTLY? <<<");
            switch (activeTest) {
                case LEFT_FRONT:
                    telemetry.addLine("LEFT FRONT — did wheel spin FORWARD?");
                    break;
                case RIGHT_FRONT:
                    telemetry.addLine("RIGHT FRONT — did wheel spin FORWARD?");
                    break;
                case LEFT_BACK:
                    telemetry.addLine("LEFT BACK — did wheel spin FORWARD?");
                    break;
                case RIGHT_BACK:
                    telemetry.addLine("RIGHT BACK — did wheel spin FORWARD?");
                    break;
                case INTAKE_IN:
                    telemetry.addLine("INTAKE IN — did it pull inward?");
                    break;
                case INTAKE_OUT:
                    telemetry.addLine("INTAKE OUT — did it push outward?");
                    break;
            }
            telemetry.addLine("");
            telemetry.addLine("  Press A = YES (correct)");
            telemetry.addLine("  Press B = NO  (wrong direction)");
            telemetry.addLine("");

            // Listen for YES/NO on the confirmation screen
            handleConfirmation();
        } else {
            telemetry.addLine("Press a button to test a motor.");
            telemetry.addLine("");
        }

        // ── Controls reminder ──
        telemetry.addLine("━━━ CONTROLS ━━━━━━━━━━━━━━━━━");
        telemetry.addLine("LB  = Test LEFT FRONT  drive");
        telemetry.addLine("RB  = Test RIGHT FRONT drive");
        telemetry.addLine("LT  = Test LEFT BACK   drive");
        telemetry.addLine("RT  = Test RIGHT BACK  drive");
        telemetry.addLine("A   = Test INTAKE in");
        telemetry.addLine("B   = Test INTAKE out");
        telemetry.addLine("Y   = Show VERDICT");
        telemetry.addLine("BACK= Reset all");
        telemetry.addLine("");

        // ── Current results summary ──
        telemetry.addLine("━━━ CURRENT RESULTS ━━━━━━━━━━");
        telemetry.addData("Left Front ", resultIcon(leftFrontResult));
        telemetry.addData("Right Front", resultIcon(rightFrontResult));
        telemetry.addData("Left Back  ", resultIcon(leftBackResult));
        telemetry.addData("Right Back ", resultIcon(rightBackResult));
        telemetry.addData("Intake     ", resultIcon(intakeResult));
    }

    // =========================================================
    // HANDLE YES/NO CONFIRMATION AFTER MOTOR STOPS
    // =========================================================
    // Called every loop while motor is stopped and awaiting answer.
    // Uses raw gamepad reads (not edge detection) because we're in
    // a confirmation state — we just watch for the press.
    // =========================================================
    private boolean awaitingConfirm = false;

    private void handleConfirmation() {
        // We check for button presses here specifically for confirm/deny
        // These use raw reads and we guard with awaitingConfirm flag
        if (!awaitingConfirm) {
            awaitingConfirm = true;
            return;
        }

        if (gamepad1.a) {
            // YES — it spun the right way
            recordResult(activeTest, Result.CORRECT);
            activeTest      = ActiveTest.NONE;
            awaitingConfirm = false;
        } else if (gamepad1.b) {
            // NO — it spun the wrong way
            recordResult(activeTest, Result.NEEDS_REVERSE);
            activeTest      = ActiveTest.NONE;
            awaitingConfirm = false;
        }
    }

    // =========================================================
    // RECORD RESULT
    // =========================================================
    private void recordResult(ActiveTest test, Result result) {
        switch (test) {
            case LEFT_FRONT:  leftFrontResult  = result; break;
            case RIGHT_FRONT: rightFrontResult = result; break;
            case LEFT_BACK:   leftBackResult   = result; break;
            case RIGHT_BACK:  rightBackResult  = result; break;
            case INTAKE_IN:
            case INTAKE_OUT:  intakeResult     = result; break;
        }
    }

    // =========================================================
    // DISPLAY — VERDICT SCREEN
    // =========================================================
    private void displayVerdict() {
        telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        telemetry.addLine("  VERDICT — COPY INTO YOUR CODE");
        telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        telemetry.addLine("");

        // Drivetrain
        telemetry.addLine("-- DRIVETRAIN --");
        telemetry.addData("leftFrontDrive  direction",  recommendDrive(leftFrontResult,  true));
        telemetry.addData("rightFrontDrive direction",  recommendDrive(rightFrontResult, false));
        telemetry.addData("leftBackDrive   direction",  recommendDrive(leftBackResult,   true));
        telemetry.addData("rightBackDrive  direction",  recommendDrive(rightBackResult,  false));
        telemetry.addLine("");

        // Intake
        telemetry.addLine("-- INTAKE --");
        if (intakeResult == Result.NOT_TESTED) {
            telemetry.addLine("intake: NOT TESTED yet");
        } else if (intakeResult == Result.CORRECT) {
            telemetry.addLine("intake: setDirection(FORWARD) — no change needed");
        } else {
            telemetry.addLine("intake: setDirection(REVERSE) — flip it");
        }
        telemetry.addLine("");

        // Warning if anything untested
        boolean anyUntested = leftFrontResult  == Result.NOT_TESTED
                || rightFrontResult == Result.NOT_TESTED
                || leftBackResult   == Result.NOT_TESTED
                || rightBackResult  == Result.NOT_TESTED
                || intakeResult     == Result.NOT_TESTED;

        if (anyUntested) {
            telemetry.addLine("⚠ Some motors not tested yet.");
            telemetry.addLine("  Press Y again to go back and test them.");
        } else {
            telemetry.addLine("✅ All motors tested!");
            telemetry.addLine("  Copy the directions above into");
            telemetry.addLine("  your main TeleOp init() block.");
        }

        telemetry.addLine("");
        telemetry.addLine("Press Y to go back | BACK to reset");
    }

    // =========================================================
    // HELPERS
    // =========================================================

    /**
     * Returns the recommended direction string for a drive motor.
     * Left-side motors default to FORWARD, right-side to REVERSE
     * for a standard mecanum setup — flip if the test said wrong.
     */
    private String recommendDrive(Result result, boolean isLeftSide) {
        if (result == Result.NOT_TESTED) return "⏳ NOT TESTED";

        // Standard mecanum: left = FORWARD, right = REVERSE
        // If the test says CORRECT, use the standard direction.
        // If NEEDS_REVERSE, flip it.
        boolean useForward;
        if (isLeftSide) {
            useForward = (result == Result.CORRECT);
        } else {
            useForward = (result == Result.NEEDS_REVERSE); // right side is inverted standard
        }

        return useForward ? "✅ FORWARD" : "❌ REVERSE";
    }

    private String resultIcon(Result result) {
        switch (result) {
            case CORRECT:       return "✅ Correct";
            case NEEDS_REVERSE: return "❌ Needs REVERSE";
            default:            return "⏳ Not tested";
        }
    }

    private void stopAllMotors() {
        if (leftFrontDrive  != null) leftFrontDrive.setPower(0);
        if (rightFrontDrive != null) rightFrontDrive.setPower(0);
        if (leftBackDrive   != null) leftBackDrive.setPower(0);
        if (rightBackDrive  != null) rightBackDrive.setPower(0);
        if (intake          != null) intake.setPower(0);
    }

    private void resetAll() {
        stopAllMotors();
        activeTest      = ActiveTest.NONE;
        showVerdict     = false;
        awaitingConfirm = false;
        leftFrontResult  = Result.NOT_TESTED;
        rightFrontResult = Result.NOT_TESTED;
        leftBackResult   = Result.NOT_TESTED;
        rightBackResult  = Result.NOT_TESTED;
        intakeResult     = Result.NOT_TESTED;
    }

    @Override
    public void stop() {
        stopAllMotors();
    }
}