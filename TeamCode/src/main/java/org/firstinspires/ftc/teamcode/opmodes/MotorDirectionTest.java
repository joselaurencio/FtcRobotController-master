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
 *    whether the intake and shooter directions are correct —
 *    without needing to look at the robot while it's running.
 *
 *  HOW TO USE:
 *    1. Put the robot on the ground with WHEELS FREE TO SPIN.
 *    2. Run this OpMode.
 *    3. Follow the on-screen instructions on the Driver Hub.
 *    4. Press each button, watch the wheel/intake/shooter, answer Y/N.
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
 *
 *    Gamepad 2:
 *      LEFT  BUMPER  → Test LEFT  LAUNCHER (should spin outward/upward)
 *      RIGHT BUMPER  → Test RIGHT LAUNCHER (should spin outward/upward)
 *
 *    Gamepad 1:
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
    private DcMotor leftLauncher;
    private DcMotor rightLauncher;

    // ===== Test power — low enough to be safe, high enough to see clearly =====
    private static final double TEST_POWER        = 0.35;
    private static final double TRIGGER_THRESHOLD = 0.5;

    // ===== Test result states =====
    private enum Result { NOT_TESTED, CORRECT, NEEDS_REVERSE }

    private Result leftFrontResult   = Result.NOT_TESTED;
    private Result rightFrontResult  = Result.NOT_TESTED;
    private Result leftBackResult    = Result.NOT_TESTED;
    private Result rightBackResult   = Result.NOT_TESTED;
    private Result intakeResult      = Result.NOT_TESTED;
    private Result leftLauncherResult  = Result.NOT_TESTED;
    private Result rightLauncherResult = Result.NOT_TESTED;

    // ===== Which motor is currently being tested =====
    private enum ActiveTest {
        NONE,
        LEFT_FRONT, RIGHT_FRONT, LEFT_BACK, RIGHT_BACK,
        INTAKE_IN, INTAKE_OUT,
        LEFT_LAUNCHER, RIGHT_LAUNCHER
    }
    private ActiveTest activeTest = ActiveTest.NONE;

    // ===== Button edge detection — Gamepad 1 =====
    private boolean lastLB    = false;
    private boolean lastRB    = false;
    private boolean lastA     = false;
    private boolean lastB     = false;
    private boolean lastY     = false;
    private boolean lastBack  = false;
    private boolean lastLT    = false;
    private boolean lastRT    = false;

    // ===== Button edge detection — Gamepad 2 =====
    private boolean lastGP2LB = false;
    private boolean lastGP2RB = false;

    // ===== Show verdict screen =====
    private boolean showVerdict = false;

    // ===== Timer to auto-stop motor after short burst =====
    private long testStartTime = 0;
    private static final long TEST_DURATION_MS = 1200; // run each motor for 1.2 seconds

    // ===== Confirmation guard =====
    private boolean awaitingConfirm = false;

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
        leftLauncher    = hardwareMap.get(DcMotor.class, "left_launcher");
        rightLauncher   = hardwareMap.get(DcMotor.class, "right_launcher");

        // Set all directions to FORWARD so we're testing raw wiring, not code offsets.
        // This way the test tells you exactly what to put in your main code.
        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLauncher.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLauncher.setDirection(DcMotorSimple.Direction.FORWARD);

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

        // ── Gamepad 1 reads ──
        boolean lb   = gamepad1.left_bumper;
        boolean rb   = gamepad1.right_bumper;
        boolean lt   = gamepad1.left_trigger  > TRIGGER_THRESHOLD;
        boolean rt   = gamepad1.right_trigger > TRIGGER_THRESHOLD;
        boolean a    = gamepad1.a;
        boolean b    = gamepad1.b;
        boolean y    = gamepad1.y;
        boolean back = gamepad1.back;

        // ── Gamepad 2 reads ──
        boolean gp2lb = gamepad2.left_bumper;
        boolean gp2rb = gamepad2.right_bumper;

        // ===== BUTTON PRESS DETECTION — DRIVE =====

        if (lb && !lastLB)  startTest(ActiveTest.LEFT_FRONT);
        if (rb && !lastRB)  startTest(ActiveTest.RIGHT_FRONT);
        if (lt && !lastLT)  startTest(ActiveTest.LEFT_BACK);
        if (rt && !lastRT)  startTest(ActiveTest.RIGHT_BACK);

        // ===== INTAKE =====
        if (a && !lastA)    startTest(ActiveTest.INTAKE_IN);
        if (b && !lastB)    startTest(ActiveTest.INTAKE_OUT);

        // ===== LAUNCHERS — Gamepad 2 =====
        if (gp2lb && !lastGP2LB) startTest(ActiveTest.LEFT_LAUNCHER);
        if (gp2rb && !lastGP2RB) startTest(ActiveTest.RIGHT_LAUNCHER);

        // ===== VERDICT / RESET =====
        if (y && !lastY) {
            stopAllMotors();
            activeTest  = ActiveTest.NONE;
            showVerdict = !showVerdict;
        }
        if (back && !lastBack) {
            resetAll();
        }

        // ── Save previous states ──
        lastLB    = lb;
        lastRB    = rb;
        lastLT    = lt;
        lastRT    = rt;
        lastA     = a;
        lastB     = b;
        lastY     = y;
        lastBack  = back;
        lastGP2LB = gp2lb;
        lastGP2RB = gp2rb;

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
        activeTest      = test;
        showVerdict     = false;
        awaitingConfirm = false;
        testStartTime   = System.currentTimeMillis();
    }

    // =========================================================
    // RUN ACTIVE TEST — applies power and auto-stops after duration
    // =========================================================
    private void runActiveTest() {
        if (activeTest == ActiveTest.NONE) return;

        long elapsed = System.currentTimeMillis() - testStartTime;

        if (elapsed > TEST_DURATION_MS) {
            stopAllMotors();
            return; // keep activeTest set so telemetry can ask for confirmation
        }

        switch (activeTest) {
            case LEFT_FRONT:    leftFrontDrive.setPower(TEST_POWER);   break;
            case RIGHT_FRONT:   rightFrontDrive.setPower(TEST_POWER);  break;
            case LEFT_BACK:     leftBackDrive.setPower(TEST_POWER);    break;
            case RIGHT_BACK:    rightBackDrive.setPower(TEST_POWER);   break;
            case INTAKE_IN:     intake.setPower(TEST_POWER);           break;
            case INTAKE_OUT:    intake.setPower(-TEST_POWER);          break;
            case LEFT_LAUNCHER: leftLauncher.setPower(TEST_POWER);     break;
            case RIGHT_LAUNCHER:rightLauncher.setPower(TEST_POWER);    break;
        }
    }

    // =========================================================
    // DISPLAY — MAIN TEST SCREEN
    // =========================================================
    private void displayTestScreen() {

        long elapsed     = System.currentTimeMillis() - testStartTime;
        boolean motorRunning = (activeTest != ActiveTest.NONE) && (elapsed <= TEST_DURATION_MS);

        telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        telemetry.addLine("  MOTOR DIRECTION TEST");
        telemetry.addLine("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        telemetry.addLine("");

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
                case LEFT_LAUNCHER:
                    telemetry.addLine("Testing: LEFT LAUNCHER");
                    telemetry.addLine("Expected: flywheel spins OUTWARD");
                    telemetry.addLine("(top of wheel moves away from center)");
                    break;
                case RIGHT_LAUNCHER:
                    telemetry.addLine("Testing: RIGHT LAUNCHER");
                    telemetry.addLine("Expected: flywheel spins OUTWARD");
                    telemetry.addLine("(top of wheel moves away from center)");
                    break;
            }
            telemetry.addLine("");

        } else if (activeTest != ActiveTest.NONE) {
            telemetry.addLine(">>> DID IT SPIN CORRECTLY? <<<");
            switch (activeTest) {
                case LEFT_FRONT:     telemetry.addLine("LEFT FRONT — did wheel spin FORWARD?");          break;
                case RIGHT_FRONT:    telemetry.addLine("RIGHT FRONT — did wheel spin FORWARD?");         break;
                case LEFT_BACK:      telemetry.addLine("LEFT BACK — did wheel spin FORWARD?");           break;
                case RIGHT_BACK:     telemetry.addLine("RIGHT BACK — did wheel spin FORWARD?");          break;
                case INTAKE_IN:      telemetry.addLine("INTAKE IN — did it pull inward?");               break;
                case INTAKE_OUT:     telemetry.addLine("INTAKE OUT — did it push outward?");             break;
                case LEFT_LAUNCHER:  telemetry.addLine("LEFT LAUNCHER — did it spin outward?");          break;
                case RIGHT_LAUNCHER: telemetry.addLine("RIGHT LAUNCHER — did it spin outward?");         break;
            }
            telemetry.addLine("");
            telemetry.addLine("  GP1 A = YES (correct)");
            telemetry.addLine("  GP1 B = NO  (wrong direction)");
            telemetry.addLine("");
            handleConfirmation();

        } else {
            telemetry.addLine("Press a button to test a motor.");
            telemetry.addLine("");
        }

        // ── Controls reminder ──
        telemetry.addLine("━━━ CONTROLS ━━━━━━━━━━━━━━━━━");
        telemetry.addLine("GP1 LB  = LEFT FRONT  drive");
        telemetry.addLine("GP1 RB  = RIGHT FRONT drive");
        telemetry.addLine("GP1 LT  = LEFT BACK   drive");
        telemetry.addLine("GP1 RT  = RIGHT BACK  drive");
        telemetry.addLine("GP1 A   = INTAKE in");
        telemetry.addLine("GP1 B   = INTAKE out");
        telemetry.addLine("GP2 LB  = LEFT  LAUNCHER");
        telemetry.addLine("GP2 RB  = RIGHT LAUNCHER");
        telemetry.addLine("GP1 Y   = Show VERDICT");
        telemetry.addLine("GP1 BACK= Reset all");
        telemetry.addLine("");

        // ── Current results summary ──
        telemetry.addLine("━━━ CURRENT RESULTS ━━━━━━━━━━");
        telemetry.addData("Left Front    ", resultIcon(leftFrontResult));
        telemetry.addData("Right Front   ", resultIcon(rightFrontResult));
        telemetry.addData("Left Back     ", resultIcon(leftBackResult));
        telemetry.addData("Right Back    ", resultIcon(rightBackResult));
        telemetry.addData("Intake        ", resultIcon(intakeResult));
        telemetry.addData("Left Launcher ", resultIcon(leftLauncherResult));
        telemetry.addData("Right Launcher", resultIcon(rightLauncherResult));
    }

    // =========================================================
    // HANDLE YES/NO CONFIRMATION AFTER MOTOR STOPS
    // =========================================================
    private void handleConfirmation() {
        if (!awaitingConfirm) {
            awaitingConfirm = true;
            return;
        }

        if (gamepad1.a) {
            recordResult(activeTest, Result.CORRECT);
            activeTest      = ActiveTest.NONE;
            awaitingConfirm = false;
        } else if (gamepad1.b) {
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
            case LEFT_FRONT:     leftFrontResult      = result; break;
            case RIGHT_FRONT:    rightFrontResult     = result; break;
            case LEFT_BACK:      leftBackResult       = result; break;
            case RIGHT_BACK:     rightBackResult      = result; break;
            case INTAKE_IN:
            case INTAKE_OUT:     intakeResult         = result; break;
            case LEFT_LAUNCHER:  leftLauncherResult   = result; break;
            case RIGHT_LAUNCHER: rightLauncherResult  = result; break;
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

        // ── Drivetrain ──
        telemetry.addLine("-- DRIVETRAIN --");
        telemetry.addData("leftFrontDrive  direction", recommendDrive(leftFrontResult,  true));
        telemetry.addData("rightFrontDrive direction", recommendDrive(rightFrontResult, false));
        telemetry.addData("leftBackDrive   direction", recommendDrive(leftBackResult,   true));
        telemetry.addData("rightBackDrive  direction", recommendDrive(rightBackResult,  false));
        telemetry.addLine("");

        // ── Intake ──
        telemetry.addLine("-- INTAKE --");
        if (intakeResult == Result.NOT_TESTED) {
            telemetry.addLine("intake: ⏳ NOT TESTED yet");
        } else if (intakeResult == Result.CORRECT) {
            telemetry.addLine("intake: setDirection(FORWARD)  ✅");
        } else {
            telemetry.addLine("intake: setDirection(REVERSE)  ❌ flip it");
        }
        telemetry.addLine("");

        // ── Launchers ──
        telemetry.addLine("-- LAUNCHERS --");
        telemetry.addData("left_launcher  direction", recommendLauncher(leftLauncherResult,  true));
        telemetry.addData("right_launcher direction", recommendLauncher(rightLauncherResult, false));
        telemetry.addLine("");

        // ── All-tested check ──
        boolean anyUntested =
                leftFrontResult      == Result.NOT_TESTED ||
                        rightFrontResult     == Result.NOT_TESTED ||
                        leftBackResult       == Result.NOT_TESTED ||
                        rightBackResult      == Result.NOT_TESTED ||
                        intakeResult         == Result.NOT_TESTED ||
                        leftLauncherResult   == Result.NOT_TESTED ||
                        rightLauncherResult  == Result.NOT_TESTED;

        if (anyUntested) {
            telemetry.addLine("⚠ Some motors not tested yet.");
            telemetry.addLine("  Press Y to go back and test them.");
        } else {
            telemetry.addLine("✅ All motors tested!");
            telemetry.addLine("  Copy the directions above into");
            telemetry.addLine("  your main TeleOp init() block.");
        }

        telemetry.addLine("");
        telemetry.addLine("Press Y to go back | GP1 BACK to reset");
    }

    // =========================================================
    // HELPERS
    // =========================================================

    /**
     * Drive motor recommendation.
     * Standard mecanum: left = FORWARD, right = REVERSE.
     * If test says NEEDS_REVERSE, flip from the standard.
     */
    private String recommendDrive(Result result, boolean isLeftSide) {
        if (result == Result.NOT_TESTED) return "⏳ NOT TESTED";
        boolean useForward;
        if (isLeftSide) {
            useForward = (result == Result.CORRECT);
        } else {
            useForward = (result == Result.NEEDS_REVERSE);
        }
        return useForward ? "✅ FORWARD" : "❌ REVERSE";
    }

    /**
     * Launcher recommendation.
     * For a dual-flywheel shooter facing the same direction, the two
     * wheels typically spin in opposite directions (one FORWARD, one
     * REVERSE) to both push the ring upward/outward.
     *
     * Convention used here:
     *   left_launcher  → expected to be FORWARD  when spinning outward
     *   right_launcher → expected to be REVERSE  when spinning outward
     *
     * If the test says the motor was already correct at FORWARD power,
     * use FORWARD; if it needs to be flipped, use REVERSE.
     */
    private String recommendLauncher(Result result, boolean isLeftSide) {
        if (result == Result.NOT_TESTED) return "⏳ NOT TESTED";

        // Left launcher expected FORWARD; right expected REVERSE.
        // If the test result agrees with that expectation → use the standard direction.
        // If it disagrees → flip it.
        boolean useForward;
        if (isLeftSide) {
            useForward = (result == Result.CORRECT);       // left: CORRECT → FORWARD
        } else {
            useForward = (result == Result.NEEDS_REVERSE); // right: CORRECT at FORWARD → actually needs REVERSE
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
        if (leftLauncher    != null) leftLauncher.setPower(0);
        if (rightLauncher   != null) rightLauncher.setPower(0);
    }

    private void resetAll() {
        stopAllMotors();
        activeTest           = ActiveTest.NONE;
        showVerdict          = false;
        awaitingConfirm      = false;
        leftFrontResult      = Result.NOT_TESTED;
        rightFrontResult     = Result.NOT_TESTED;
        leftBackResult       = Result.NOT_TESTED;
        rightBackResult      = Result.NOT_TESTED;
        intakeResult         = Result.NOT_TESTED;
        leftLauncherResult   = Result.NOT_TESTED;
        rightLauncherResult  = Result.NOT_TESTED;
    }

    @Override
    public void stop() {
        stopAllMotors();
    }
}