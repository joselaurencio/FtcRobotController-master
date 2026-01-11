package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {

    private DcMotor left;
    private DcMotor right;

    // ===== TUNABLE CONSTANTS =====
    private static final double DEADZONE_DEG = 0.75;   // degrees
    private static final double MAX_TURN_POWER = 0.45;
    private static final double MIN_TURN_POWER = 0.08;

    public Drivetrain(HardwareMap hw) {
        left = hw.get(DcMotor.class, "leftDrive");
        right = hw.get(DcMotor.class, "rightDrive");

        right.setDirection(DcMotor.Direction.REVERSE);
    }

    public void alignToTarget(double tx) {

        // Deadband
        if (Math.abs(tx) < DEADZONE_DEG) {
            stop();
            return;
        }

        // === Nonlinear power curve (your equation) ===
        double error = Math.abs(tx);

        double rawPower =
                32445.52 * Math.pow(error, -1.996884);

        // Normalize & clamp
        double turnPower = clip(rawPower, MIN_TURN_POWER, MAX_TURN_POWER);

        // Reapply sign
        turnPower *= Math.signum(tx);

        // Tank turn
        left.setPower(turnPower);
        right.setPower(-turnPower);
    }

    public void stop() {
        left.setPower(0);
        right.setPower(0);
    }

    private double clip(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
