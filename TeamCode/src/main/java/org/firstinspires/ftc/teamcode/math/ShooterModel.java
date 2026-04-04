package org.firstinspires.ftc.teamcode.math;

public class ShooterModel {

    // ===== PHYSICS CONSTANTS (UNCHANGED) =====
    private static final double g = 9.81;
    private static final double theta = Math.toRadians(48);
    private static final double wheelRadius = 0.196;
    private static final double eta = 0.550;

    private static final double shooterHeight = 0.43;
    private static final double targetHeight = 1.1;

    // ===== SWITCH BETWEEN MODELS =====
    private static final boolean USE_LINEAR_MODEL = true;

    // ===== YOUR NEW REGRESSIONS =====
    // LEFT SHOOTER
    private static final double LEFT_M = 6.23098;
    private static final double LEFT_B = 2240.76324;

    // RIGHT SHOOTER
    private static final double RIGHT_M = 6.70697;
    private static final double RIGHT_B = 2010.40165;

    /**
     * Returns RPM based on distance and which shooter is being used
     * @param d distance in CM
     * @param isLeftShooter true = left shooter, false = right shooter
     */
    public static double distanceToRPM(double d, boolean isLeftShooter) {

        if (USE_LINEAR_MODEL) {

            double rpm;

            if (isLeftShooter) {
                // ===== LEFT MODEL =====
                rpm = LEFT_M * d + LEFT_B;
            } else {
                // ===== RIGHT MODEL =====
                rpm = RIGHT_M * d + RIGHT_B;
            }

            return Math.max(rpm, 0);
        }

        // ===== PHYSICS MODEL (FALLBACK) =====

        double meters = d / 100.0;

        double deltaY = targetHeight - shooterHeight;
        double numerator = g * Math.pow(meters, 2);
        double denominator = 2 * Math.pow(Math.cos(theta), 2)
                * (meters * Math.tan(theta) - deltaY);

        if (denominator <= 0) return 0;

        double v0 = Math.sqrt(numerator / denominator);

        double rimVelocityRequired = v0 * 2;

        double rpm = (rimVelocityRequired * 60)
                / (2 * Math.PI * wheelRadius * eta);

        return rpm;
    }
}