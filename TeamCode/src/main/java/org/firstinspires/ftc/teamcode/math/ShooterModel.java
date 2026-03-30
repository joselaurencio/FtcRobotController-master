package org.firstinspires.ftc.teamcode.math;

public class ShooterModel {

    // Constants grounded in the paper's physical parameters
    private static final double g = 9.81;
    private static final double theta = Math.toRadians(48); // Launch angle
    private static final double wheelRadius = 0.196;       // Flywheel radius

    // Consolidated Efficiency Factor (η)
    // Accounts for slip, compression, and momentum transfer
    private static final double eta = 0.550;

    private static final double shooterHeight = 0.43;
    private static final double targetHeight = 1.1;

    private static final boolean USE_LINEAR_MODEL = true;

    public static double distanceToRPM(double d) {

        // d is in CENTIMETERS

        if (USE_LINEAR_MODEL) {
            // Linear regression: RPM = 8.54396 * distance + 1718.5989
            double rpm = 8.54396 * d + 1718.5989;

            return Math.max(rpm, 0);
        }

        // ===== ORIGINAL PHYSICS MODEL =====

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