package org.firstinspires.ftc.teamcode.math;

public class ShooterModel {

    // Constants grounded in the paper's physical parameters
    private static final double g = 9.81;
    private static final double theta = Math.toRadians(55); // Launch angle
    private static final double wheelRadius = 0.106;       // Flywheel radius

    // Consolidated Efficiency Factor (η)
    // Accounts for slip, compression, and momentum transfer
    private static final double eta = 0.50;

    private static final double shooterHeight = 0.508;
    private static final double targetHeight = 1.1;

    public static double distanceToRPM(double d) {
        // 1. Calculate required exit velocity (v0)
        double deltaY = targetHeight - shooterHeight;
        double numerator = g * Math.pow(d, 2);
        double denominator = 2 * Math.pow(Math.cos(theta), 2) * (d * Math.tan(theta) - deltaY);

        if (denominator <= 0) return 0;

        double v0 = Math.sqrt(numerator / denominator);

        // 2. Convert v0 to required Flywheel RPM
        // Note: For a single-wheel hooded shooter, rim speed = 2 * v0.
        // Formula: RPM = (60 * (v0 * 2)) / (2 * PI * r * eta)
        double rimVelocityRequired = v0 * 2;

        double rpm = (rimVelocityRequired * 60) / (2 * Math.PI * wheelRadius * eta);

        return rpm;
    }
}