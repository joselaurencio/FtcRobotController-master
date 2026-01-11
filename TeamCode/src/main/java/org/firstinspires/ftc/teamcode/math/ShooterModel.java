package org.firstinspires.ftc.teamcode.math;

public class ShooterModel {

    // Constants (paper-backed)
    private static final double g = 9.81;
    private static final double theta = Math.toRadians(55);
    private static final double wheelRadius = 0.106; // meters
    private static final double efficiency = 0.85;

    private static final double eta = .6;

    private static final double shooterHeight = 0.508; // meters
    private static final double targetHeight = 0.6096;  // meters

    public static double distanceToRPM(double d) {

        double numerator = g * d * d;
        double denominator = 2 * Math.pow(Math.cos(theta), 2)
                * (d * Math.tan(theta) - (targetHeight - shooterHeight));

        if (denominator <= 0) return 0;

        double v0 = Math.sqrt(numerator / denominator);
        v0 /= efficiency;

        return (v0  * 60 / (2 * Math.PI * wheelRadius * eta));
    }
}
