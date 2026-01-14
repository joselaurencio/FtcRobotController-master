package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimelightVision {

    private final Limelight3A limelight;
    private LLResult latest;

    // =========================
    // CONFIG (MEASURE THESE)
    // =========================
    private static final double CAMERA_HEIGHT = 0.4445; // meters
    private static final double TARGET_HEIGHT = 0.7495; // meters
    private static final double CAMERA_PITCH_DEG = 0.0; // degrees (positive = up)

    // Area-based calibration (OPTIONAL)
    private static final double AREA_SCALE = 32445.52;
    private static final double AREA_EXPONENT = -1.996884;

    public LimelightVision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();
    }

    /** Call once per loop */
    public void update() {
        latest = limelight.getLatestResult();
    }

    // =========================
    // STATUS
    // =========================
    public boolean hasTarget() {
        return latest != null && latest.isValid();
    }

    // =========================
    // RAW LIMELIGHT VALUES
    // =========================
    public double getTx() {
        return hasTarget() ? latest.getTx() : 0.0;
    }

    public double getTy() {
        return hasTarget() ? latest.getTy() : 0.0;
    }

    public double getTa() {
        return hasTarget() ? latest.getTa() : 0.0;
    }

    // =========================
    // DISTANCE METHODS
    // =========================

    /**
     * ✅ OFFICIAL Limelight distance method
     * Uses fixed camera angle + ty
     */
    public double getTrigDistanceMeters() {
        if (!hasTarget()) return 0.0;

        double angleDeg = CAMERA_PITCH_DEG + getTy();
        double angleRad = Math.toRadians(angleDeg);

        if (Math.abs(Math.tan(angleRad)) < 1e-6) return 0.0;

        return (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleRad);
    }

    /**
     * ⚠️ Experimental area-based distance
     * Requires careful calibration
     */
    public double getAreaDistanceMeters() {
        if (!hasTarget()) return 0.0;

        double ta = getTa();
        if (ta <= 0.01) return 0.0;

        // Convert from cm → meters
        return (AREA_SCALE * Math.pow(ta, AREA_EXPONENT)) / 100.0;
    }

    /**
     * Recommended distance for competition
     */
    public double getBestDistanceMeters() {
        return getTrigDistanceMeters();
    }
}
