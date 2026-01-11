package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightVision {

    private final Limelight3A limelight;
    private LLResult latest;

    // =========================
    // CONFIG CONSTANTS
    // =========================
    private static final double CAMERA_HEIGHT = 0.4445;    // meters
    private static final double TARGET_HEIGHT = 0.6096;    // meters (goal)
    private static final double CAMERA_PITCH_RAD = Math.toRadians(10);

    public LimelightVision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();
    }

    /** Call ONCE per loop */
    public void update() {
        latest = limelight.getLatestResult();
    }

    // =========================
    // BASIC STATUS
    // =========================

    public boolean hasTarget() {
        return latest != null && latest.isValid();
    }

    public boolean hasPose() {
        return hasTarget() && latest.getBotpose_MT2() != null;
    }

    // =========================
    // LIMELIGHT 2D VALUES
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
    // APRILTAG POSE (3D)
    // =========================

    public Pose3D getBotPose() {
        return hasPose() ? latest.getBotpose_MT2() : null;
    }

    public double getPoseX() {
        return hasPose() ? latest.getBotpose_MT2().getPosition().x : 0.0;
    }

    public double getPoseY() {
        return hasPose() ? latest.getBotpose_MT2().getPosition().y : 0.0;
    }

    public double getPoseZ() {
        return hasPose() ? latest.getBotpose_MT2().getPosition().z : 0.0;
    }

    // =========================
    // DISTANCE CALCULATIONS
    // =========================

    /**
     * AprilTag-based horizontal distance (BEST when available)
     * Uses robot pose relative to tag (matches video)
     */
    public double getDistanceMeters() {
        if (!hasPose()) return 0.0;

        double x = getPoseX();
        double z = getPoseZ();

        return Math.hypot(x, z);
    }

    /**
     * Trig-based distance fallback using ty
     * Used when AprilTag pose is unavailable
     */
    public double getTrigDistanceMeters() {
        if (!hasTarget()) return 0.0;

        double tyRad = Math.toRadians(getTy());
        double totalAngle = CAMERA_PITCH_RAD + tyRad;

        if (totalAngle <= 0.01) return 0.0;

        double heightDiff = TARGET_HEIGHT - CAMERA_HEIGHT;
        return heightDiff / Math.tan(totalAngle);
    }

    /**
     * Smart distance selector:
     * - Prefer AprilTag pose
     * - Fallback to trig distance
     */
    public double getBestDistanceMeters() {
        double poseDist = getDistanceMeters();
        if (poseDist > 0.05) {
            return poseDist;
        }
        return getTrigDistanceMeters();
    }

    // =========================
    // DEBUG / PAPER SUPPORT
    // =========================

    public boolean usingAprilTagDistance() {
        return hasPose() && getDistanceMeters() > 0.05;
    }

    public LLResult getRawResult() {
        return latest;
    }
}
