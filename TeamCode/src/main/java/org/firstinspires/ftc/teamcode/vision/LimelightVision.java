package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimelightVision {

    private final Limelight3A limelight;
    private LLResult latest;

    // =========================
    // CALIBRATION CONSTANTS
    // =========================
    // From your collected data + curve fitting
    private static final double AREA_SCALE = 41061.19;
    private static final double AREA_EXPONENT = -2.050188;

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
    // TARGET STATUS
    // =========================

    public boolean hasTarget() {
        return latest != null && latest.isValid();
    }

    // =========================
    // LIMELIGHT RAW VALUES
    // =========================

    public double getTx() {
        return hasTarget() ? latest.getTx() : 0.0;
    }

    public double getTa() {
        return hasTarget() ? latest.getTa() : 0.0;
    }

    // =========================
    // DISTANCE (AREA METHOD)
    // =========================

    /**
     * Distance to AprilTag using calibrated target area power model
     * Units depend on calibration (cm or meters — be consistent!)
     */
    public double getDistanceFromArea() {
        if (!hasTarget()) return 0.0;

        double ta = getTa();
        if (ta <= 0.0001) return 0.0;

        double distanceWithoutOffsetAdjustment = Math.pow((ta / AREA_SCALE), (1.0 / AREA_EXPONENT));
        double distanceWithOffsetAdjustment = distanceWithoutOffsetAdjustment + 25.5;
        // x = (ta / scale)^(1 / exponent)
        return distanceWithOffsetAdjustment;
    }

    // =========================
    // DEBUG SUPPORT
    // =========================
    // =========================
// APRILTAG ID
// =========================

    // =========================
// APRILTAG / OBELISK ID
// =========================

    public int getObeliskNumber() {

        if (!hasTarget()) return -1;

        if (latest.getFiducialResults() == null ||
                latest.getFiducialResults().isEmpty()) {
            return -1;
        }

        // SimpleList method: take first detected tag
        return latest.getFiducialResults()
                .get(0)
                .getFiducialId();
    }
    public LLResult getRawResult() {
        return latest;
    }
}
