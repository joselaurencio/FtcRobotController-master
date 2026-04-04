package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * LaunchArtifact subsystem — handles feeder servo firing logic.
 * Imported from TeleOp: leftFeeder, rightFeeder (CRServo)
 */
public class LaunchArtifact {

    private CRServo leftFeeder;
    private CRServo rightFeeder;

    private final double FEED_TIME_SECONDS = 3;
    private final double STOP_SPEED = 0.0;
    private final double FULL_SPEED = 1.0;

    private ElapsedTime leftFeederTimer = new ElapsedTime();
    private ElapsedTime rightFeederTimer = new ElapsedTime();

    // State enums
    private enum FeederState { IDLE, FEEDING }
    private FeederState leftFeederState = FeederState.IDLE;
    private FeederState rightFeederState = FeederState.IDLE;

    /**
     * Constructor — pass in only the feeder servos.
     */
    public LaunchArtifact(CRServo leftFeeder, CRServo rightFeeder) {
        this.leftFeeder = leftFeeder;
        this.rightFeeder = rightFeeder;

        this.leftFeeder.setPower(STOP_SPEED);
        this.rightFeeder.setPower(STOP_SPEED);
    }

    /**
     * Fire left feeder — call once per shot request.
     */
    public void fireLeftFeeder() {
        if (leftFeederState == FeederState.IDLE) {
            leftFeederState = FeederState.FEEDING;
            leftFeeder.setPower(FULL_SPEED);
            leftFeederTimer.reset();
        }
    }

    /**
     * Fire right feeder — call once per shot request.
     */
    public void fireRightFeeder() {
        if (rightFeederState == FeederState.IDLE) {
            rightFeederState = FeederState.FEEDING;
            rightFeeder.setPower(FULL_SPEED);
            rightFeederTimer.reset();
        }
    }

    /**
     * Update feeder state machines — call every loop.
     */
    public void update() {
        // Left feeder state machine
        switch (leftFeederState) {
            case IDLE:
                // waiting for fireLeftFeeder() call
                break;

            case FEEDING:
                if (leftFeederTimer.seconds() > FEED_TIME_SECONDS) {
                    leftFeeder.setPower(STOP_SPEED);
                    leftFeederState = FeederState.IDLE;
                }
                break;
        }

        // Right feeder state machine
        switch (rightFeederState) {
            case IDLE:
                // waiting for fireRightFeeder() call
                break;

            case FEEDING:
                if (rightFeederTimer.seconds() > FEED_TIME_SECONDS) {
                    rightFeeder.setPower(STOP_SPEED);
                    rightFeederState = FeederState.IDLE;
                }
                break;
        }
    }

    /**
     * Get left feeder state (for telemetry).
     */
    public FeederState getLeftFeederState() {
        return leftFeederState;
    }

    /**
     * Get right feeder state (for telemetry).
     */
    public FeederState getRightFeederState() {
        return rightFeederState;
    }
}