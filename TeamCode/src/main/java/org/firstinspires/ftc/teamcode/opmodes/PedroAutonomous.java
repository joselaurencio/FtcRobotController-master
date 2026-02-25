package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(121.720, 124.766),
                                    new Pose(80.625, 87.127)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(40))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(80.625, 87.127),
                                    new Pose(127.313, 83.274)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(127.313, 83.274),
                                    new Pose(81.117, 87.316)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(81.117, 87.316),
                                    new Pose(82.314, 55.256),
                                    new Pose(127.577, 58.264)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(127.577, 58.264),
                                    new Pose(81.560, 87.107)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        switch (pathState) {
            case 0:
                // Start the first path
                follower.followPath(paths.Path1);
                pathState = 1;
                break;

            case 1:
                // Check if the follower is finished with Path1
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2);
                    pathState = 2;
                }
                break;

            case 2:
                // Check if finished with Path2
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    pathState = 3;
                }
                break;
            case 3:
                // Check if finished with Path3
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4);
                    pathState = 4;
                }
                break;
            // Continue for paths 4 and 5...
            case 4:
                // Check if finished with Path3
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    // Done!
                    pathState = -1;
                }
                break;
        }
        return pathState;
    }
}