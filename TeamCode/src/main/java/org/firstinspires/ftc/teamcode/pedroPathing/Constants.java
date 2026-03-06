package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.41)
            .forwardZeroPowerAcceleration(-20.02836888108038)
            .lateralZeroPowerAcceleration(-53.12910638685177)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.08,
                    0,
                    0.005,
                    0.04
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.65,
                    0.0,
                    0.00008,
                    0.04
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.05,
                    0,
                    0.000285,
                    0.6,
                    0.015
            ))
            .centripetalScaling(0.0001)
            .useSecondaryDrivePIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryTranslationalPIDF(false);;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, .9, 1);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(3)
            .strafePodX(-7.5)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .xVelocity(63.92616235928271)
            .yVelocity(51.341422974594)
            .maxPower(1)
            .rightFrontMotorName("right_front_drive")
            .rightRearMotorName("right_back_drive")
            .leftRearMotorName("left_back_drive")
            .leftFrontMotorName("left_front_drive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}