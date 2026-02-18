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

public class ConstantsChasis {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(5.75)
            .forwardZeroPowerAcceleration(-44.10929477087724)
            .lateralZeroPowerAcceleration(-76.6680988559598)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.05,
                    0.0,
                    0.1,
                    0.03
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.5,
                    0.0,
                    0.01,
                    0.02
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.015,
                    0.0,
                    0.00075,
                    0.6,
                    0.035
            ))
            .centripetalScaling(0.0005);


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.9)
            .rightFrontMotorName("motorFrontRight")
            .rightRearMotorName("motorRearRight")
            .leftRearMotorName("motorRearLeft")
            .leftFrontMotorName("motorFrontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(61.58374007417774)
            .yVelocity(54.20592188397105);



    public static PinpointConstants localizerConstants = new PinpointConstants()
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .forwardPodY(-6.29921)
            .strafePodX(-1.81102);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();

    }




}
