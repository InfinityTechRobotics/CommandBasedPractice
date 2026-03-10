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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//This is the one we're using 3/8/26

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.05)
            .forwardZeroPowerAcceleration(-34.9424)
            .lateralZeroPowerAcceleration(-75.19009)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.125,
                    0.0,
                    0.01,
                    0.04
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.15,
                    0.0,
                    0.125,
                    0.03
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.2,
                    0.0,
                    0.0001,
                    0.,
                    0.07
            ))
            .centripetalScaling(0.0003);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            .rightFrontMotorName("motorFrontRight")
            .rightRearMotorName("motorRearRight")
            .leftRearMotorName("motorRearLeft")
            .leftFrontMotorName("motorFrontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(75.3298)
            .yVelocity(61.01995);



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
