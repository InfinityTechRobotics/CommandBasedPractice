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

public class ConstantsStates {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.05)
            .forwardZeroPowerAcceleration(-36.4161)
            .lateralZeroPowerAcceleration(-82.1192)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.08,
                    0.0,
                    0.01,
                    0.03
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.8,
                    0.0,
                    0.02,
                    0.03
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.01,
                    0.0,
                    0.0005,
                    0.6,
                    0.05
            ))
            .centripetalScaling(0.0005);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.98)
            .rightFrontMotorName("motorFrontRight")
            .rightRearMotorName("motorRearRight")
            .leftRearMotorName("motorRearLeft")
            .leftFrontMotorName("motorFrontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(67.5188)
            .yVelocity(48.4023);



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