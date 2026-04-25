package org.firstinspires.ftc.teamcode.DisabledCompetition;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.DisabledHardware.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous
    public class altAutoBlueStates extends OpMode {
    Shooter shooter = new Shooter();

    double START_DELAY_TIME = 5.0;

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    public static double NEW_P = 100.;   // 10.
    public static double NEW_I = 1.0;    // 3.00
    public static double NEW_D = 20.0;    // 0.0
    public static double NEW_F = 3.5;    // 0.0

    private final Pose startPose = new Pose(64, 8, Math.toRadians(90));
    private final Pose leavePose = new Pose(46, 18, Math.toRadians(180));
    private final Pose prePickupPose21 = new Pose(54, 36, Math.toRadians(180)); // Preparing to intake third set of artifacts.
    private final Pose pickupPose21 = new Pose(21, 36, Math.toRadians(180)); // Last (Third Set) of Artifacts from the Spike Mark(GPP).
    private final Pose scorePose = new Pose(61, 16, Math.toRadians(115));
    private final Pose prePickupPoseLoadingZone = new Pose(21, 36, Math.toRadians(270));
    private final Pose pickupPoseLoadingZone = new Pose(21, 18, Math.toRadians(270));
    private final Pose controlPoint1 = new Pose(56, 48); // Control point - you get the idea - read the name
    private final Pose controlPoint2 = new Pose(56, 72); // Control point - you get the idea - read the name


    private PathChain driveStartToScore, driveToScore, driveToLeave, driveToPrePickup21, driveToPickup21, driveToPrePickupLoadingZone, driveToPickupLoadingZone, driveToLoadingScore;

    DcMotorEx motorIntake, motorTransfer;
    DcMotorEx motorFlywheel;

    double targetRPM = 0.0;
    double flywheelRPM = 0.0;
    double TPS;
    double CPR = 28.0;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;

    double transferOn = 0.8;
    double transferOff = 0.0;

    double intakeOn = 0.8;
    double intakeOff = 0.0;

    double TARGET_AUTON_RPM = 3100.0;

    int shootingSequenceFlag = 1;

    PIDFCoefficients pidfModified;



    private void buildPaths() {

        driveStartToScore = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToPrePickup21 = follower.pathBuilder()
                .addPath(new BezierCurve(pickupPose21, prePickupPose21))
                .setLinearHeadingInterpolation(pickupPose21.getHeading(), prePickupPose21.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToPickup21 = follower.pathBuilder()
                .addPath(new BezierLine(prePickupPose21, pickupPose21))
                .setLinearHeadingInterpolation(prePickupPose21.getHeading(), pickupPose21.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToScore = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose21, scorePose))
                .setLinearHeadingInterpolation(pickupPose21.getHeading(), scorePose.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToPrePickupLoadingZone = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, controlPoint1, prePickupPoseLoadingZone))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickupPoseLoadingZone.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToPickupLoadingZone = follower.pathBuilder()
                .addPath(new BezierLine(prePickupPoseLoadingZone, pickupPoseLoadingZone))
                .setLinearHeadingInterpolation(prePickupPoseLoadingZone.getHeading(), pickupPoseLoadingZone.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToLoadingScore = follower.pathBuilder()
                .addPath(new BezierCurve(pickupPoseLoadingZone, controlPoint2, scorePose))
                .setLinearHeadingInterpolation(pickupPoseLoadingZone.getHeading(), scorePose.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToLeave = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, leavePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leavePose.getHeading())
                .setTimeoutConstraint(0)
                .build();

    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10:
                if (pathTimer.getElapsedTimeSeconds() > START_DELAY_TIME) {
                    shooter.closeServoStop();
                    shooter.downServoPaddle();
                    follower.followPath(driveStartToScore);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    setPathState(1000);
                }
                break;
            case 1000:
                if (shootingSequenceFlag == 33) {
                    shootingSequenceFlag = 777;
                } else if (shootingSequenceFlag == 21) {
                    shootingSequenceFlag = 33;
                } else if (shootingSequenceFlag == 1) {
                    shootingSequenceFlag = 21;
                }
                setPathState(1001);
                break;
            case 1001:
                if (!follower.isBusy()) {
                    motorIntake.setPower(intakeOn);
                    motorTransfer.setPower(transferOn);
                    setPathState(10001);
                }
                break;
            case 10001:
                if (pathTimer.getElapsedTimeSeconds() > 0.75) { // changed from 0.5 to 0.25
                    shooter.openServoStop();
                    setPathState(10002);
                }
                break;
            case 10002:
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                    shooter.closeServoStop();
                    setPathState(10003);
                }
                break;
            case 10003:
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    shooter.openServoStop();
                    setPathState(10004);
                }
                break;
            case 10004:
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                    shooter.closeServoStop();
                    setPathState(10005);
                }
                break;
            case 10005:
                if (pathTimer.getElapsedTimeSeconds() > 0.75) {
                    shooter.openServoStop();
                    setPathState(10006);
                }
                break;
            case 10006:
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                    shooter.shootServoPaddle();
                    setPathState(10007);
                }
                break;
            case 10007:
                if (pathTimer.getElapsedTimeSeconds() > 0.6) {
                    shooter.downServoPaddle();
                    shooter.closeServoStop();
                    motorIntake.setPower(intakeOff);
                    setPathState(10008);
                }
                break;
            case 10008:
                setPathState(10009);
                break;
            case 10009:  // sends to next driving path
                if (shootingSequenceFlag == 21) {
                    setPathState(210);
                } else if (shootingSequenceFlag == 33) {
                    setPathState(330);
                } else {
                    setPathState(777);
                }
                break;
            case 210: // beginning of set of actions for spike mark 21, gets ready to pickup
                if (!follower.isBusy()) {
                    follower.followPath(driveToPrePickup21);
                    setPathState(211);
                }
                break;
            case 211: // timer case
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    if (!follower.isBusy()) {
                        motorIntake.setPower(intakeOn);
                        setPathState(212);
                    }
                }
                break;
            case 212: // picks up first set of artifacts (21)
                if (!follower.isBusy()) {
                    follower.followPath(driveToPickup21);
                    setPathState(213);
                }
                break;
            case 213: // drives toward goal to score first set of artifacts (21)
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    if (!follower.isBusy()) {
                        follower.followPath(driveToScore);
                        setPathState(214);
                    }
                }
                break;
            case 214: // case for shooting
                if (!follower.isBusy()) {
                    setPathState(1000);
                }
                break;
            case 330: // beginning of set of actions for spike mark 21, gets ready to pickup
                if (!follower.isBusy()) {
                    follower.followPath(driveToPrePickupLoadingZone);
                    setPathState(331);
                }
                break;
            case 331: // timer case
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    if (!follower.isBusy()) {
                        motorIntake.setPower(intakeOn);
                        setPathState(332);
                    }
                }
                break;
            case 332: // picks up first set of artifacts (21)
                if (!follower.isBusy()) {
                    follower.followPath(driveToPickupLoadingZone);
                    setPathState(333);
                }
                break;
            case 333: // drives toward goal to score first set of artifacts (21)
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    motorIntake.setPower(intakeOff);
                    if (!follower.isBusy()) {
                        follower.followPath(driveToLoadingScore);
                        setPathState(334);
                    }
                }
                break;
            case 334: // case for shooting
                if (!follower.isBusy()) {
                    setPathState(1000);
                }
                break;
            case 777:
                if (!follower.isBusy()) {
                    follower.followPath(driveToLeave);
                    TARGET_AUTON_RPM = 0.0;
                    setPathState(999);
                }
                break;
            case 999: // last state, just stops and turns things off
                if(!follower.isBusy()) {
                    motorTransfer.setPower(transferOff);
                    motorIntake.setPower(intakeOff);
                    setPathState(912);
                }
                break;
        }
    }

    private void setPathState(int pState) {

        pathState = pState;
        pathTimer.resetTimer();

    }

    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();

        targetRPM = TARGET_AUTON_RPM;
        TPS = targetRPM / 60. * CPR;
        motorFlywheel.setVelocity(TPS);
        flywheelRPM = motorFlywheel.getVelocity() * 60 / CPR;

        telemetry.addData("Path State", pathState); // the current path the code is running
        telemetry.addData("X", follower.getPose().getX()); // x pos
        telemetry.addData("Y", follower.getPose().getY()); // y pos
        telemetry.addData("Heading", follower.getPose().getHeading()); // heading
        telemetry.addData("Path Timer",pathTimer.getElapsedTimeSeconds());
        telemetry.addData("OpMode Timer", opmodeTimer.getElapsedTimeSeconds());
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Flywheel RPM", flywheelRPM);
        telemetry.addData("Flywheel Motor Power", motorFlywheel.getPower());
        telemetry.addData("Shooting Sequence", shootingSequenceFlag);
        telemetry.addData("P,I,D,F (modified)", "P: %.4f, I: %.4f, D: %.4f, F: %.4f",
                pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);
        telemetry.update();

    }

    @Override
    public void init() {

        shooter.init(hardwareMap);

        motorIntake = hardwareMap.get(DcMotorEx.class, "motorIntake");
        motorTransfer = hardwareMap.get(DcMotorEx.class, "motorTransfer");
        motorFlywheel = hardwareMap.get(DcMotorEx.class, "motorFlywheel");

        motorIntake.setDirection(DcMotorEx.Direction.FORWARD);
        motorTransfer.setDirection(DcMotorEx.Direction.FORWARD);
        motorFlywheel.setDirection(DcMotorEx.Direction.FORWARD);

        motorFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorTransfer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        motorFlywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

        pidfModified = motorFlywheel.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        shooter.closeServoStop();
        shooter.downServoPaddle();

        shooter.centerServoTurret();

    }

    @Override
    public void init_loop() {
        // Nothing
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(10);
    }


    @Override
    public void stop() {
        // Nothing
    }



}