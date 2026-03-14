package org.firstinspires.ftc.teamcode.SKR.auton;

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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous
public class BlueGoalAutonV1 extends OpMode {

    boolean COMPETITION_MODE = false;

    private int obeliskResult = 0;

    double START_DELAY_TIME = 2.;

    private Shooter shooter;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

//    public static double NEW_P = 100.;   // 10.
//    public static double NEW_I = 1.;    // 3.
//    public static double NEW_D = 20.;    // 0.
//    public static double NEW_F = 3.5;    // 0.

    private final Pose startPose = new Pose(28, 129, Math.toRadians(144)); // Start Pose of our robot.
//    private final Pose viewPose = new Pose(80, 120, Math.toRadians(90)); // Pose to read the Obelisk.

//        private final Pose motifDetection = new Pose(59, 110, Math.toRadians(90));

    //        private final Pose scorePose = new Pose(52, 101, Math.toRadians(138)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePose = new Pose(52, 101, Math.toRadians(142.5)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private final Pose prePickupPose21 = new Pose(48, 39.5, Math.toRadians(180)); // Preparing to intake third set of artifacts.

    private final Pose pickupPose21 = new Pose(22, 39.5, Math.toRadians(180)); // Last (Third Set) of Artifacts from the Spike Mark(GPP).

    private final Pose prePickupPose22 = new Pose(50, 60.5, Math.toRadians(180)); // Preparing to intake second set of artifacts.

    private final Pose pickupPose22 = new Pose(22, 60.5, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark(PGP).

    private final Pose prePickupPose23 = new Pose(50, 88, Math.toRadians(180)); // Preparing to intake first set of artifacts.

    private final Pose pickupPose23 = new Pose(26, 88, Math.toRadians(180));// Highest (First Set) of Artifacts from the Spike Mark(PPG).

    private final Pose preGateHit = new Pose(30, 78, Math.toRadians(90));

    private final Pose gateHit = new Pose( 16, 78, Math.toRadians(90));

    private final Pose controlPoint1 = new Pose(54, 0); // Control point - you get the idea - read the name

    private final Pose controlPoint2 = new Pose(80, 63.5); // 67! ;) - you should get what this means by now - read the name aigin - idk i cant spel

    private  final Pose endPose = new Pose(54, 70, Math.toRadians(180));//its the end - if you took the time to read this, you get it - otherwise vid is disappointed :(

    private final Pose finalShootPose = new Pose (52, 113, Math.toRadians(150));

    private PathChain driveToGoal, driveToPrePickup23, driveToPickup23, driveToGoal23, driveToPrePickup22, driveToPickup22, driveToGoal22, driveToPrePickup21, driveToPickup21, driveToGoal21, driveToEnd, driveToGate, driveGateToGoal;

    DcMotorEx motorIntake, motorTransfer, motorFlywheel;
    //DcMotorEx motorFlywheel;

    //    double targetRPM = 0.;
    double flywheelRPM = 0.;
    double TPS;
//    double CPR = 28.;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;

    double transferOn = 0.8;
    double transferOff = 0.;

    double intakeOn = 0.8;
    double intakeOff = 0.0;

    double TARGET_AUTON_RPM = 2250.;

    int shootingSequenceFlag = 1;

    PIDFCoefficients pidfModified;

    private void buildPaths() {
        driveToGoal = buildDrivePath(follower, startPose, scorePose, false);

        // The paths for the first set of artifacts(PPG) are below
        driveToPrePickup23 = buildDrivePath(follower, scorePose, prePickupPose23, true);

        driveToPickup23 = buildDrivePath(follower, prePickupPose23, pickupPose23, true);

        driveToGoal23 = buildDrivePath(follower, pickupPose23, scorePose, true);

        // The paths for the second set of artifacts(PGP) are below
        driveToPrePickup22 = buildDrivePath(follower, scorePose, prePickupPose22, true);

        driveToPickup22 = buildDrivePath(follower, prePickupPose22, pickupPose22, true);

        driveToGoal22 = buildDrivePath(follower, pickupPose22, controlPoint2, true);

        // The paths for the third set of artifacts(GPP) are below
        driveToPrePickup21 = buildDrivePath(follower, scorePose, prePickupPose21, true);

        driveToPickup21 = buildDrivePath(follower, prePickupPose21, pickupPose21, true);

        driveToGoal21 = buildDrivePath(follower, pickupPose21, finalShootPose, true);

//        driveToGate = follower.pathBuilder()
//                .addPath(new BezierLine(pickupPose23, preGateHit))
//                .setLinearHeadingInterpolation(pickupPose23.getHeading(), preGateHit.getHeading())
//                .addPath(new BezierLine(preGateHit, gateHit))
//                .setLinearHeadingInterpolation(preGateHit.getHeading(), gateHit.getHeading())
//                .setTimeoutConstraint(0)
//                .build();
        driveToGate = buildDrivePath(follower, pickupPose23, preGateHit, true);
        driveToGate = buildDrivePath(follower, preGateHit, gateHit, true);

        driveGateToGoal = buildDrivePath(follower, gateHit, scorePose, true);

        driveToGoal = buildDrivePath(follower, scorePose, endPose, true);

    }

    public PathChain buildDrivePath(
            Follower follower,
            Pose beginPose,
            Pose endPose,
            boolean isCurve
    ) {
        PathChain path = follower.pathBuilder()
                .addPath(isCurve
                        ? new BezierCurve(beginPose, endPose)
                        : new BezierLine(beginPose, endPose)
                )
                .setLinearHeadingInterpolation(
                        beginPose.getHeading(),
                        endPose.getHeading()
                )
                .setTimeoutConstraint(0)
                .build();
        return path;
    }


    private void updateAutonomousPathWhileLooping() {
        switch (pathState) {
            case 10:
                obeliskResult = 23;
                shooter.closeServoStop();
                shooter.downServoPaddle();
                follower.followPath(driveToGoal);
                setPathState(11);
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds() > START_DELAY_TIME) {
                    setPathState(1000);
                }
                break;
            case 1000:
                if (shootingSequenceFlag == 232221) {
                    shootingSequenceFlag = 777;
                } else if (shootingSequenceFlag == 2322) {
                    shootingSequenceFlag = 232221;
                } else if (shootingSequenceFlag == 23) {
                    shootingSequenceFlag = 2322;
                } else if (shootingSequenceFlag == 1) {
                    shootingSequenceFlag = 23;
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
                if (pathTimer.getElapsedTimeSeconds() > 0.25) { // changed from 0.5 to 0.25
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
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
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
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
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
                if (pathTimer.getElapsedTimeSeconds() > 0.8) {
                    //motorTransfer.setPower(transferOff);
                    shooter.downServoPaddle();
                    shooter.closeServoStop();
                    motorIntake.setPower(intakeOff);
                    setPathState(10008);
                }
                break;
            case 10008:
                setPathState(10009);
                break;
            case 10009: // updates shooting sequence flag
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    setPathState(10010);
                }
                break;
            case 10010:  // sends to next driving path
                if (shootingSequenceFlag == 21) {
                    setPathState(210);
                } else if (shootingSequenceFlag == 22) {
                    setPathState(220);
                } else if (shootingSequenceFlag == 23) {
                    setPathState(230);
                } else if (shootingSequenceFlag == 2123) {
                    setPathState(230);
                } else if (shootingSequenceFlag == 2223) {
                    setPathState(230);
                } else if (shootingSequenceFlag == 2322) {
                    setPathState(220);
                } else if (shootingSequenceFlag == 212322) {
                    setPathState(220);
                } else if (shootingSequenceFlag == 222321) {
                    setPathState(210);
                } else if (shootingSequenceFlag == 232221) {
                    setPathState(210);
                } else if (shootingSequenceFlag == 777) {
                    setPathState(999);
                }
                //motorTransfer.setPower(transferOn);
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
//                    motorIntake.setPower(intakeOff);
                    if (!follower.isBusy()) {
                        follower.followPath(driveToGoal21);
                        setPathState(214);
                    }
                }
                break;
            case 214: // case for shooting
                if (!follower.isBusy()) {
                    setPathState(1000);
                }
                break;
            case 220: // beginning of set of actions for spike mark 22, gets ready to pickup
                if (!follower.isBusy()) {
                    follower.followPath(driveToPrePickup22);
                    setPathState(221);
                }
                break;
            case 221: // timer case
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    if (!follower.isBusy()) {
                        motorIntake.setPower(intakeOn);
                        setPathState(222);
                    }
                }
                break;
            case 222: // picks up second set of artifacts (22)
                if (!follower.isBusy()) {
                    follower.followPath((driveToPickup22));
                    setPathState(223);
                }
                break;
            case 223: // drives toward goal to score second set of artifacts (22)
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    motorIntake.setPower(intakeOff);
                    if (!follower.isBusy()) {
                        follower.followPath(driveToGoal22);
                        setPathState(224);
                    }
                }
                break;
            case 224: // case for shooting
                if (!follower.isBusy()) {
                    setPathState(1000);
                }
                break;
            case 230: // beginning of actions for spike mark 23, gets ready to pickup
                if (!follower.isBusy()) {
                    follower.followPath(driveToPrePickup23);
                    setPathState(231);
                }
                break;
            case 231:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    if (!follower.isBusy()) {
                        motorIntake.setPower(intakeOn);
                        setPathState(232);
                    }
                }
                break;
            case 232: // picks up third set of artifacts (23)
                if (!follower.isBusy()) {
                    follower.followPath((driveToPickup23));
                    setPathState(2322);
                }
                break;
            case 2322:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
//                    motorIntake.setPower(intakeOff);
                    if (!follower.isBusy()) {
                        follower.followPath(driveToGate);
                        setPathState(233);
                    }
                }
                break;
            case 233: // timer case
                if (!follower.isBusy()) {
                    follower.followPath(driveGateToGoal);
                    setPathState(234);
                }
                break;
            case 234: // case for shooting
                if (!follower.isBusy()) {
                    setPathState(1000);
                }
                break;
            case 777:
                if (!follower.isBusy()) {
                    follower.followPath(driveToEnd);
                    TARGET_AUTON_RPM = 0.0;
                    setPathState(999);
                }
                break;
            case 999: // last state, just stops and waits
                if(pathTimer.getElapsedTimeSeconds() > 1) {
                    TARGET_AUTON_RPM = 0.0;
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

        updateFollowerWhileLooping();

        updateAutonomousPathWhileLooping();

        resetRPMWhileLooping();

        if ( ! isCompetitionMode()) {
            updateDiverStationConsole();
        }
    }

    private boolean isCompetitionMode() {
        return COMPETITION_MODE;
    }

    private void updateFollowerWhileLooping() {
        follower.update();
    }

    private void resetRPMWhileLooping() {
        double targetRPM = TARGET_AUTON_RPM;
        double CPR = 28.;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;
        //TPS = targetRPM / 60. * CPR;
        TPS = TARGET_AUTON_RPM / 60. * CPR;
        motorFlywheel.setVelocity(TPS);
        flywheelRPM = motorFlywheel.getVelocity() * 60 / CPR;
    }

    private void updateDiverStationConsole() {
        telemetry.addData("Obelisk ID", obeliskResult); // telemetry for which motif was detected.
        telemetry.addData("Path State", pathState); // the current path the code is running
        telemetry.addData("X", follower.getPose().getX()); // x pos
        telemetry.addData("Y", follower.getPose().getY()); // y pos
        telemetry.addData("Heading", follower.getPose().getHeading()); // heading
        telemetry.addData("Path Timer",pathTimer.getElapsedTimeSeconds());
        telemetry.addData("OpMode Timer", opmodeTimer.getElapsedTimeSeconds());
        //telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Target RPM", TARGET_AUTON_RPM);
        telemetry.addData("Flywheel RPM", flywheelRPM);
        telemetry.addData("Shooting Sequence", shootingSequenceFlag);
        telemetry.update();
    }

    @Override
    public void init() {
        initShooter();

        initMotors();

        initTimers();

        initFollower();

        buildPaths();

//        follower.setStartingPose(startPose);

//        shooter.closeServoStop();
//        shooter.downServoPaddle();
//        shooter.centerServoTurret();
    }

    private void initFollower() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
    }

    private void initTimers() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
    }

    private void initMotors() {
        motorIntake = initIntakeMotor(hardwareMap);
        motorTransfer = initTransferMotor(hardwareMap);
        motorFlywheel = initFlyWheelMotor(hardwareMap);
        pidfModified = motorFlywheel.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    private void initShooter() {
        shooter = new Shooter();
        shooter.init(hardwareMap);
        shooter.closeServoStop();
        shooter.downServoPaddle();
        shooter.centerServoTurret();
    }

    private DcMotorEx initIntakeMotor(HardwareMap hwMap) {
        DcMotorEx motor = hwMap.get(DcMotorEx.class, "motorIntake");
        motor.setDirection(DcMotorEx.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    private DcMotorEx initTransferMotor(HardwareMap hwMap) {
        DcMotorEx motor = hwMap.get(DcMotorEx.class, "motorTransfer");
        motor.setDirection(DcMotorEx.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        return motor;
    }

    private DcMotorEx initFlyWheelMotor(HardwareMap hwMap) {
        double NEW_P = 100.;   // 10.
        double NEW_I = 1.;    // 3.
        double NEW_D = 20.;    // 0.
        double NEW_F = 3.5;    // 0.

        DcMotorEx motor = hwMap.get(DcMotorEx.class, "motorFlywheel");
        motor.setDirection(DcMotorEx.Direction.FORWARD);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F));
        return motor;
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