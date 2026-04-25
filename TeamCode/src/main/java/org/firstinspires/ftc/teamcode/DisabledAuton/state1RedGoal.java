    package org.firstinspires.ftc.teamcode.DisabledAuton;

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
        public class state1RedGoal extends OpMode {

        Shooter shooter = new Shooter();

        private int obeliskResult = 0;

        double START_DELAY_TIME = 2.;

        private Follower follower;
        private Timer pathTimer, opmodeTimer;
        private int pathState;

        public static double NEW_P = 100.;   // 10.
        public static double NEW_I = 1.;    // 3.
        public static double NEW_D = 20.;    // 0.
        public static double NEW_F = 3.5;    // 0.

        private final Pose startPose = new Pose(116, 129, Math.toRadians(36)); // Start Pose of our robot.
    //    private final Pose viewPose = new Pose(80, 120, Math.toRadians(90)); // Pose to read the Obelisk.

        private final Pose motifDetection = new Pose(85, 110, Math.toRadians(90));

        private final Pose scorePose = new Pose(90, 102, Math.toRadians(37.5)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

        private final Pose prePickupPose21 = new Pose(92, 41, Math.toRadians(0)); // Preparing to intake third set of artifacts.

        private final Pose pickupPose21 = new Pose(129, 41, Math.toRadians(0)); // Last (Third Set) of Artifacts from the Spike Mark(GPP).

        private final Pose prePickupPose22 = new Pose(92, 63, Math.toRadians(0)); // Preparing to intake second set of artifacts.

        private final Pose pickupPose22 = new Pose(129, 63, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark(PGP).

        private final Pose prePickupPose23 = new Pose(92, 88.75, Math.toRadians(0)); // Preparing to intake first set of artifacts.

        private final Pose pickupPose23 = new Pose(125, 88.75, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark(PPG).

        private final Pose preGateHit = new Pose(118, 80, Math.toRadians(90));

        private final Pose gateHit = new Pose( 121, 73.5, Math.toRadians(90));

        private final Pose controlPoint1 = new Pose(90, 0); // Control point - you get the idea - read the name

        private final Pose controlPoint2 = new Pose(80, 60); // 67! ;) - you should get what this means by now - read the name aigin - idk i cant spel

        private  final Pose endPose = new Pose(90, 67, Math.toRadians(0));//its the end - if you took the time to read this, you get it - otherwise vid the auton eyelid is dissapointed :(

        private final Pose finalShootPose = new Pose (90, 110, Math.toRadians(30));

        private PathChain driveToGoal, driveToPrePickup23, driveToPickup23, driveToGoal23, driveToPrePickup22, driveToPickup22, driveToGoal22, driveToPrePickup21, driveToPickup21, driveToGoal21, driveToEnd, driveToGate, driveGateToGoal;

        DcMotorEx motorIntake, motorTransfer;
        DcMotorEx motorFlywheel;

        double targetRPM = 0.;
        double flywheelRPM = 0.;
        double TPS;
        double CPR = 28.;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;

        double transferOn = 0.8;
        double transferOff = 0.;

        double intakeOn = 0.8;
        double intakeOff = 0.0;

        double TARGET_AUTON_RPM = 2250.;

        int shootingSequenceFlag = 1;

        PIDFCoefficients pidfModified;


        private void buildPaths() {

            driveToGoal = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, scorePose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                    .setTimeoutConstraint(0)
                    .build();

            // The paths for the first set of artifacts(PPG) are below
            driveToPrePickup23 = follower.pathBuilder()
                    .addPath(new BezierCurve(scorePose, prePickupPose23))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), prePickupPose23.getHeading())
                    .setTimeoutConstraint(0)
                    .build();

            driveToPickup23 = follower.pathBuilder()
                    .addPath(new BezierCurve(prePickupPose23, pickupPose23))
                    .setLinearHeadingInterpolation(prePickupPose23.getHeading(), pickupPose23.getHeading())
                    .setTimeoutConstraint(0)
                    .build();

            driveToGoal23 = follower.pathBuilder()
                    .addPath(new BezierCurve(pickupPose23, scorePose))
                    .setLinearHeadingInterpolation(pickupPose23.getHeading(), scorePose.getHeading())
                    .setTimeoutConstraint(0)
                    .build();

            // The paths for the second set of artifacts(PGP) are below
            driveToPrePickup22 = follower.pathBuilder()
                    .addPath(new BezierCurve(scorePose, prePickupPose22))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), prePickupPose22.getHeading())
                    .setTimeoutConstraint(0)
                    .build();

            driveToPickup22 = follower.pathBuilder()
                    .addPath(new BezierCurve(prePickupPose22, pickupPose22))
                    .setLinearHeadingInterpolation(prePickupPose22.getHeading(), pickupPose22.getHeading())
                    .setTimeoutConstraint(0)
                    .build();

            driveToGoal22 = follower.pathBuilder()
                    .addPath(new BezierCurve(pickupPose22, controlPoint2, scorePose))
                    .setLinearHeadingInterpolation(pickupPose22.getHeading(), scorePose.getHeading())
                    .setTimeoutConstraint(0)
                    .build();
            // The paths for the third set of artifacts(GPP) are below
            driveToPrePickup21 = follower.pathBuilder()
                    .addPath(new BezierCurve(scorePose, prePickupPose21))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), prePickupPose21.getHeading())
                    .setTimeoutConstraint(0)
                    .build();

            driveToPickup21 = follower.pathBuilder()
                    .addPath(new BezierLine(prePickupPose21, pickupPose21))
                    .setLinearHeadingInterpolation(prePickupPose21.getHeading(), pickupPose21.getHeading())
                    .setTimeoutConstraint(0)
                    .build();

            driveToGoal21= follower.pathBuilder()
                    .addPath(new BezierCurve(pickupPose21, finalShootPose))
                    .setLinearHeadingInterpolation(pickupPose21.getHeading(), finalShootPose.getHeading())
                    .setTimeoutConstraint(0)
                    .build();

            driveToGate = follower.pathBuilder()
                    .addPath(new BezierLine(pickupPose23, preGateHit))
                    .setLinearHeadingInterpolation(pickupPose23.getHeading(), preGateHit.getHeading())
                    .addPath(new BezierLine(preGateHit, gateHit))
                    .setLinearHeadingInterpolation(preGateHit.getHeading(), gateHit.getHeading())
                    .setTimeoutConstraint(0)
                    .build();

            driveGateToGoal = follower.pathBuilder()
                    .addPath(new BezierLine(gateHit, scorePose))
                    .setLinearHeadingInterpolation(gateHit.getHeading(), scorePose.getHeading())
                    .setTimeoutConstraint(0)
                    .build();

            driveToEnd = follower.pathBuilder()
                    .addPath(new BezierCurve(scorePose, endPose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                    .setTimeoutConstraint(0)
                    .build();


        }

        public void autonomousPathUpdate() {
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

            follower.update();
            autonomousPathUpdate();

            targetRPM = TARGET_AUTON_RPM;
            TPS = targetRPM / 60. * CPR;
            motorFlywheel.setVelocity(TPS);
            flywheelRPM = motorFlywheel.getVelocity() * 60 / CPR;

            telemetry.addData("Obelisk ID", obeliskResult); // telemetry for which motif was detected.
            telemetry.addData("Path State", pathState); // the current path the code is running
            telemetry.addData("X", follower.getPose().getX()); // x pos
            telemetry.addData("Y", follower.getPose().getY()); // y pos
            telemetry.addData("Heading", follower.getPose().getHeading()); // heading
            telemetry.addData("Path Timer",pathTimer.getElapsedTimeSeconds());
            telemetry.addData("OpMode Timer", opmodeTimer.getElapsedTimeSeconds());
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Flywheel RPM", flywheelRPM);
            telemetry.addData("Shooting Sequence", shootingSequenceFlag);
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