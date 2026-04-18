    package org.firstinspires.ftc.teamcode.DisabledAuton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Hardware.ShooterSpinfinityDuo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

    @Disabled
    @Autonomous
        public class redAlt extends OpMode {

        private final Pose startPose = new Pose(80, 8, Math.toRadians(90));
        private final Pose loadingZone = new Pose(133, 15, Math.toRadians(0));
        private final Pose reLoadingZone = new Pose(113, 15, Math.toRadians(0));
        private final Pose shhooting = new Pose(84, 16, Math.toRadians(65));
        ShooterSpinfinityDuo shooter = new ShooterSpinfinityDuo();

        private int obeliskResult = 0;

        double START_DELAY_TIME = 2.;

        private Follower follower;
        private Timer pathTimer, opmodeTimer, shootTimer;
        private int pathState;


        public static double NEW_P = 150.0;   // 10.0
        public static double NEW_I = 5.0;    // 3.0
        public static double NEW_D = 40.0;    // 0.0
        public static double NEW_F = 1.25;    // 0.0

        double shootingTime = 0.0;


        DcMotorEx motorIntake;
        DcMotorEx motorFlywheel;
        DcMotorEx motorFlywheel2;

        double targetRPM = 0.0;
        double flywheelRPM = 0.0;
        double flywheelRPM2 = 0.0;
        double TPS;
        double CPR = 28.0;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;

        double transferOn = 0.8;
        double transferOff = 0.0;

        double intakeOn = 0.8;
        double intakeOff = 0.0;

        double TARGET_AUTON_RPM = 3150; //2475.0

        int shootingSequenceFlag = 1;

        PIDFCoefficients pidfModified;

        int telemtryUpdate = 0;

        private PathChain driveToLoadingZone, driveToScorePoseL, driveToScorePoseS, driveAwayToLoadingZone, driveToAwayReLoadingZone;

        private void buildPaths() {

            driveToLoadingZone = follower.pathBuilder()
                    .addPath(new BezierLine(shhooting, reLoadingZone))
                    .setLinearHeadingInterpolation(shhooting.getHeading(), reLoadingZone.getHeading())
                    .addPath(new BezierLine(reLoadingZone, loadingZone))
                    .setTimeoutConstraint(0)
                    .build();

            driveAwayToLoadingZone = follower.pathBuilder()
                    .addPath(new BezierLine(loadingZone, reLoadingZone))
                    .setLinearHeadingInterpolation(loadingZone.getHeading(), reLoadingZone.getHeading())
                    .setTimeoutConstraint(0)
                    .build();

            driveToAwayReLoadingZone = follower.pathBuilder()
                    .addPath(new BezierLine(reLoadingZone, loadingZone))
                    .setLinearHeadingInterpolation(reLoadingZone.getHeading(), loadingZone.getHeading())
                    .setTimeoutConstraint(0)
                    .build();

            driveToScorePoseL = follower.pathBuilder()
                    .addPath(new BezierLine(loadingZone, shhooting))
                    .setLinearHeadingInterpolation(loadingZone.getHeading(), shhooting.getHeading())
                    .setTimeoutConstraint(0)
                    .build();

            driveToScorePoseS = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, shhooting))
                    .setLinearHeadingInterpolation(startPose.getHeading(), shhooting.getHeading())
                    .setTimeoutConstraint(0)
                    .build();



        }

        public void autonomousPathUpdate() {

            switch (pathState) {
                case 10:
                    shooter.closeServoStop();
                    shooter.downServoPaddle();
                    motorIntake.setPower(intakeOn);
                    if (pathTimer.getElapsedTimeSeconds() > 3) {
                        follower.followPath(driveToScorePoseS);
                        setPathState(11);
                    }
                    break;
                case 11:
                    if (!follower.isBusy()) {
                        setPathState(1000);
                    }
                    break;
                case 1000:
                    if (shootingSequenceFlag == 22102321) {
                        shootingSequenceFlag = 777;
                    } else if (shootingSequenceFlag == 221023) {
                        shootingSequenceFlag = 22102321;
                    } else if (shootingSequenceFlag == 2210) {
                        shootingSequenceFlag = 221023;
                    } else if (shootingSequenceFlag == 1) {
                        shootingSequenceFlag = 22;
                    } else if (shootingSequenceFlag == 22) {
                        shootingSequenceFlag = 2210;
                    }
                    setPathState(1001);
                    break;
                case 1001:
                    if (!follower.isBusy()) {
                        motorIntake.setPower(intakeOn);

                        setPathState(10001);
                    }
                    break;
                case 10001:
                    if (pathTimer.getElapsedTimeSeconds() > 0.25) { // changed from 0.5 to 0.25
                        shootTimer.resetTimer();
                        shooter.openServoStop();
                        setPathState(10006);
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
                    if (pathTimer.getElapsedTimeSeconds() > 0.6) {
                        shooter.shootServoPaddle();
                        setPathState(10007);
                    }
                    break;
                case 10007:
                    if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                        //motorTransfer.setPower(transferOff);
                        shootingTime = shootTimer.getElapsedTimeSeconds();
                        shooter.downServoPaddle();
                        shooter.closeServoStop();
                        motorIntake.setPower(intakeOff);
                        setPathState(10008);
                    }
                    break;
                case 10008: // updates shooting sequence flag
                    if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                        setPathState(667);
                    }
                    break;
                case 667:
                    if (!follower.isBusy()) {
                        setPathState(10010);
                    }
                    break;
                case 10010:  // sends to next driving path
                    if (shootingSequenceFlag == 22) {
                        setPathState(220);
                    } else if (shootingSequenceFlag == 2210) {
                        setPathState(777);
                    } else if (shootingSequenceFlag == 221023) {
                        setPathState(777);
                    } else if (shootingSequenceFlag == 22102321) {
                        setPathState(210);
                    } else if (shootingSequenceFlag == 777) {
                        setPathState(999);
                    }
                    //motorTransfer.setPower(transferOn);
                    break;
                case 210: // beginning of set of actions for spike mark 21, gets ready to pickup
                    motorIntake.setPower(intakeOn);
                    if (!follower.isBusy()) {
//                        follower.followPath(driveToPickup21);
                        setPathState(211);
                    }
                    break;
                case 211: // drives toward goal to score first set of artifacts (21)
                    if (!follower.isBusy()) {
//                            follower.followPath(driveToGoal21);
                        setPathState(214);
                    }
                    break;
                case 214: // case for shooting
                    if (!follower.isBusy()) {
                        setPathState(1000);
                    }
                    break;
                case 220: // beginning of set of actions for spike mark 22, gets ready to pickup
                    motorIntake.setPower(intakeOn);
                    if (!follower.isBusy()) {
                        follower.followPath(driveToLoadingZone);
                        setPathState(0221);
                    }
                    break;
                case 0221:
                    if (!follower.isBusy()) {
                        follower.followPath(driveAwayToLoadingZone);
                        setPathState(0222);
                    }
                    break;
                case 0222:
                    if (!follower.isBusy()) {
                        follower.followPath(driveToAwayReLoadingZone);
                        setPathState(221);
                    }
                case 221: // drives toward goal to score second set of artifacts (22)
                    if (!follower.isBusy()) {
                        follower.followPath(driveToScorePoseL);
                        setPathState(11);
                    }
                    break;
                case 224: // case for shooting
                    if (!follower.isBusy()) {
                        setPathState(1000);
                    }
                    break;
                case 230: // beginning of actions for spike mark 23, gets ready to pickup
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        if (!follower.isBusy()) {
                            motorIntake.setPower(intakeOn);
                            setPathState(231);
                        }
                    }
                    break;
                case 231:
                    if (!follower.isBusy()) {
//                        follower.followPath(driveToPickup23);
                        setPathState(234);
                    }
                    break;
                case 234:
                    if (!follower.isBusy()) {
//                        follower.followPath(driveToGoal23);
                        setPathState(235);
                    }
                    break;
                case 235: // case for shooting
                    if (!follower.isBusy()) {
                        setPathState(1000);
                    }
                    break;
                case 300: // case for "gobbling" from the gate
                    if (!follower.isBusy()) {
                        follower.followPath(driveToScorePoseL);
                        motorIntake.setPower(intakeOn);
                        setPathState(11);
                    }
                    break;
                case 301:
                    if (!follower.isBusy()) {
//                        follower.followPath(driveDownFromGate);
                        setPathState(302);
                    }
                    break;
                case 302:
                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {
//                        follower.followPath(driveToAwayFromGate);
                        setPathState(303);
                    }
                    break;
                case 303:
                    if (!follower.isBusy()) {
                        setPathState(1000);
                    }
                    break;
                case 777:
                    if (!follower.isBusy()) {
//                        follower.followPath(driveToEnd);
                        TARGET_AUTON_RPM = 0.0;
                        setPathState(999);
                    }
                    break;
                case 999: // last state, just stops and waits
                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        TARGET_AUTON_RPM = 0.0;
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
            motorFlywheel2.setVelocity(TPS);
            flywheelRPM = motorFlywheel.getVelocity() * 60 / CPR;
            flywheelRPM2 = motorFlywheel2.getVelocity() * 60 / CPR;

            telemetry.addData("Obelisk ID", obeliskResult); // telemetry for which motif was detected.
            telemetry.addData("Path State", pathState); // the current path the code is running
            telemetry.addData("X", follower.getPose().getX()); // x pos
            telemetry.addData("Y", follower.getPose().getY()); // y pos
            telemetry.addData("Heading", follower.getPose().getHeading()); // heading
            telemetry.addData("Path Timer",pathTimer.getElapsedTimeSeconds());
            telemetry.addData("OpMode Timer", opmodeTimer.getElapsedTimeSeconds());
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Flywheel RPM", flywheelRPM);
            telemetry.addData("Flywheel RPM2", flywheelRPM2);
            telemetry.addData("Shooting Sequence", shootingSequenceFlag);
            telemetry.addData("Intake Status", motorIntake.getPower());

            if (telemtryUpdate == 49){
                telemtryUpdate = 0;
                telemetry.update();
            } else {
                telemtryUpdate +=1;
            }



        }

        @Override
        public void init() {

            shooter.init(hardwareMap);

            motorIntake = hardwareMap.get(DcMotorEx.class, "motorIntake");
            motorFlywheel = hardwareMap.get(DcMotorEx.class, "motorFlywheel");
            motorFlywheel2 = hardwareMap.get(DcMotorEx.class, "motorFlywheel2");

            motorIntake.setDirection(DcMotorEx.Direction.REVERSE);
            motorFlywheel.setDirection(DcMotorEx.Direction.REVERSE);
            motorFlywheel2.setDirection(DcMotorEx.Direction.FORWARD);


            motorFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            motorIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motorFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

            PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
            motorFlywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

            pidfModified = motorFlywheel.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);

            pathTimer = new Timer();
            opmodeTimer = new Timer();
            opmodeTimer.resetTimer();

            shootTimer = new Timer();


            follower = Constants.createFollower(hardwareMap);
            buildPaths();
            follower.setStartingPose(startPose);

            shooter.closeServoStop();
            shooter.downServoPaddle();

            shooter.centerMotorTurret();

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