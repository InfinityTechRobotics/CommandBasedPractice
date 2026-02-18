package org.firstinspires.ftc.teamcode.DisabledAuton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Disabled
@Autonomous
    public class autoBlehTwoTeenthFour extends OpMode {

    boolean targetFound = false;
    private Limelight3A limelight;

    private int obeliskResult = 0;

    private static final int DESIRED_TAG_ID_1 = 21;

    private static final int DESIRED_TAG_ID_2 = 22;

    private static final int DESIRED_TAG_ID_3 = 23;

    double OBELISK_DETECT_MAX_TIME = 3;
    double START_DELAY_TIME = 2;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    public static double NEW_P = 100.;   // 10.
    public static double NEW_I = 1.;    // 3.
    public static double NEW_D = 20.;    // 0.
    public static double NEW_F = 3.5;    // 0.

    private final Pose startPose = new Pose(64, 8, Math.toRadians(90)); // Start Pose of our robot.
//    private final Pose viewPose = new Pose(80, 120, Math.toRadians(90)); // Pose to read the Obelisk.

    private final Pose scorePose = new Pose(54, 98, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private final Pose prePickupPose21 = new Pose(50, 34, Math.toRadians(180)); // Preparing to intake third set of artifacts.

    private final Pose pickupPose21 = new Pose(22, 34, Math.toRadians(180)); // Last (Third Set) of Artifacts from the Spike Mark(GPP).

    private final Pose prePickupPose22 = new Pose(50, 58, Math.toRadians(180)); // Preparing to intake second set of artifacts.

    private final Pose pickupPose22 = new Pose(22, 58, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark(PGP).

    private final Pose prePickupPose23 = new Pose(50, 84, Math.toRadians(180)); // Preparing to intake first set of artifacts.

    private final Pose pickupPose23 = new Pose(26, 84, Math.toRadians(180));// Highest (First Set) of Artifacts from the Spike Mark(PPG).

    private  final Pose endPose = new Pose(54, 70, Math.toRadians(180));//its the end - if you took the time to read this, you get it - otherwise vid is disappointed :(

    private final Pose controlPoint1 = new Pose(90, 0); // Control point - you get the idea - read the name

    private final Pose controlPoint2 = new Pose(64, 60);

    private PathChain driveToGoal, driveToPrePickup23, driveToPickup23, driveToGoal23, driveToPrePickup22, driveToPickup22, driveToGoal22, driveToPrePickup21, driveToPickup21, driveToGoal21, driveToEnd;

    private Servo servoStop;

    DcMotor motorIntake, motorTransfer;
    DcMotorEx motorFlywheel;

    Servo servoPaddleLeft;
    Servo servoTurret;


    double targetRPM = 0.;
    double flywheelRPM = 0.;
    double TPS;
    double CPR = 28.;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;


//    double flyWheelSpeedHigh = 0.7;
//    double flyWheelSpeedLow = 0.65;
//    double flyWheelSpeedOff = 0;

    double transferOn = 0.8;
    double transferOff = 0.;

    double intakeOn = 0.8;
    double intakeOff = 0.0;

    double TARGET_AUTON_RPM = 2250.;

    double SERVO_PADDLE_SHOOT_POS = 0.75;
    double SERVO_PADDLE_DOWN_POS = 0.3;

    int shootingSequenceFlag = 1;

    double SERVO_STOP_OPEN_POS = 0.15;
    double SERVO_STOP_CLOSE_POS = 0.33;


    private void buildPaths() {
        // The beginning paths(regardless of motif pattern) are below
//        driveToObelisk = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, viewPose))
//                .setConstantHeadingInterpolation(startPose.getHeading())
//                .setTimeoutConstraint(0)
//                .build();

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
                .addPath(new BezierCurve(pickupPose21, controlPoint1, scorePose))
                .setLinearHeadingInterpolation(pickupPose21.getHeading(), scorePose.getHeading())
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
            case 0: //Reads Obelisk
                LLResult result = limelight.getLatestResult(); // takes limelight reading
                if (result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults(); // AprilTag numbers are fiducial markers
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        obeliskResult = fr.getFiducialId(); // Sets obeliskResult to 21, 22, or 23 depending on the AprilTag
                        if (obeliskResult == DESIRED_TAG_ID_1 || obeliskResult == DESIRED_TAG_ID_2 || obeliskResult == DESIRED_TAG_ID_3 && opmodeTimer.getElapsedTimeSeconds() > START_DELAY_TIME) {
                            setPathState(10); // No matter what motif it is, we go score our preloaded artifacts
                        } else if ((opmodeTimer.getElapsedTimeSeconds() > OBELISK_DETECT_MAX_TIME)) {
                            obeliskResult = 22; // If we can't read the obelisk within 5 seconds, we'll assume it's pattern PGP
                            setPathState(10); // We go score our preloaded artifacts
                        } else {
                            setPathState(0); // Until five seconds are up, we keep checking the limelight reading
                        }
                    }
                }
                else if (opmodeTimer.getElapsedTimeSeconds() > OBELISK_DETECT_MAX_TIME) {
                    obeliskResult = 22; // Assumes PGP if it doesn't detect within five seconds
                    setPathState(10); // Goes to score preloaded artifacts
                }
                else {
                    obeliskResult = 0; // We don't set an obelisk result until five seconds are up
                    setPathState(0); // Keeps checking limelight reading
                }
                break;
            case 10: // drives toward goal to score preloaded artifacts
                servoStop.setPosition(SERVO_STOP_CLOSE_POS);
                servoPaddleLeft.setPosition(SERVO_PADDLE_DOWN_POS);
                follower.followPath(driveToGoal);
                setPathState(1000);
                break;
            case 1000:
                if (shootingSequenceFlag == 2123) {
                    shootingSequenceFlag = 212322;
                } else if (shootingSequenceFlag == 2223) {
                    shootingSequenceFlag = 222321;
                } else if (shootingSequenceFlag == 2322) {
                    shootingSequenceFlag = 232221;
                } else if (shootingSequenceFlag == 21) {
                    shootingSequenceFlag = 2123;
                } else if (shootingSequenceFlag == 22) {
                    shootingSequenceFlag = 2223;
                } else if (shootingSequenceFlag == 23) {
                    shootingSequenceFlag = 2322;
                }
                if (shootingSequenceFlag == 1) {
                    if (obeliskResult == 21) {
                        shootingSequenceFlag = 21;
                    } else if (obeliskResult == 22) {
                        shootingSequenceFlag = 22;
                    } else if (obeliskResult == 23) {
                        shootingSequenceFlag = 23;
                    }
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
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    servoStop.setPosition(SERVO_STOP_OPEN_POS);
                    setPathState(10002);
                }
                break;
            case 10002:
                if (pathTimer.getElapsedTimeSeconds() > 0.25 ) {
                    servoStop.setPosition(SERVO_STOP_CLOSE_POS);
                    setPathState(10003);
                }
                break;
            case 10003:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    servoStop.setPosition(SERVO_STOP_OPEN_POS);
                    setPathState(10004);
                }
                break;
            case 10004:
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                    servoStop.setPosition(SERVO_STOP_CLOSE_POS);
                    setPathState(10005);
                }
                break;
            case 10005:
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                    servoStop.setPosition(SERVO_STOP_OPEN_POS);
                    setPathState(10006);
                }
                break;
            case 10006:
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                    servoPaddleLeft.setPosition(SERVO_PADDLE_SHOOT_POS);
                    setPathState(10007);
                }
                break;
            case 10007:
                if (pathTimer.getElapsedTimeSeconds() > 0.75) {
                    //motorTransfer.setPower(transferOff);
                    servoPaddleLeft.setPosition(SERVO_PADDLE_DOWN_POS);
                    servoStop.setPosition(SERVO_STOP_CLOSE_POS);
                    setPathState(10008);
                }
                break;
            case 10008:
                setPathState(10009);
                break;
            case 10009: // updates shooting sequence flag
                if (pathTimer.getElapsedTimeSeconds() > 0.1 ) {
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
                } else {
                    setPathState(777);
                }
                //motorTransfer.setPower(transferOn);
                break;
            case 210: // beginning of set of actions for spike mark 21, gets ready to pickup
                if(!follower.isBusy()) {
                    follower.followPath(driveToPrePickup21);
                    setPathState(211);
                }
                break;
            case 211: // timer case
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    if(!follower.isBusy()) {
                        setPathState(212);
                    }
                }
                break;
            case 212: // picks up first set of artifacts (21)
                if(!follower.isBusy()) {
                    follower.followPath(driveToPickup21);
                    setPathState(213);
                }
                break;
            case 213: // drives toward goal to score first set of artifacts (21)
                if(pathTimer.getElapsedTimeSeconds() > 1.0) {
                    if(!follower.isBusy()) {
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
                if(!follower.isBusy()) {
                    follower.followPath(driveToPrePickup22);
                    setPathState(221);
                }
                break;
            case 221: // timer case
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    if(!follower.isBusy()) {
                        setPathState(222);
                    }
                }
                break;
            case 222: // picks up second set of artifacts (22)
                if(!follower.isBusy()) {
                    follower.followPath((driveToPickup22));
                    setPathState(223);
                }
                break;
            case 223: // drives toward goal to score second set of artifacts (22)
                if(pathTimer.getElapsedTimeSeconds() > 1.0) {
                    if(!follower.isBusy()) {
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
                if(!follower.isBusy()) {
                    follower.followPath(driveToPrePickup23);
                    setPathState(231);
                }
                break;
            case 231:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    if(!follower.isBusy()) {
                        setPathState(232);
                    }
                }
            case 232: // picks up third set of artifacts (23)
                if(!follower.isBusy()) {
                    follower.followPath((driveToPickup23));
                    setPathState(233);
                }
                break;
            case 233: // timer case
                if(pathTimer.getElapsedTimeSeconds() > 1.0) {
                    if(!follower.isBusy()) {
                        follower.followPath(driveToGoal23);
                        setPathState(234);
                    }
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
                    TARGET_AUTON_RPM = 0.;
                    setPathState(999);
                }
                break;
            case 999: // last state, just stops and waits
                if(pathTimer.getElapsedTimeSeconds() > 1) {
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

        //battery = hardwareMap.get(VoltageSensor.class, "Control Hub");

        motorIntake = hardwareMap.dcMotor.get("motorIntake");
        motorTransfer = hardwareMap.dcMotor.get("motorTransfer");
        motorFlywheel = hardwareMap.get(DcMotorEx.class, "motorFlywheel");

        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        motorTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        motorFlywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);


        servoStop = hardwareMap.get(Servo.class, "servoStop");
        servoPaddleLeft = hardwareMap.servo.get("servoPaddleLeft");
        servoTurret = hardwareMap.get(Servo.class, "servoWebcam");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.update();

        limelight.pipelineSwitch(0);

        limelight.start();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        servoStop.setPosition(SERVO_STOP_CLOSE_POS);
        servoPaddleLeft.setPosition(SERVO_PADDLE_DOWN_POS);

        servoTurret.setPosition(0.5);

    }

    @Override
    public void init_loop() {
        // Nothing
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }


    @Override
    public void stop() {
        limelight.stop();
    }

}