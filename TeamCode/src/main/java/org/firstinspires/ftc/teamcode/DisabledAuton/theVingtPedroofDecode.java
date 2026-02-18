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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Disabled
@Autonomous
    public class theVingtPedroofDecode extends OpMode {

    boolean targetFound = false;
    private Limelight3A limelight;

    private int obeliskResult = 0;

    private static final int DESIRED_TAG_ID_1 = 21;

    private static final int DESIRED_TAG_ID_2 = 22;

    private static final int DESIRED_TAG_ID_3 = 23;

    double OBELISK_DETECT_MAX_TIME = 3;
    double START_DELAY_TIME = 1;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(80, 8, Math.toRadians(90)); // Start Pose of our robot.
//    private final Pose viewPose = new Pose(80, 120, Math.toRadians(90)); // Pose to read the Obelisk.

    private final Pose scorePose = new Pose(90, 102, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private  final Pose endPose = new Pose(90, 70, Math.toRadians(0));

    private final Pose prePickupPose1 = new Pose(100, 91.75, Math.toRadians(0)); // Preparing to intake first set of artifacts.

    private final Pose pickup1Pose = new Pose(124, 91.75, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark(PPG).

    private final Pose prePickupPose2 = new Pose(104, 68, Math.toRadians(0)); // Preparing to intake second set of artifacts.

    private final Pose pickup2Pose = new Pose(128, 68, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark(PGP).

    private final Pose prePickupPose3 = new Pose(104, 44, Math.toRadians(0)); // Preparing to intake third set of artifacts.

    private final Pose pickup3Pose = new Pose(128, 44, Math.toRadians(0)); // Last (Third Set) of Artifacts from the Spike Mark(GPP).

    private final Pose controlPoint1 = new Pose(90, 0); // Control point - you get the idea - read the name

    private final Pose controlPoint2 = new Pose(80, 60);

    private PathChain driveToGoal, driveToPrePickup1, driveToPickup1, driveToGoal2, driveToPrePickup2, driveToPickup2, driveToGoal3, driveToPrePickup3, driveToPickup3, driveToGoal4, driveToEnd;

    private Servo servoStop;

    DcMotor motorIntake, motorTransfer, motorFlywheel;

    Servo servoPaddleLeft;
    double flywheelVoltageMultiplier = 1.04;

    double targetRPM, targetVoltage, flywheelPower;

//    double flyWheelSpeedHigh = 0.7;
//    double flyWheelSpeedLow = 0.65;
//    double flyWheelSpeedOff = 0;

    double transferOn = 0.8;
    double transferOff = 0.;

    double intakeOn = 0.8;
    double intakeOff = 0.0;

    double TARGET_AUTON_RPM = 2400.;

    double SERVO_PADDLE_SHOOT_POS = 0.24;
    double SERVO_PADDLE_DOWN_POS = 0.4;

    int shootingSequenceFlag = 1;
    int endShootingSequenceFlag = 0;

    double SERVO_STOP_OPEN_POS = 0.15;
    double SERVO_STOP_CLOSE_POS = 0.33;

    private VoltageSensor battery;
    double batteryVoltage;

    boolean autoRPM = true;

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
        driveToPrePickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, prePickupPose1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickupPose1.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(prePickupPose1, pickup1Pose))
                .setLinearHeadingInterpolation(prePickupPose1.getHeading(), pickup1Pose.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToGoal2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .setTimeoutConstraint(0)
                .build();
        // The paths for the second set of artifacts(PGP) are below
        driveToPrePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, prePickupPose2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickupPose2.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(prePickupPose2, pickup2Pose))
                .setLinearHeadingInterpolation(prePickupPose2.getHeading(), pickup2Pose.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToGoal3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, controlPoint2, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .setTimeoutConstraint(0)
                .build();
        // The paths for the third set of artifacts(GPP) are below
        driveToPrePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, prePickupPose3))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickupPose3.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(prePickupPose3, pickup3Pose))
                .setLinearHeadingInterpolation(prePickupPose3.getHeading(), pickup3Pose.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToGoal4 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup3Pose, controlPoint1, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
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
                        } else if (opmodeTimer.getElapsedTimeSeconds() > OBELISK_DETECT_MAX_TIME  && opmodeTimer.getElapsedTimeSeconds() > START_DELAY_TIME) {
                            obeliskResult = 22; // If we can't read the obelisk within 5 seconds, we'll assume it's pattern PGP
                            setPathState(10); // We go score our preloaded artifacts
                        } else {
                            setPathState(0); // Until five seconds are up, we keep checking the limelight reading
                        }
                    }
                }
                else if (opmodeTimer.getElapsedTimeSeconds() > OBELISK_DETECT_MAX_TIME  && opmodeTimer.getElapsedTimeSeconds() > START_DELAY_TIME) {
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
                motorIntake.setPower(intakeOn);
                follower.followPath(driveToGoal);
                motorTransfer.setPower(transferOn);
                    setPathState(10000001);
                break;
            case 10000001:
                if (!follower.isBusy()) {
                    setPathState(1000);
                }
                break;
            case 1000:
                motorTransfer.setPower(transferOff);
                if(pathTimer.getElapsedTimeSeconds() > 0.25) {
                    servoStop.setPosition(SERVO_STOP_OPEN_POS);
                }
                if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                    servoPaddleLeft.setPosition(SERVO_PADDLE_SHOOT_POS); // extends the flaps to complete the action
                    setPathState(10001);
                }
                break;
            case 10001:
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    servoPaddleLeft.setPosition(SERVO_PADDLE_DOWN_POS); // lowers the flaps while finishing the action
                    servoStop.setPosition(SERVO_STOP_CLOSE_POS);
                    setPathState(10002);
                }
                break;
            case 10002:
                if(pathTimer.getElapsedTimeSeconds() > 1) { // moves up the next artifact to be shot
                    motorTransfer.setPower(transferOn); // sets transfer power to complete the action
                    setPathState(10003);
                }
                break;
            case 10003:
                if(pathTimer.getElapsedTimeSeconds() > 1) { // stops the transfer after the prior action completed & shoots the next artifact
                    motorTransfer.setPower(transferOff);    // shuts down transfer to prevent a jam
                }
                if(pathTimer.getElapsedTimeSeconds() > 1.25) {
                    servoStop.setPosition(SERVO_STOP_OPEN_POS);
                }
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    servoPaddleLeft.setPosition(SERVO_PADDLE_SHOOT_POS); // extends the flaps to complete the action
                    setPathState(10004);
                }
                break;
            case 10004:
                if(pathTimer.getElapsedTimeSeconds() > 0.5) { // lowers the flaps while finishing the action
                    servoPaddleLeft.setPosition(SERVO_PADDLE_DOWN_POS);
                    servoStop.setPosition(SERVO_STOP_CLOSE_POS);
                    setPathState(10005);
                }
                break;
            case 10005:
                if (pathTimer.getElapsedTimeSeconds() > 1) { // moves up the next artifact to be shot
                    motorTransfer.setPower(transferOn); // sets transfer power to complete the action
                    setPathState(10006);
                }
                break;
            case 10006:
                if(pathTimer.getElapsedTimeSeconds() > 2.6) { // stops the transfer after the prior action completed & shoots the next artifact
                    motorTransfer.setPower(transferOff);    // shuts down transfer to prevent a jam
                }
                if(pathTimer.getElapsedTimeSeconds() > 2.25) {
                    servoStop.setPosition(SERVO_STOP_OPEN_POS);
                }
                if(pathTimer.getElapsedTimeSeconds() > 2.5) {
                    servoPaddleLeft.setPosition(SERVO_PADDLE_SHOOT_POS); // extends the flaps to complete the action
                    setPathState(10007);
                }
                break;
            case 10007:
                if(pathTimer.getElapsedTimeSeconds() > 1.5 ) {  // lowers the flaps while finishing the action and shuts of transfer ending the sequence
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
                    if (obeliskResult == 23) {
                        shootingSequenceFlag = 2;
                    } else if (obeliskResult == 22) {
                        shootingSequenceFlag = 3;
                    }  else if (obeliskResult == 21) {
                        shootingSequenceFlag = 4;
                    }
                    setPathState(10010);
                }
                break;
            case 10010:  // sends to next driving path
                if (pathTimer.getElapsedTimeSeconds() > 0.1 ) {
                    if ((shootingSequenceFlag == 2) && (endShootingSequenceFlag == 0)) {
                        setPathState(20);
                        endShootingSequenceFlag = 1;
                    } else if ((shootingSequenceFlag == 3) && (endShootingSequenceFlag == 0)) {
                        setPathState(50);
                        endShootingSequenceFlag = 1;
                    } else if ((shootingSequenceFlag == 4) && (endShootingSequenceFlag == 0)) {
                        setPathState(80);
                        endShootingSequenceFlag = 1;
                    } else if (endShootingSequenceFlag != 0) {
                        setPathState(777);
                    }
                motorTransfer.setPower(transferOn);
                }
                break;
            case 20: // beginning of first set of actions, gets ready to pickup
                if(!follower.isBusy()) {
                    follower.followPath(driveToPrePickup1);
                    setPathState(21);
                }
                break;
            case 21:
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    if(!follower.isBusy()) {
                        setPathState(30);
                    }
                }
            case 30: // picks up first set of artifacts
                if(!follower.isBusy()) {
                    follower.followPath((driveToPickup1));
                    setPathState(31);
                }
                break;
            case 31: // timer case
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    if(!follower.isBusy()) {
                        follower.followPath(driveToGoal2);
                        setPathState(40);
                    }
                }
                break;
            case 40: // case for shooting
                if (!follower.isBusy()) {
                    setPathState(1000);
                }
                break;
            case 50: // beginning of second set of actions, gets ready to pickup
                if(!follower.isBusy()) {
                    follower.followPath(driveToPrePickup2);
                    setPathState(51);
                }
                break;
            case 51: // timer case
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    if(!follower.isBusy()) {
                        setPathState(60);
                    }
                }
                break;
            case 60: // picks up second set of artifacts
                if(!follower.isBusy()) {
                    follower.followPath((driveToPickup2));
                    setPathState(61);
                }
                break;
            case 61: // drives toward goal to score second set of artifacts
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    if(!follower.isBusy()) {
                        follower.followPath(driveToGoal3);
                        setPathState(70);
                    }
                }
                break;
            case 70: // case for shooting
                if (!follower.isBusy()) {
                    setPathState(1000);
                }
                break;
            case 80: // beginning of second set of actions, gets ready to pickup
                if(!follower.isBusy()) {
                    follower.followPath(driveToPrePickup3);
                    setPathState(81);
                }
                break;
            case 81: // timer case
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    if(!follower.isBusy()) {
                        setPathState(90);
                    }
                }
                break;
            case 90: // picks up third set of artifacts
                if(!follower.isBusy()) {
                    follower.followPath(driveToPickup3);
                    setPathState(91);
                }
                break;
            case 91: // drives toward goal to score second set of artifacts
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    if(!follower.isBusy()) {
                        follower.followPath(driveToGoal4);
                        setPathState(100);
                    }
                }
                break;
            case 100: // case for shooting
                if (!follower.isBusy()) {
                    setPathState(1000);
                }
                break;
            case 777:
                if (!follower.isBusy()) {
                    follower.followPath(driveToEnd);
                    motorFlywheel.setPower(0.);
                    motorTransfer.setPower(transferOff);
                    motorIntake.setPower(intakeOff);
                    setPathState(102);
                }
                break;
            case 102: // last state, just stops and waits
                if(pathTimer.getElapsedTimeSeconds() > 2) {
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

        telemetry.addData("Obelisk ID", obeliskResult); // telemetry for which motif was detected.
        telemetry.addData("path state", pathState); // the current path the code is running
        telemetry.addData("x", follower.getPose().getX()); // x pos
        telemetry.addData("y", follower.getPose().getY()); // y pos
        telemetry.addData("heading", follower.getPose().getHeading()); // heading
        telemetry.addData("Path Timer",pathTimer.getElapsedTimeSeconds());
        telemetry.addData("OpMode Timer", opmodeTimer.getElapsedTimeSeconds());
        telemetry.addData("Voltage", battery.getVoltage());
        telemetry.addData("Target Voltage", targetVoltage);
        telemetry.addData("Flywheel Motor Power", motorFlywheel.getPower());
        telemetry.addData("Target RPM", targetRPM);
        telemetry.update();

    }

    @Override
    public void init() {

        battery = hardwareMap.get(VoltageSensor.class, "Control Hub");

        motorIntake = hardwareMap.dcMotor.get("motorIntake");
        motorTransfer = hardwareMap.dcMotor.get("motorTransfer");
        motorFlywheel = hardwareMap.dcMotor.get("motorFlywheel");

        motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        motorTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        servoStop = hardwareMap.get(Servo.class, "servoStop");
        servoPaddleLeft = hardwareMap.servo.get("servoPaddleLeft");

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

        targetRPM = TARGET_AUTON_RPM;

        targetVoltage = (0.0022 * targetRPM + 3.2) * flywheelVoltageMultiplier;
        batteryVoltage = battery.getVoltage();
        flywheelPower = Math.round(100. * targetVoltage / batteryVoltage) / 100.;
        if (flywheelPower < 0.5) {
            flywheelPower = 0.5;
        }
        motorFlywheel.setPower(flywheelPower);

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