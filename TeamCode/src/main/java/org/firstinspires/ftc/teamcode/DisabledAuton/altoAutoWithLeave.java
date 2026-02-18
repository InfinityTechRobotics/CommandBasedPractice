package org.firstinspires.ftc.teamcode.DisabledAuton;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous
public class altoAutoWithLeave extends OpMode {

    private final Pose startPose = new Pose(80, 8, Math.toRadians(90));
    private final Pose leavePose = new Pose(92, 8, Math.toRadians(90));

    private Follower follower;


    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private Servo servoStop;

    DcMotor motorIntake, motorTransfer, motorFlywheel;

    Servo servoPaddleLeft;
    double flywheelVoltageMultiplier = 1.04;

    double targetRPM, targetVoltage, flywheelPower;


    double transferOn = 0.8;
    double transferOff = 0.;

    double intakeOn = 0.8;
    double intakeOff = 0.0;

    double TARGET_AUTON_RPM = 3000.;

    double SERVO_PADDLE_SHOOT_POS = 0.24;
    double SERVO_PADDLE_DOWN_POS = 0.4;

    double SERVO_STOP_OPEN_POS = 0.15;
    double SERVO_STOP_CLOSE_POS = 0.33;

    private VoltageSensor battery;
    double batteryVoltage;

    private PathChain driveToLeave;

    private void buildPaths() {

        driveToLeave = follower.pathBuilder()
                .addPath(new BezierLine(startPose, leavePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), leavePose.getHeading())
                .setTimeoutConstraint(0)
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
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
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
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
                    servoPaddleLeft.setPosition(SERVO_PADDLE_DOWN_POS);
                    servoStop.setPosition(SERVO_STOP_CLOSE_POS);
                    setPathState(10008);
                }
                break;
            case 10008:
                if (!follower.isBusy()) {
                    follower.followPath(driveToLeave);
                    setPathState(10009);
            }
                break;
            case 10009:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    setPathState(777);
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

        targetVoltage = (0.0022 * targetRPM + 3.2) * flywheelVoltageMultiplier;
        batteryVoltage = battery.getVoltage();
        flywheelPower = Math.round(100. * targetVoltage / batteryVoltage) / 100.;
        if (flywheelPower < 0.5) {
            flywheelPower = 0.5;
        }
        motorFlywheel.setPower(flywheelPower);

        autonomousPathUpdate();

        telemetry.addData("path state", pathState); // the current path the code is running
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

        motorFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorTransfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        servoStop = hardwareMap.get(Servo.class, "servoStop");
        servoPaddleLeft = hardwareMap.servo.get("servoPaddleLeft");

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        servoStop.setPosition(SERVO_STOP_CLOSE_POS);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }




    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }



}