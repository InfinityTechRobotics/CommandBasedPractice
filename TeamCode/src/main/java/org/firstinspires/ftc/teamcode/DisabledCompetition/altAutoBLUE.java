package org.firstinspires.ftc.teamcode.DisabledCompetition;


import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

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
public class altAutoBLUE extends OpMode {

    private final Pose startPose = new Pose(64, 8, Math.toRadians(90));
    private final Pose scorePose = new Pose(54, 8, Math.toRadians(90));


    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private Servo servoStop;

    DcMotor motorIntake, motorTransfer, motorFlywheel;

    Servo servoPaddleLeft;
    double flywheelVoltageMultiplier = 0.98;

    double targetRPM, targetVoltage, flywheelPower;


    double transferOn = 0.8;
    double transferOff = 0.;

    double intakeOn = 0.8;
    double intakeOff = 0.0;

    double TARGET_AUTON_RPM = 3200.;

    double SERVO_PADDLE_SHOOT_POS = 0.24;
    double SERVO_PADDLE_DOWN_POS = 0.45;

    double SERVO_STOP_OPEN_POS = 0.15;
    double SERVO_STOP_CLOSE_POS = 0.33;

    private VoltageSensor battery;
    double batteryVoltage;

    private PathChain driveToGoal;

    private void buildPaths() {

        driveToGoal = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .setTimeoutConstraint(0)
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                servoStop.setPosition(SERVO_STOP_CLOSE_POS);
                motorIntake.setPower(intakeOn);
                motorTransfer.setPower(transferOn);
                if(pathTimer.getElapsedTimeSeconds() > 8) {
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
                    follower.followPath(driveToGoal);
                    if (!follower.isBusy()) {
                        setPathState(10008);
                    }
                }
                break;
            case 10008:
                motorFlywheel.setPower(0.0);
                motorTransfer.setPower(transferOff);
                motorIntake.setPower(intakeOff);
                setPathState(10009);
                break;
        }
    }

    private void setPathState(int pState) {

        pathState = pState;
        pathTimer.resetTimer();

    }

    @Override
    public void loop() {




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