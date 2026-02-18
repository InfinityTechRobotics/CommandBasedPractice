package org.firstinspires.ftc.teamcode.Disabled;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Autonomous(name = "shootingSequence", group = "pedroPathing")
    public class shootingSequence extends OpMode {


    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    DcMotor motorIntake, motorTransfer, motorFlywheel;

    Servo servoPaddleLeft, servoPaddleRight;

    double flyWheelSpeedHigh = 0.9;
    double flyWheelSpeedLow = 0.6;
    double flyWheelSpeedOff = 0;

    double transferOn = 0.8;
    double transferOff = 0.0;

    double intakeOn = 0.8;
    double intakeOff = 0.0;

    double flapLEngaged = 0.45;
    double flapLDisengaged = 0.65;


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // powers on flywheel and intake
                motorFlywheel.setPower(flyWheelSpeedHigh);
                motorIntake.setPower(intakeOn);
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setPathState(40);
                }
                 break;
            case 40: // case for shooting
                    if(pathTimer.getElapsedTimeSeconds() > 0.25 && pathTimer.getElapsedTimeSeconds() < 1.0) { // pushes out first artifact to be shot
                        servoPaddleLeft.setPosition(flapLEngaged); // extends the flaps to complete the action
                        servoPaddleRight.setPosition(flapLEngaged);
                    }

                    if(pathTimer.getElapsedTimeSeconds() > 2 && pathTimer.getElapsedTimeSeconds() < 3) { // lowers the flaps while finishing the action
//                      servoPaddleLeft.setPosition(flapLDisengaged);
                        servoPaddleRight.setPosition(flapLDisengaged);
                    }
                    if(pathTimer.getElapsedTimeSeconds() > 4 && pathTimer.getElapsedTimeSeconds() < 5) { // moves up the next artifact to be shot
                        motorTransfer.setPower(transferOn); // sets transfer power to complete the action
                    }
                    if(pathTimer.getElapsedTimeSeconds() > 6 && pathTimer.getElapsedTimeSeconds() < 7) { // stops the transfer after the prior action completed & shoots the next artifact
                        motorTransfer.setPower(transferOff); // shuts down transfer to prevent a jam
//                    servoPaddleLeft.setPosition(flapLEngaged); // extends the flaps to complete the action
                        servoPaddleRight.setPosition(flapLEngaged);
                    }
                    if(pathTimer.getElapsedTimeSeconds() > 7 && pathTimer.getElapsedTimeSeconds() < 8) { // lowers the flaps while finishing the action
//                      servoPaddleLeft.setPosition(flapLDisengaged);
                        servoPaddleRight.setPosition(flapLDisengaged);
                    }
                    if(pathTimer.getElapsedTimeSeconds() > 8 && pathTimer.getElapsedTimeSeconds() < 9) { // moves up the next artifact to be shot
                        motorTransfer.setPower(transferOn); // sets transfer power to complete the action
                    }
                    if(pathTimer.getElapsedTimeSeconds() > 9 && pathTimer.getElapsedTimeSeconds() < 10) { // stops the transfer after the prior action completed & shoots the next artifact
                        motorTransfer.setPower(transferOff); // shuts down transfer to prevent a jam
//                    servoPaddleLeft.setPosition(flapLEngaged); // extends the flaps to complete the action
                        servoPaddleRight.setPosition(flapLEngaged);
                    }

                    if(pathTimer.getElapsedTimeSeconds() > 10 ) {  // lowers the flaps while finishing the action and shuts of transfer ending the sequence
    //                  servoPaddleLeft.setPosition(flapLDisengaged);
                        servoPaddleRight.setPosition(flapLDisengaged);
                        motorTransfer.setPower(transferOff);
                        setPathState(102);
                    }
                break;

            case 102: // last state, just stops and waits while shutting down
                if(pathTimer.getElapsedTimeSeconds() > 0.1) {
                    motorFlywheel.setPower(flyWheelSpeedOff);
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

        autonomousPathUpdate();

        telemetry.addData("path state", pathState); // the current path the code is running
        telemetry.addData("Path Timer",pathTimer.getElapsedTimeSeconds());
        telemetry.addData("OpMode Timer", opmodeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    @Override
    public void init() {

        motorIntake = hardwareMap.dcMotor.get("motorIntake");
        motorTransfer = hardwareMap.dcMotor.get("motorTransfer");
        motorFlywheel = hardwareMap.dcMotor.get("motorFlywheel");

        motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        motorTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        servoPaddleLeft = hardwareMap.servo.get("servoPaddleLeft");
        servoPaddleRight = hardwareMap.servo.get("servoPaddleRight");

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }


    @Override
    public void stop() {

    }

}