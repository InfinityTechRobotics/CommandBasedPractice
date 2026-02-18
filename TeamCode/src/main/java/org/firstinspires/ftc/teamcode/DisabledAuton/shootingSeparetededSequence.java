package org.firstinspires.ftc.teamcode.DisabledAuton;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Autonomous
    public class shootingSeparetededSequence extends OpMode {


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

    double flapLEngaged = 0.24;
    double flapLDisengaged = 0.37;

    int shootingSequenceFlag = 1;
    int endShootingSequenceFlag = 0;
    int obeliskResult = 0;

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // powers on flywheel and intake
                motorFlywheel.setPower(flyWheelSpeedHigh);
                motorIntake.setPower(intakeOn);
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setPathState(40);
                }
                 break;
            case 1000: // case for shooting
                    if(pathTimer.getElapsedTimeSeconds() > 1) { // pushes out first artifact to be shot
                        servoPaddleLeft.setPosition(flapLEngaged); // extends the flaps to complete the action
//                        servoPaddleRight.setPosition(flapLEngaged);
                        setPathState(1001);
                    }
                    break;
            case 1001:
                    if(pathTimer.getElapsedTimeSeconds() > 1) { // lowers the flaps while finishing the action
                      servoPaddleLeft.setPosition(flapLDisengaged);
//                        servoPaddleRight.setPosition(flapLDisengaged);
                        setPathState(1002);
                    }
                    break;
            case 1002:
                    if(pathTimer.getElapsedTimeSeconds() > 2) { // moves up the next artifact to be shot
                        motorTransfer.setPower(transferOn); // sets transfer power to complete the action
                        setPathState(1003);
                    }
                    break;
            case 1003:
                    if(pathTimer.getElapsedTimeSeconds() > 2) { // stops the transfer after the prior action completed & shoots the next artifact
                        motorTransfer.setPower(transferOff); // shuts down transfer to prevent a jam
                    servoPaddleLeft.setPosition(flapLEngaged); // extends the flaps to complete the action
//                        servoPaddleRight.setPosition(flapLEngaged);
                        setPathState(1004);
                    }
                    break;
            case 1004:
                    if(pathTimer.getElapsedTimeSeconds() > 1) { // lowers the flaps while finishing the action
                      servoPaddleLeft.setPosition(flapLDisengaged);
//                        servoPaddleRight.setPosition(flapLDisengaged);
                        setPathState(1005);
                    }
                    break;
            case 1005:
                    if (pathTimer.getElapsedTimeSeconds() > 1) { // moves up the next artifact to be shot
                        motorTransfer.setPower(transferOn); // sets transfer power to complete the action
                        setPathState(1006);
                    }
                    break;
            case 1006:
                    if(pathTimer.getElapsedTimeSeconds() > 1) { // stops the transfer after the prior action completed & shoots the next artifact
                        motorTransfer.setPower(transferOff); // shuts down transfer to prevent a jam
                    servoPaddleLeft.setPosition(flapLEngaged); // extends the flaps to complete the action
//                        servoPaddleRight.setPosition(flapLEngaged);
                        setPathState(1007);
                    }
                    break;
            case 1007:
                    if(pathTimer.getElapsedTimeSeconds() > 1 ) {  // lowers the flaps while finishing the action and shuts of transfer ending the sequence
                      servoPaddleLeft.setPosition(flapLDisengaged);
    //                    servoPaddleRight.setPosition(flapLDisengaged);
                        motorTransfer.setPower(transferOff);
                        setPathState(1008);
                    }
                break;

            case 1008: // last state, just stops and waits while shutting down
                if (obeliskResult == 23) {
                    shootingSequenceFlag = 2;
                    endShootingSequenceFlag = 1;
                } else if (obeliskResult == 22) {
                    shootingSequenceFlag = 3;
                    endShootingSequenceFlag = 2;
                }  else if (obeliskResult == 21) {
                    shootingSequenceFlag = 4;
                    endShootingSequenceFlag = 3;
                }

                if ((shootingSequenceFlag == 2) && !(endShootingSequenceFlag == 0)) {
                    setPathState(20);
                } else if ((shootingSequenceFlag == 3) && !(endShootingSequenceFlag == 0)) {
                    setPathState(50);
                } else if ((shootingSequenceFlag == 4) && !(endShootingSequenceFlag == 0)) {
                    setPathState(80);
                } else if (endShootingSequenceFlag != 0) {
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