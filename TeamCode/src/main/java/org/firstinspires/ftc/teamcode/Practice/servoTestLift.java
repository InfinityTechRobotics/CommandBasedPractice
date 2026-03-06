package org.firstinspires.ftc.teamcode.Practice;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class servoTestLift extends OpMode {

    private Servo servoA;

    private Servo servoB;

    double servoPositionA = 0;
    double servoPositionB = 0;

    double servoPositionR = 0;
    double servoPositionE = 1;

    @Override
    public void init() { // runs once

        servoA = hardwareMap.get(Servo.class, "RPMIndicatorLeft");
        servoB = hardwareMap.get(Servo.class, "servoLiftB");

    }


    public void loop() {

        if(gamepad1.aWasPressed()) {

            servoA.setPosition(servoPositionR);
            servoB.setPosition(servoPositionR);

        } else if (gamepad1.yWasPressed()) {

            servoA.setPosition(servoPositionE);
            servoB.setPosition(servoPositionE);

        }


        if (gamepad1.xWasPressed()) {
           servoPositionA -= 0.1;
        } else if (gamepad1.bWasPressed()) {
            servoPositionA += 0.1 ;
        } else if (gamepad1.dpadLeftWasPressed()) {
            servoPositionB -= 0.1;
        } else if (gamepad1.dpadRightWasPressed()) {
            servoPositionB += 0.1;
        }

        if (gamepad1.rightBumperWasPressed()) {
            servoA.setPosition(servoPositionA);
            servoB.setPosition(servoPositionB);
        }

        telemetry.addData("servoA pos is:", servoA.getPosition());
        telemetry.addData("ServoB pos is:", servoB.getPosition());
        telemetry.update();

        // range is 0 - 1

    }

}
