package org.firstinspires.ftc.teamcode.shadowday;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class servoPractice extends OpMode {

    private Servo servo;

    double servoPos;

    @Override
    public void init() { // runs once

        servo = hardwareMap.get(Servo.class, "servoTest");


    }


    public void loop() {

        if (gamepad1.xWasPressed()) {
           servoPos = 0;
        } else if (gamepad1.bWasPressed()) {
            servoPos = 1;
        } else if (gamepad1.yWasPressed()) {
            servoPos += 0.01;
        } else if (gamepad1.aWasPressed()) {
            servoPos -= 0.01;
        }

        servo.setPosition(servoPos);

        telemetry.addData("Motor Power is:", servoPos);

        telemetry.update();

    }

}
