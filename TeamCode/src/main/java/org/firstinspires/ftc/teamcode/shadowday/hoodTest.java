package org.firstinspires.ftc.teamcode.shadowday;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp

public class hoodTest extends OpMode {

    private Servo servo;

    double servoPosition = 0.5;

    @Override
    public void init() { // runs once

        servo = hardwareMap.get(Servo.class, "servoShooter");

        servo.setPosition(servoPosition);


    }


    public void loop() {



        if (gamepad1.xWasPressed()) {
           servoPosition -= 0.01;
        } else if (gamepad1.bWasPressed()) {
            servoPosition += 0.01 ;
        } else if (gamepad1.yWasPressed()) {
            servoPosition += 0.1;
        } else if (gamepad1.aWasPressed()) {
            servoPosition -= 0.1;
        }

        servo.setPosition(servoPosition);

        telemetry.addData("servo pos is:", servoPosition);
        telemetry.update();

        // range is 0 - 1

    }

}
