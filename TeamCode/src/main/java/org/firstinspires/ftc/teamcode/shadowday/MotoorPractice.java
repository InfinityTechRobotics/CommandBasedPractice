package org.firstinspires.ftc.teamcode.shadowday;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp

public class MotoorPractice extends OpMode {

    private DcMotor motor;

    double motorSpeed;

    @Override
    public void init() { // runs once

        motor = hardwareMap.dcMotor.get("motorTest");


        motor.setDirection(DcMotor.Direction.FORWARD);

    }


    public void loop() {

        if (gamepad1.xWasPressed()) {
           motorSpeed = 0;
        } else if (gamepad1.bWasPressed()) {
            motorSpeed = 0.5;
        } else if (gamepad1.yWasPressed()) {
            motorSpeed += 0.05;
        } else if (gamepad1.aWasPressed()) {
            motorSpeed -= 0.05;
        }

        motor.setPower(motorSpeed);

        telemetry.addData("Motor Power is", motorSpeed);

        telemetry.update();

    }

}
