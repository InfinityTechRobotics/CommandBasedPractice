package org.firstinspires.ftc.teamcode.shadowday;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@TeleOp

public class MotoorPracticeEncoderTest extends OpMode {

    private DcMotor motor;

    int motorPos;

    @Override
    public void init() { // runs once

        motor = hardwareMap.dcMotor.get("motorTurret");


        motor.setDirection(DcMotor.Direction.FORWARD);

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


    public void loop() {

        if (gamepad1.xWasPressed()) {
           motorPos -= 10;
        } else if (gamepad1.bWasPressed()) {
            motorPos += 10;
        } else if (gamepad1.yWasPressed()) {
            motorPos += 25;
        } else if (gamepad1.aWasPressed()) {
            motorPos -= 25;
        }

        motor.setTargetPosition(motorPos);

        telemetry.addData("Motor Power is", motorPos);

        telemetry.update();

    }

}
