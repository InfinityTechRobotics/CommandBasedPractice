package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp
public class MotorTestIncrementINSR extends OpMode {
    private DcMotor motor;
    private DcMotor motor2;
    double  motorPower = 0.;
    double motor2Power = 0.;
    boolean aAlreadyPressed;
    boolean yAlreadyPressed;
    boolean dpadDAlreadyPressed;
    boolean dpadUAlreadyPressed;

    @Override public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motor2 = hardwareMap.get(DcMotor.class, "motor1");
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            motorPower = 0.;
        }
        else if (gamepad1.y && !yAlreadyPressed) {
            motorPower = motorPower + 0.05;
        }
        else if (gamepad1.a && !aAlreadyPressed) {
            motorPower = motorPower - 0.05;
        }
        else if (gamepad1.dpad_down && !dpadDAlreadyPressed) {
            motor2Power = motor2Power - 0.05;
        }
        else if (gamepad1.dpad_up && !dpadUAlreadyPressed) {
            motor2Power = motor2Power + 0.05;
        }
        else if (gamepad1.dpad_left) {
            motor2Power = 0.;
        }

        motor2.setPower(motor2Power);
        motor.setPower(motorPower);

        aAlreadyPressed = gamepad1.a;
        yAlreadyPressed = gamepad1.y;

        dpadUAlreadyPressed = gamepad1.dpad_up;
        dpadDAlreadyPressed = gamepad1.dpad_down;

        telemetry.addData("Motor 1 Power", motor.getPower());
        telemetry.addData("Motor 2 Power", motor2.getPower());
        telemetry.update();
    }

}
