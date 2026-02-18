package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp
public class MotorRPMIndicator extends OpMode {
    private static ElapsedTime timer = new ElapsedTime();
    private DcMotor motor;
    private DcMotor motor2;
    private Servo servoWebcam;
    double  motorPower = 0.;
    double motor2Power = 0.;
    boolean aAlreadyPressed;
    boolean yAlreadyPressed;
    boolean dpadDAlreadyPressed;
    boolean dpadUAlreadyPressed;

    double currentPos = 0;
    double previousPos = 0;
    double CPR = 145.1;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;
    double deltaRev = 0;
    double previousTime = 0;
    double currentTime = 0;
    double deltaTime = 0;
    double RPM = 0;

    int i = 0;


    @Override public void init() {

        timer.reset();

        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motor2 = hardwareMap.get(DcMotor.class, "motor1");
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        servoWebcam = hardwareMap.get(Servo.class,"servoWebcam");

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

        if (i % 10 == 0) {
            currentPos = motor.getCurrentPosition();
            currentTime = timer.time();

            deltaRev = (currentPos - previousPos) / CPR;
            deltaTime = currentTime - previousTime;

            RPM = deltaRev / deltaTime * 60;

            previousPos = currentPos;
            previousTime = currentTime;
        }
        if (RPM > 1000){
            servoWebcam.setPosition(0.555);
        }
        else {
            servoWebcam.setPosition(0.333);
        }

        telemetry.addData("Motor 1 Power", motor.getPower());
        telemetry.addData("Motor 2 Power", motor2.getPower());
        telemetry.addData("RPM", RPM);
        telemetry.addData("Timer", timer.time());
        telemetry.addData("i",i);
        telemetry.update();

        i += 1;
    }

}
