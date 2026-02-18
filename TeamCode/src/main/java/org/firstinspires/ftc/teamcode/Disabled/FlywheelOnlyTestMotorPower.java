package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Configurable
@Disabled
@TeleOp
public class FlywheelOnlyTestMotorPower extends OpMode {

    public DcMotorEx motorFlywheel;

    double CPR = 28.;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;
    double targetRPM = 0.;
    double flywheelRPM = 0.;
    double TPS;

    private static ElapsedTime timer = new ElapsedTime();
    double currentTime, prevTime, elapsedTime;

    int i = 0;

    @Override
    public void init() {

        motorFlywheel = hardwareMap.get(DcMotorEx.class, "motorFlywheel");
        motorFlywheel.setDirection(DcMotorEx.Direction.FORWARD);
        motorFlywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

    }

    public void start() {
        timer.reset();
        prevTime = 0.;
    }

    @Override
    public void loop() {

        // Flywheel Speed Setting
        if (gamepad2.xWasPressed()) {
            targetRPM = 0.;
        } else if (gamepad2.bWasPressed()) {
            targetRPM = 0.7;
        } else if (gamepad2.aWasPressed()) {
            targetRPM -= 0.05;
        } else if (gamepad2.yWasPressed()) {
            targetRPM += 0.5;
        }

        // Calculate and set flywheel motor velocity
        motorFlywheel.setPower(targetRPM);

        flywheelRPM = motorFlywheel.getPower();

        i += 1;

        if (i % 100 == 0) {
            currentTime = timer.seconds();
            elapsedTime = currentTime - prevTime;
            prevTime = currentTime;
        }

        // Telemetry Data
        telemetry.addData("Timer", timer.seconds());
        telemetry.addData("Elapsed Time (100 loops)", elapsedTime);
        telemetry.addData("Target Power", targetRPM);
        telemetry.addData("Flywheel Power", flywheelRPM);
        telemetry.update();



    }

}
