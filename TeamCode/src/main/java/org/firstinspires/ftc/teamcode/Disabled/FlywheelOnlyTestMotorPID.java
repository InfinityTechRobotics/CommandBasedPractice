package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Configurable
@Disabled
@TeleOp
public class FlywheelOnlyTestMotorPID extends OpMode {

    public DcMotorEx motorFlywheel;

    double CPR = 28.;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;
    double targetRPM = 0.;
    double flywheelRPM = 0.;
    double TPS;

    private static ElapsedTime timer = new ElapsedTime();
    double currentTime, prevTime, elapsedTime;

    int i = 0;

    public static double NEW_P = 100.;   // 100.
    public static double NEW_I = 1.;    // 1.
    public static double NEW_D = 20.;    // 20.
    public static double NEW_F = 3.5;    // 3.5

    @Override
    public void init() {

        motorFlywheel = hardwareMap.get(DcMotorEx.class, "motorFlywheel");
        motorFlywheel.setDirection(DcMotorEx.Direction.FORWARD);
        motorFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Set Flywheel Motor PIDF coefficients
        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        motorFlywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

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
            targetRPM = 2400.;
        } else if (gamepad2.aWasPressed()) {
            targetRPM -= 50.;
        } else if (gamepad2.yWasPressed()) {
            targetRPM += 50.;
        }

        // Calculate and set flywheel motor velocity
        TPS = targetRPM / 60. * CPR;
        motorFlywheel.setVelocity(TPS);

        flywheelRPM = motorFlywheel.getVelocity() / CPR * 60;

        i += 1;

        if (i % 100 == 0) {
            currentTime = timer.seconds();
            elapsedTime = currentTime - prevTime;
            prevTime = currentTime;
        }

        // Telemetry Data
        telemetry.addData("Timer", timer.seconds());
        telemetry.addData("Elapsed Time (100 loops)", elapsedTime);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Flywheel RPM", flywheelRPM);
        telemetry.update();



    }

}
