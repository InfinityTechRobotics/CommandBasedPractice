package org.firstinspires.ftc.teamcode.SKR;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

//@Disabled
@Configurable
@TeleOp
public class FlywheelDuoTuning extends OpMode {

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public static double NEW_P = 10.;   // 10.
    public static double NEW_I = 3.;    // 3.
    public static double NEW_D = 0.;    // 0.
    public static double NEW_F = 0.;    // 0.

    public DcMotorEx motorFlywheel;
    public DcMotorEx motorFlywheel2;

    double CPR = 28.;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;
    double targetRPM = 0.;
    double flywheelRPM = 0.;
    double flywheelRPM2 = 0.;
    double TPS;

    PIDFCoefficients pidfModified;
    PIDFCoefficients pidfModified2;


    public void init() {

        telemetry.setMsTransmissionInterval(11);

        motorFlywheel = hardwareMap.get(DcMotorEx.class, "motorFlywheel");
        motorFlywheel2 = hardwareMap.get(DcMotorEx.class, "motorFlywheel2");

        motorFlywheel.setDirection(DcMotorEx.Direction.REVERSE);
        motorFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motorFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorFlywheel2.setDirection(DcMotorEx.Direction.FORWARD);
        motorFlywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motorFlywheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        panelsTelemetry.debug("Init was ran!");
        panelsTelemetry.update(telemetry);

    }

    public void loop() {

        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        motorFlywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);
        motorFlywheel2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

        pidfModified = motorFlywheel.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pidfModified2 = motorFlywheel2.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set Flywheel Velocity
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
        flywheelRPM2 = motorFlywheel2.getVelocity() / CPR * 60;

        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Flywheel RPM", flywheelRPM);
        telemetry.addData("Flywheel 2 RPM", flywheelRPM2);
        telemetry.addData("P,I,D,F (modified)", "P: %.4f, I: %.4f, D: %.4f, F: %.4f",
                pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);
        telemetry.addData("P,I,D,F (modified) 2", "P: %.4f, I: %.4f, D: %.4f, F: %.4f",
                pidfModified2.p, pidfModified2.i, pidfModified2.d, pidfModified2.f);
        telemetry.update();

        // Panels Telemetry Data
        panelsTelemetry.addData("Target RPM", targetRPM);
        panelsTelemetry.addData("Flywheel RPM", flywheelRPM);
        panelsTelemetry.addData("Flywheel 2 RPM", flywheelRPM2);
        panelsTelemetry.update(telemetry);

    }

}