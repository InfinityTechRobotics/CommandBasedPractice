package org.firstinspires.ftc.teamcode.SKR;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
@TeleOp
public class FlywheelTuning extends OpMode {

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public static double NEW_P = 10.;   // 10.
    public static double NEW_I = 3.;    // 3.
    public static double NEW_D = 0.;    // 0.
    public static double NEW_F = 0.;    // 0.

    public DcMotorEx motorFlywheel;

    double CPR = 28.;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;
    double targetRPM = 0.;
    double flywheelRPM = 0.;
    double TPS;

    PIDFCoefficients pidfModified;


    public void init() {

        telemetry.setMsTransmissionInterval(11);

        motorFlywheel = hardwareMap.get(DcMotorEx.class, "motorFlywheel");

        motorFlywheel.setDirection(DcMotorEx.Direction.FORWARD);
        motorFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motorFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        panelsTelemetry.debug("Init was ran!");
        panelsTelemetry.update(telemetry);

    }

    public void loop() {

        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        motorFlywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

        pidfModified = motorFlywheel.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set Flywheel Velocity
        if (gamepad2.x) {
            targetRPM = 0.;
        } else if (gamepad2.b) {
            targetRPM = 2500.;
        } else if (gamepad2.a) {
            targetRPM -= 50.;
        } else if (gamepad2.y) {
            targetRPM += 50.;
        }

        // Calculate and set flywheel motor velocity
        TPS = targetRPM / 60. * CPR;
        motorFlywheel.setVelocity(TPS);

        flywheelRPM = motorFlywheel.getVelocity() / CPR * 60;

        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Flywheel RPM", flywheelRPM);
        telemetry.addData("P,I,D,F (modified)", "P: %.4f, I: %.4f, D: %.4f, F: %.4f",
                pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);
        telemetry.update();

        // Panels Telemetry Data
        panelsTelemetry.addData("Target RPM", targetRPM);
        panelsTelemetry.addData("Flywheel RPM", flywheelRPM);
        panelsTelemetry.update(telemetry);

    }

}