package org.firstinspires.ftc.teamcode.Practice;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

@Disabled
//@Configurable
@TeleOp

public class FlywheelPID extends OpMode {

    public DcMotorEx flywheelMotor;
    private ControlSystem controller;

    public static double NEW_P = 100.;   // 10.
    public static double NEW_I = 1.;    // 3.
    public static double NEW_D = 20.;    // 0.
    public static double NEW_F = 3.5;    // 0.
    PIDFCoefficients pidfModified;

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "motorFlywheel");

        flywheelMotor.setDirection(DcMotorEx.Direction.FORWARD);

        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        flywheelMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

//        controller = ControlSystem.builder()
//                .velPid(100, 1, 20)
//                .build();
//
//        controller.setGoal(new KineticState(0.0, 0.0));

        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        flywheelMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

        pidfModified = flywheelMotor.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);

        panelsTelemetry.debug("Init was ran!");
        panelsTelemetry.update(telemetry);

    }

    @Override
    public void loop() {

        if (gamepad1.aWasPressed()) {
            flywheelMotor.setVelocity(1120);
        } else if (gamepad1.bWasPressed()) {
            flywheelMotor.setVelocity(0);
        } else if (gamepad1.xWasPressed()) {
            flywheelMotor.setVelocity(560);
        }
//        if (gamepad1.aWasPressed()) {
//            controller.setGoal(new KineticState(0.0, 1120));
//        } else if (gamepad1.bWasPressed()) {
//            controller.setGoal(new KineticState(0.0, 0.0));
//        } else if (gamepad1.xWasPressed()) {
//            controller.setGoal(new KineticState(0.0, 560));
//        }

        flywheelMotor.setPower(controller.calculate(new KineticState(
                flywheelMotor.getCurrentPosition(),
                flywheelMotor.getVelocity()))
        );

        telemetry.addData("P,I,D,F (modified)", "P: %.4f, I: %.4f, D: %.4f, F: %.4f",
                pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);



    }

}



