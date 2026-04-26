package org.firstinspires.ftc.teamcode.SKR;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Disabled
//@Configurable
@TeleOp
public class TurretMotorTuning extends OpMode {

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public static double NEW_P = 10.;   // 10.
    public static double NEW_I = 0.;    // 3.
    public static double NEW_D = 0.;    // 0.
    public static double NEW_F = 0.;    // 0.

    public DcMotorEx motorTurret;

    PIDFCoefficients pidfModified;

    int targetPos = 0;


    public void init() {

        telemetry.setMsTransmissionInterval(11);

        motorTurret = hardwareMap.get(DcMotorEx.class, "motorTurret");
        motorTurret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorTurret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorTurret.setTargetPosition(0);
        motorTurret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorTurret.setDirection(DcMotorEx.Direction.FORWARD);

        panelsTelemetry.debug("Init was ran!");
        panelsTelemetry.update(telemetry);

        motorTurret.setPower(0.75);
    }

    public void loop() {

        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        motorTurret.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

        motorTurret.setPower(0.75);

        // Set Flywheel Velocity
        if (gamepad2.x) {
            targetPos = 0;
        } else if (gamepad2.b) {
            targetPos = 500;
        } else if (gamepad2.a) {
            targetPos = -500;
        } else if (gamepad2.yWasPressed()) {
            targetPos += 100;
        }

        motorTurret.setTargetPosition(targetPos);


        // Panels Telemetry Data
        panelsTelemetry.addData("Target Pos", targetPos);
        panelsTelemetry.addData("Current Pos", motorTurret.getCurrentPosition());
        panelsTelemetry.update(telemetry);

    }

}