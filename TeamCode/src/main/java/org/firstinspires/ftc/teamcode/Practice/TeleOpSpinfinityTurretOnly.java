package org.firstinspires.ftc.teamcode.Practice;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Disabled
@TeleOp
public class TeleOpSpinfinityTurretOnly extends OpMode {
    public DcMotorEx motorTurret;

    public int targetPos = 0;

    public double max = 500;

    public double min = -500;

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public void init() {
        motorTurret = hardwareMap.get(DcMotorEx.class, "motorTurret");
        motorTurret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorTurret.setTargetPosition(targetPos);
        motorTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void loop() {

        if (gamepad1.xWasPressed()) {
            targetPos -= 1;
        } else if (gamepad1.bWasPressed()) {
            targetPos += 1;
        } else if (gamepad1.yWasPressed()) {
            targetPos += 10;
        } else if (gamepad1.aWasPressed()) {
            targetPos -= 10;
        }

        motorTurret.setTargetPosition(targetPos);

        motorTurret.setPower(0.5);

        panelsTelemetry.addData("Turret Target Position", targetPos);
        panelsTelemetry.addData("Motor Turret Position", motorTurret.getCurrentPosition());
        panelsTelemetry.addData("Turret Current", motorTurret.getCurrent(CurrentUnit.AMPS));
        panelsTelemetry.update(telemetry);
    }
}