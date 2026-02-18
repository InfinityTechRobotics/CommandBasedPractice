package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Drive;

@Disabled
@TeleOp
public class TeleOpHardwareOTOS extends OpMode {
    Drive drive = new Drive();
    OTOS otos = new OTOS();

    public void init() {

        drive.init(hardwareMap);
        otos.init(hardwareMap, telemetry);

    }
    public void loop () {

        double y = -gamepad1.left_stick_y;
        double x =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        drive.moveRobot(y, x, rx);

        telemetry.addData("X coordinate", otos.getOtosPosX());
        telemetry.addData("Y coordinate", otos.getOtosPosY());
        telemetry.addData("Heading angle", otos.getOtosPosH());
        telemetry.update();

    }
}
