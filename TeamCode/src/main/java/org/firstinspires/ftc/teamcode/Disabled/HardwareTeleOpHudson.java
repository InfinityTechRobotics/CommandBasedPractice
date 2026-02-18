package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.Drive;

@Disabled
public class HardwareTeleOpHudson extends OpMode {

    Drive drive = new Drive();

    public void init() {

        drive.init(hardwareMap);

    }
    public void loop () {

        double y = -gamepad1.left_stick_y;
        double x =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        drive.moveRobot(y, x, rx);
    }
}
