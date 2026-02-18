package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class testSurya extends OpMode {

    @Override
    public void init() {
    }

    @Override
    public void loop() {
        telemetry.addData("Hello Team", "Infinity Tech 13684");
        telemetry.update();

    }

}
