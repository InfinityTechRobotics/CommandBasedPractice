package org.firstinspires.ftc.teamcode.Practice;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

//@TeleOp
public class testHF extends OpMode {

    @Override
    public void init() {
    }
    @Override

    public void loop() {
        telemetry.addData("Hi everyone", "Thank you Isha's dad for fixing my gradle!!!");
        telemetry.update();
    }
}