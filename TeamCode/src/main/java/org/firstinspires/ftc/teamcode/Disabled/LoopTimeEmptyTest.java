package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp
public class LoopTimeEmptyTest extends OpMode {

    private static ElapsedTime timer = new ElapsedTime();
    double currentTime, prevTime, elapsedTime;

    int i = 0;

    @Override
    public void init() {
    }

    public void start() {
        timer.reset();
        prevTime = 0.;
    }

    @Override
    public void loop() {
        i += 1;
        if (i % 100 == 0) {
            currentTime = timer.seconds();
            elapsedTime = currentTime - prevTime;
            prevTime = currentTime;
        }

        // Telemetry Data
        telemetry.addData("Timer", timer.seconds());
        telemetry.addData("Elapsed Time (100 loops)", elapsedTime);
        telemetry.update();
    }
}
