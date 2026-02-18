package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelight {

    private Limelight3A limelight;

    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);

        limelight.start();

    }

    public LLResult getLimelightResult() {
        return limelight.getLatestResult();
    }

    public void stopLimelight() {
        limelight.stop();
    }

}
