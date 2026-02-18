package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp

public class DetectingArtifactsBothSR extends OpMode {
    Limelight3A limelight;

    Servo rgbLight1;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(20);

        limelight.pipelineSwitch(2);

        limelight.start();


        rgbLight1 = hardwareMap.get(Servo.class, "rgblight1");
    }

    public void loop() {

        if (gamepad1.a) {
            limelight.pipelineSwitch(3);
            telemetry.addData("Ball Color", "Green");
        }
        else {
            limelight.pipelineSwitch(2);
            telemetry.addData("Ball Color", "Purple");
        }

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {

            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa();

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);

            if (result.getPipelineIndex() == 2) {
                rgbLight1.setPosition(0.722);
            } else if (result.getPipelineIndex() == 3) {
                rgbLight1.setPosition(0.5);
            }

        } else {
            telemetry.addData("Limelight", "No Targets");
            rgbLight1.setPosition(0.);
        }
        telemetry.addData("Pipeline Index", result.getPipelineIndex());

        telemetry.update();
    }
}
