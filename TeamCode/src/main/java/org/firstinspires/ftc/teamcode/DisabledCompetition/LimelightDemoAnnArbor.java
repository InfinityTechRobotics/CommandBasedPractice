package org.firstinspires.ftc.teamcode.DisabledCompetition;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@Disabled
@TeleOp
public class LimelightDemoAnnArbor extends OpMode {

    private Limelight3A limelight;

    private static final int DESIRED_TAG_ID = 20;

    private Servo servoTurret;

    double bearing;

    boolean targetFound = false;

    boolean turretTracking = true;

    double error, currentPos, newPos;

    public boolean lastBPress;

    public void init() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        servoTurret = hardwareMap.get(Servo.class, "servoWebcam");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();

        servoTurret.setPosition(0.56);
        currentPos = servoTurret.getPosition();

    }

    public void loop() {

        LLResult result = limelight.getLatestResult();

        if (result.isValid()) {
            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                if (fr.getFiducialId() == DESIRED_TAG_ID) {
                    bearing = fr.getTargetXDegrees();
                    targetFound = true;
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", fr.getFiducialId());
                    bearing = 0.;
                    targetFound = false;
                }
            }
        } else {
            telemetry.addData("Limelight", "No data available");
            bearing = 0.;
            targetFound = false;
        }

        if (targetFound) {
            error = bearing;
            currentPos = servoTurret.getPosition();
            telemetry.addData("Bearing Error", error);
        } else {
            error = 0;
            telemetry.addData("\n>", "Target Not Found\n");
        }

        if (Math.abs(error) > 1.0) {
            newPos = currentPos + error * 0.0001;
        } else {
            newPos = currentPos;
        }



        // Toggle turret when B is pressed on gamepad 1
        if (gamepad1.b && !lastBPress) {
            turretTracking = !turretTracking;
        }

        lastBPress = gamepad1.b;

        if (turretTracking) {
            servoTurret.setPosition(newPos);
        } else {
            servoTurret.setPosition(0.5);
        }
        telemetry.update();

    }

    public void stop() {
        limelight.stop();
    }

}