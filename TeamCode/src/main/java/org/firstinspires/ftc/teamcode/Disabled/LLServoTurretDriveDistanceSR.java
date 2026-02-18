package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Hardware.Drive;
import org.firstinspires.ftc.teamcode.Hardware.Pinpoint;

import java.util.List;

@Disabled
@TeleOp

public class LLServoTurretDriveDistanceSR extends LinearOpMode {

    double h1 = 14.;
    double h2 = 29.5;
    double a1 = 20.;
    double a2 = 0.;

    double angleToGoalDegrees, angleToGoalRadians, distanceFromLimelightToGoalInches;


    private double distance;

    private Limelight3A limelight;

    private static final int DESIRED_TAG_ID = 24;

    private Servo servo;

    double error, currentPos, newPos, range;

    double bearing;

    @Override
    public void runOpMode() {

        Drive drive = new Drive();
        Pinpoint pinpoint = new Pinpoint();
        Pose2D pose2D;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        servo = hardwareMap.get(Servo.class, "servoWebcam");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();

        boolean targetFound = false;

        servo.setPosition(0.5);

        double powerFactor = 0.8;

        drive.init(hardwareMap);
        pinpoint.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            pose2D = pinpoint.getPinpointPose();

            if (gamepad1.left_bumper) {
                drive.moveRobotRC(y, x, rx, powerFactor);
            } else {
                double botHeading = pose2D.getHeading(AngleUnit.RADIANS);
                drive.moveRobotFC(y, x, rx, botHeading, powerFactor);
            }

            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());


            LLResult result = limelight.getLatestResult();

            if (result.isValid()) {
                // Access general information
                Pose3D botpose = result.getBotpose();

                telemetry.addData("tx", result.getTx());
                telemetry.addData("txnc", result.getTxNC());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("tync", result.getTyNC());

                telemetry.addData("Botpose", botpose.toString());

                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    if (fr.getFiducialId() == DESIRED_TAG_ID) {
                        bearing = fr.getTargetXDegrees();
                        a2 = fr.getTargetYDegrees();
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
            }

            if(targetFound) {
                error = bearing;
                currentPos = servo.getPosition();

                angleToGoalDegrees = a1 + a2;
                angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

                distanceFromLimelightToGoalInches = (h2 - h1) / Math.tan(angleToGoalRadians);

                telemetry.addData("Distance To Apriltag", distanceFromLimelightToGoalInches);
                telemetry.addData("Bearing Error",error);

                if (Math.abs(error) > 1.0 ) {
                    newPos = currentPos + error * 0.0004;
                } else {
                    newPos = currentPos;

                }

                servo.setPosition(newPos);


     //           telemetry.addData("Range",range);
            }
            else {
                telemetry.addData("\n>","Target Not Found\n");
            }

            telemetry.update();

            
        }
        limelight.stop();
    }
}
