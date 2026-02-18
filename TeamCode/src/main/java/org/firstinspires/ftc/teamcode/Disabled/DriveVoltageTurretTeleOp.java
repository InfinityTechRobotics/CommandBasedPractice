package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.Drive;
import org.firstinspires.ftc.teamcode.Hardware.Pinpoint;

import java.util.List;

@Disabled
@TeleOp
public class DriveVoltageTurretTeleOp extends OpMode {
    Drive drive = new Drive();
    Pinpoint pinpoint = new Pinpoint();
    Pose2D pose2D;

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

    boolean targetFound = false;

    boolean turretTracking = true;
    public boolean lastBPress;

    double DRIVE_POWER_FACTOR = 0.8;
    double DRIVE_POWER_FACTOR_LOW = 0.5;
    double DRIVE_POWER_FACTOR_HIGH = 1;
    double powerFactor = DRIVE_POWER_FACTOR;

    public DcMotor motorIntake, motorTransfer, motorFlywheel;
    public Servo servoPaddleLeft, servoPaddleRight;
    public boolean lastRightBump, lastLeftBump;
    public boolean lastDpadUp, lastDpadDown;


    private VoltageSensor battery;

    boolean voltSpeed = false;

    public boolean intakeOn, transferOn;


    public void init() {

        drive.init(hardwareMap);
        pinpoint.init(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        servo = hardwareMap.get(Servo.class, "servoWebcam");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();

        servo.setPosition(0.5);

        motorIntake = hardwareMap.dcMotor.get("motorIntake");
        motorTransfer = hardwareMap.dcMotor.get("motorTransfer");
        motorFlywheel = hardwareMap.dcMotor.get("motorFlywheel");

        motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        motorTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorTransfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        servoPaddleLeft = hardwareMap.servo.get("servoPaddleLeft");
        servoPaddleRight = hardwareMap.servo.get("servoPaddleRight");

        battery = hardwareMap.get(VoltageSensor.class, "Control Hub");

    }

    public void loop() {

        // Driver Controls

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.left_bumper || gamepad1.right_bumper) {
            powerFactor = DRIVE_POWER_FACTOR_LOW;
        } else if (gamepad1.left_stick_button) {
            powerFactor = DRIVE_POWER_FACTOR_HIGH;
        } else {
            powerFactor = DRIVE_POWER_FACTOR;
        }

        pose2D = pinpoint.getPinpointPose();

        if (gamepad1.a) {
            pinpoint.pinpointReset();
        }

        if (gamepad1.left_bumper) {
            drive.moveRobotRC(y, x, rx, powerFactor);
        } else {
            double botHeading = pose2D.getHeading(AngleUnit.RADIANS);
            drive.moveRobotFC(y, x, rx, botHeading, powerFactor);
        }

//        drive.moveRobotRC(y, x, rx, powerFactor);


        // Operator Controls
        if (gamepad2.left_stick_button) {
            voltSpeed = true;
        } else if (gamepad2.right_stick_button) {
            voltSpeed = false;
        }

        // Set Flywheel Power
        if (gamepad2.a) {
            motorFlywheel.setPower(0.0);
        } else if (gamepad2.b) {
            motorFlywheel.setPower(0.6);
        } else if (gamepad2.x) {
            motorFlywheel.setPower(0.75);
        } else if (gamepad2.y) {
            motorFlywheel.setPower(0.9);
        } else if (voltSpeed == true) {
            // dynamically set flywheel speed based off of voltage
            motorFlywheel.setPower(1.5715 - (0.07 * (battery.getVoltage())));
        }


        // Toggle intake when right_bumper is pressed
        if (gamepad2.right_bumper && !lastRightBump) {
            intakeOn = !intakeOn;
            if (intakeOn) {
                motorIntake.setPower(0.8);
            } else {
                motorIntake.setPower(0.);
            }
        }

        lastRightBump = gamepad2.right_bumper;

        // Toggle transfer when left_bumper is pressed
        if (gamepad2.left_bumper && !lastLeftBump) {
            transferOn = !transferOn;
            if (transferOn) {
                motorTransfer.setPower(0.8);
            } else {
                motorTransfer.setPower(0.);
            }
        }
        lastLeftBump = gamepad2.left_bumper;

        // Control Direction of Intake and Transfer Motors
        if (gamepad2.dpad_up && !lastDpadUp) {
            motorTransfer.setPower(0.);
            motorIntake.setPower(0.);
            motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
            motorTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        lastDpadUp = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !lastDpadDown) {
            motorTransfer.setPower(0.);
            motorIntake.setPower(0.);
            motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
            motorTransfer.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        lastDpadDown = gamepad2.dpad_down;

        // Control Paddle Servo
        if (gamepad2.right_trigger > 0.25) {
//            servoPaddleLeft.setPosition(0.45);
            servoPaddleLeft.setPosition(0.24);
            motorTransfer.setPower(0.);
        } else {
            servoPaddleLeft.setPosition(0.37);
//            servoPaddleRight.setPosition(0.08);
        }



        LLResult result = limelight.getLatestResult();

        if (result.isValid()) {
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

        if (targetFound = true) {
           error = bearing;
//                range = desiredTag.ftcPose.range;
           currentPos = servo.getPosition();

           angleToGoalDegrees = a1 + a2;
           angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

           distanceFromLimelightToGoalInches = (h2 - h1) / Math.tan(angleToGoalRadians);

           telemetry.addData("Distance To Apriltag", distanceFromLimelightToGoalInches);
           telemetry.addData("Bearing Error", error);
       }

            if (Math.abs(error) > 1.0) {
                newPos = currentPos + error * 0.0004;
            } else {
                newPos = currentPos;

            }

        // Toggle turret when B is pressed on gamepad 1
        if (gamepad1.b && !lastBPress) {
            turretTracking = !turretTracking;
        }

        lastBPress = gamepad1.b;

        if (turretTracking) {
            servo.setPosition(newPos);
        } else {
            servo.setPosition(0.5);
        }



            //           telemetry.addData("Range",range);

            {
                telemetry.addData("\n>", "Target Not Found\n");
            }


            // Telemetry Data
            telemetry.addData("Drive Power Factor", powerFactor);
            telemetry.addData("Intake Motor Power", motorIntake.getPower());
            telemetry.addData("Transfer Motor Power", motorTransfer.getPower());
            telemetry.addData("Flywheel Motor Power", motorFlywheel.getPower());
            telemetry.addData("Paddle Servo Position", servoPaddleLeft.getPosition());
            telemetry.addData("Voltage", battery.getVoltage());

            telemetry.update();

    }

    public void stop() {
        limelight.stop();
    }

}