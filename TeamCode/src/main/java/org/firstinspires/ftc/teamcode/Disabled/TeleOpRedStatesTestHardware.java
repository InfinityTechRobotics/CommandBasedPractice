package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.Drive;
import org.firstinspires.ftc.teamcode.DisabledHardware.Flywheel;
import org.firstinspires.ftc.teamcode.Hardware.Pinpoint;
import org.firstinspires.ftc.teamcode.DisabledHardware.Shooter;
import org.firstinspires.ftc.teamcode.DisabledHardware.Spintake;

import java.util.List;

@Disabled
@TeleOp
public class TeleOpRedStatesTestHardware extends OpMode {

    Drive drive = new Drive();
    Pinpoint pinpoint = new Pinpoint();
    Shooter shooter = new Shooter();
    Spintake spintake = new Spintake();
    Flywheel flywheel = new Flywheel();

    Pose2D pose2D;

    double distanceToGoalInches;
    double a2 = 0;

    private Limelight3A limelight;

    private static final int DESIRED_TAG_ID = 24;

    private Servo servoTurret;

    double error, currentPos, newPos;

    double bearing;

    double botHeading;

    boolean targetFound = false;

    boolean turretTracking = true;
    public boolean lastBPress;

    double DRIVE_POWER_FACTOR = 0.95;
    double DRIVE_POWER_FACTOR_LOW = 0.6;
    double DRIVE_POWER_FACTOR_HIGH = 1;

    double powerFactor = DRIVE_POWER_FACTOR;

    public boolean lastRightBump, lastLeftBump;
    public boolean lastDpadUp, lastDpadDown;
    public boolean lastDpadLeft, lastDpadRight;
    boolean aAlreadyPressed, yAlreadyPressed;
    boolean y1AlreadyPressed;

    boolean autoRPM = true;
    boolean fieldCentric = true;

    public boolean intakeOn, transferOn;

    double targetRPM = 0.;
    double flywheelRPM = 0.;


    public void init() {

        drive.init(hardwareMap);
        pinpoint.init(hardwareMap);
        shooter.init(hardwareMap);
        spintake.init(hardwareMap);
        flywheel.init(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        servoTurret = hardwareMap.get(Servo.class, "servoWebcam");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();

        servoTurret.setPosition(0.5);
        currentPos = servoTurret.getPosition();

        shooter.closeServoStop();
        shooter.downServoPaddle();

    }

    public void loop() {

        // Driver Controls
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1; //counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        if (gamepad1.left_bumper || gamepad1.right_bumper) {
            powerFactor = DRIVE_POWER_FACTOR_LOW;
        } else if (gamepad1.left_stick_button) {
            powerFactor = DRIVE_POWER_FACTOR_HIGH;
        } else {
            powerFactor = DRIVE_POWER_FACTOR;
        }

        if (gamepad1.a) {
            pinpoint.pinpointReset();
        }

        if (gamepad1.y && !y1AlreadyPressed) {
            fieldCentric = !fieldCentric;
        }

        y1AlreadyPressed = gamepad1.y;

        pose2D = pinpoint.getPinpointPose();

        if (fieldCentric) {
            botHeading = pose2D.getHeading(AngleUnit.RADIANS);
        }
        else {
            botHeading = 0;
        }

        if (gamepad1.left_bumper) {
            drive.moveRobotRC(y, x, rx, powerFactor);
        } else {
            drive.moveRobotFC(y, x, rx, botHeading, powerFactor);
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
                    targetFound = false;
                }
            }
        } else {
            telemetry.addData("Limelight", "No data available");
            targetFound = false;
        }

        if (targetFound) {
            error = bearing;
            distanceToGoalInches = flywheel.distanceToGoalCalc(a2);

            telemetry.addData("Distance To AprilTag", distanceToGoalInches);
            telemetry.addData("Bearing Error", error);
        } else {
            error = 0;
            distanceToGoalInches = 60.;

            telemetry.addData("\n>", "Target Not Found\n");
        }

        currentPos = servoTurret.getPosition();

        if (Math.abs(error) > 1.0) {
            newPos = currentPos + error * 0.0008; // 0.0004
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

        // Turn Auto RPM Calculation On or Off
        if (gamepad2.left_stick_button) {
            autoRPM = true;
        } else if (gamepad2.right_stick_button) {
            autoRPM = false;
        }

        // Calculate Flywheel Target RPM
        if (autoRPM) {
            // dynamically set flywheel speed based off Limelight distance measurement
            targetRPM = 11.7 * distanceToGoalInches + 1743;     // targetRPM = 12.1 * distanceToGoalInches + 1725;
        } else {
            // manually set RPM distance
            if (gamepad2.x) {
                targetRPM = 0.;
            } else if (gamepad2.b) {
                targetRPM = 2500.;
            } else if (gamepad2.a && !aAlreadyPressed) {
                targetRPM -= 50.;
            } else if (gamepad2.y && !yAlreadyPressed) {
                targetRPM += 50.;
            }
        }

        aAlreadyPressed = gamepad2.a;
        yAlreadyPressed = gamepad2.y;

        // Set Flywheel RPM
        flywheel.setFlywheelVel(targetRPM);

        // Control Direction of Intake and Transfer Motors
        if (gamepad2.dpad_up && !lastDpadUp) {
            spintake.forwardSpintakes();
            intakeOn = false;
            transferOn = false;
        }
        lastDpadUp = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !lastDpadDown) {
            spintake.reverseSpintakes();
            intakeOn = false;
            transferOn = false;
        }
        lastDpadDown = gamepad2.dpad_down;


        // Toggle intake when right_bumper is pressed
        if (gamepad2.right_bumper && !lastRightBump) {
            intakeOn = !intakeOn;
        }

        lastRightBump = gamepad2.right_bumper;

        if (intakeOn) {
            spintake.turnIntakeOn();
        } else {
            spintake.turnIntakeOff();
        }

        // Toggle transfer when left_bumper is pressed
        if (gamepad2.left_bumper && !lastLeftBump) {
            transferOn = !transferOn;
        }

        lastLeftBump = gamepad2.left_bumper;

        if (transferOn) {
            spintake.turnTransferOn();
            spintake.setTransferLEDOn();
        } else {
            spintake.turnTransferOff();
            spintake.setTransferLEDOff();
        }


        // Control Paddle Servo
        if (gamepad2.right_trigger > 0.25) {
            shooter.shootServoPaddle();
        } else {
            shooter.downServoPaddle();
        }

        // Control Servo Stop
        if (gamepad2.left_trigger > 0.25) {
            shooter.openServoStop();
        } else {
            shooter.closeServoStop();
        }

        flywheelRPM = flywheel.getFlywheelVel();

        // Set RGB Indicator Lights
        flywheel.setFlywheelRGB(flywheelRPM, targetRPM);

        // Telemetry Data
        telemetry.addData("Field Centric", fieldCentric);
        telemetry.addData("Drive Power Factor", powerFactor);
        telemetry.addData("Auto Turret", turretTracking);
        telemetry.addData("Auto RPM", autoRPM);
        telemetry.addData("Intake On", intakeOn);
        telemetry.addData("Transfer On", transferOn);
        telemetry.addData("Stop Servo Position", shooter.servoStopPosition());
        telemetry.addData("Paddle Servo Position", shooter.servoPaddlePosition());
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Flywheel RPM", flywheelRPM);
//        telemetry.addData("Flywheel Motor Power", motorFlywheel.getPower());
//        telemetry.addData("P,I,D,F (modified)", "P: %.4f, I: %.4f, D: %.4f, F: %.4f",
//                pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);

        telemetry.update();

    }

    public void stop() {
        limelight.stop();
    }

}