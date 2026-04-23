package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.DisabledHardware.Flywheel;
import org.firstinspires.ftc.teamcode.Hardware.Pinpoint;
import org.firstinspires.ftc.teamcode.DisabledHardware.Shooter;

@Disabled
@TeleOp
public class TeleOpSpinfinityTest2 extends OpMode {

    Drive2 drive = new Drive2();
    Pinpoint pinpoint = new Pinpoint();
    Shooter shooter = new Shooter();
    Spintake2 spintake = new Spintake2();
    Flywheel flywheel = new Flywheel();

    Pose2D pose2D;

    double distanceToGoalInches;
    double a2 = 0;

    double error, currentPos, newPos;

    double botHeading;

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

        telemetry.setMsTransmissionInterval(11);

        shooter.centerServoTurret();
        currentPos = shooter.servoTurretGetPosition();

        shooter.closeServoStop();
        shooter.downServoPaddle();

    }

    public void loop() {

        // Driver Controls
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1; //counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        if (gamepad1.left_bumper) {
            powerFactor = DRIVE_POWER_FACTOR_LOW;
        } else if (gamepad1.right_bumper) {
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

        drive.moveRobotFC(y, x, rx, botHeading, powerFactor);

        telemetry.addData("Limelight", "No data available");

        error = 0;
        distanceToGoalInches = 48.;


        // Toggle turret auto tracking when B is pressed on gamepad 1
        if (gamepad1.b && !lastBPress) {
            turretTracking = !turretTracking;
        }

        lastBPress = gamepad1.b;

        if (turretTracking) {
            currentPos = shooter.servoTurretGetPosition();

            if (Math.abs(error) > 1.0) {
                newPos = shooter.newTurretPositionCalc(currentPos, error);
            } else {
                newPos = currentPos;
            }

            shooter.servoTurretSetPosition(newPos);
        } else {
            shooter.centerServoTurret();
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
            targetRPM = flywheel.targetRPMCalc(distanceToGoalInches);     // targetRPM = 12.1 * distanceToGoalInches + 1725;
        } else {
            // manually set RPM distance
            if (gamepad2.x) {
                targetRPM = 0.;
            } else if (gamepad2.b) {
                targetRPM = 2300.;
            } else if (gamepad2.a && !aAlreadyPressed) {
                targetRPM -= 50.;
            } else if (gamepad2.y && !yAlreadyPressed) {
                targetRPM += 50.;
            }
        }
        aAlreadyPressed = gamepad2.a;
        yAlreadyPressed = gamepad2.y;

        // Set Flywheel RPM
//        flywheel.setFlywheelVel(targetRPM);
        flywheel.setFlywheelVel(0.);


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
        } else {
            spintake.turnTransferOff();
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
        telemetry.update();

    }

    public void stop() {

    }

}