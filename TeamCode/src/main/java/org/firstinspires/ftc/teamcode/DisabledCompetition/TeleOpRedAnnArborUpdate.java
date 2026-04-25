package org.firstinspires.ftc.teamcode.DisabledCompetition;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.Drive;
import org.firstinspires.ftc.teamcode.Hardware.Pinpoint;
import org.firstinspires.ftc.teamcode.DisabledHardware.Shooter;

import java.util.List;

@Disabled
@TeleOp
public class TeleOpRedAnnArborUpdate extends OpMode {

    Drive drive = new Drive();
    Pinpoint pinpoint = new Pinpoint();
    Shooter shooter = new Shooter();
    Pose2D pose2D;

    public static double NEW_P = 100.;   // 10.
    public static double NEW_I = 1.;    // 3.
    public static double NEW_D = 20.;    // 0.
    public static double NEW_F = 3.5;    // 0.

    double h1 = 13.25;  // 14.25
    double h2 = 29.5;
    double a1 = 11.0;
    double a2 = 0.;
    double x1 = -1.0;    // Distance between camera and ramp

    double angleToGoalDegrees, angleToGoalRadians, distanceToGoalInches;

    private Limelight3A limelight;

    private Servo RPMIndicatorLeft, RPMIndicatorRight;

    private static final int DESIRED_TAG_ID = 24;

    private Servo servoTurret;

    double error, currentPos, newPos;

    double bearing;

    double botHeading;

    boolean targetFound = false;

    boolean turretTracking = true;
    public boolean lastBPress;

    double DRIVE_POWER_FACTOR = 0.9;
    double DRIVE_POWER_FACTOR_LOW = 0.6;
    double DRIVE_POWER_FACTOR_HIGH = 1;

    double powerFactor = DRIVE_POWER_FACTOR;

    public DcMotor motorIntake, motorTransfer;
    public DcMotorEx motorFlywheel;

    public boolean lastRightBump, lastLeftBump;
    public boolean lastDpadUp, lastDpadDown;
    public boolean lastDpadLeft, lastDpadRight;

    boolean autoRPM = true;
    boolean fieldCentric = true;
    boolean y1AlreadyPressed;

    public boolean intakeOn, transferOn;

    boolean aAlreadyPressed;
    boolean yAlreadyPressed;

    double CPR = 28.;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;
    double targetRPM = 0.;
    double flywheelRPM = 0.;
    double TPS;

    PIDFCoefficients pidfModified;


    public void init() {

        drive.init(hardwareMap);
        pinpoint.init(hardwareMap);
        shooter.init(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        servoTurret = hardwareMap.get(Servo.class, "servoWebcam");

        RPMIndicatorLeft = hardwareMap.get(Servo.class, "RPMIndicatorLeft");

        RPMIndicatorRight = hardwareMap.get(Servo.class, "RPMIndicatorRight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();

        servoTurret.setPosition(0.5);
        currentPos = servoTurret.getPosition();

        shooter.closeServoStop();
        shooter.downServoPaddle();

        motorIntake = hardwareMap.dcMotor.get("motorIntake");
        motorTransfer = hardwareMap.dcMotor.get("motorTransfer");
        motorFlywheel = hardwareMap.get(DcMotorEx.class, "motorFlywheel");

        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        motorTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorTransfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set Flywheel Motor PIDF coefficients
        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        motorFlywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

        pidfModified = motorFlywheel.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);

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
            currentPos = servoTurret.getPosition();

            angleToGoalDegrees = a1 + a2;
            angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

            distanceToGoalInches = ((h2 - h1) / Math.tan(angleToGoalRadians)) + x1;

            telemetry.addData("Distance To AprilTag", distanceToGoalInches);
            telemetry.addData("Elevation", a2);
            telemetry.addData("Bearing Error", error);
        } else {
            telemetry.addData("\n>", "Target Not Found\n");
            error = 0;
            distanceToGoalInches = 60.;
        }

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

        // Set Flywheel Power
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

        // Calculate and set flywheel motor velocity
        TPS = targetRPM / 60. * CPR;
        motorFlywheel.setVelocity(TPS);


        // Toggle intake when right_bumper is pressed
        if (gamepad2.right_bumper && !lastRightBump) {
            intakeOn = !intakeOn;
        }

        lastRightBump = gamepad2.right_bumper;

        if (intakeOn) {
            motorIntake.setPower(0.8);
        } else {
            motorIntake.setPower(0.);
        }


        // Toggle transfer when left_bumper is pressed
        if (gamepad2.left_bumper && !lastLeftBump) {
            transferOn = !transferOn;
        }

        lastLeftBump = gamepad2.left_bumper;

        if (transferOn) {
            motorTransfer.setPower(0.8);
        } else {
            motorTransfer.setPower(0.);
        }

        // Control Direction of Intake and Transfer Motors
        if (gamepad2.dpad_up && !lastDpadUp) {
            motorTransfer.setPower(0.);
            motorIntake.setPower(0.);
            motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
            motorTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeOn = false;
            transferOn = false;
        }
        lastDpadUp = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !lastDpadDown) {
            motorTransfer.setPower(0.);
            motorIntake.setPower(0.);
            motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
            motorTransfer.setDirection(DcMotorSimple.Direction.REVERSE);
            intakeOn = false;
            transferOn = false;
        }
        lastDpadDown = gamepad2.dpad_down;

        // Control Paddle Servo
        if (gamepad2.right_trigger > 0.25) {
            shooter.shootServoPaddle();
            motorTransfer.setPower(0.);
            transferOn = false;
        } else {
            shooter.downServoPaddle();
        }

        // Control Servo Stop
        if (gamepad2.left_trigger > 0.25) {
            shooter.openServoStop();
        } else {
            shooter.closeServoStop();
        }

        flywheelRPM = motorFlywheel.getVelocity() / CPR * 60;

        // RGB Indicator Lights
        if (flywheelRPM < (targetRPM - 150)) { // turns the RGB lights blue if the flywheel speed is too low
            RPMIndicatorLeft.setPosition(0.611);
            RPMIndicatorRight.setPosition(0.611);
        } else if (flywheelRPM > (targetRPM + 150)) { // turns the RGB lights orange if the flywheel speed is too high
            RPMIndicatorLeft.setPosition(0.3);
            RPMIndicatorRight.setPosition(0.3);
        } else { // turns the RGB indicator green if the flywheel speed is correct
            RPMIndicatorLeft.setPosition(0.5);
            RPMIndicatorRight.setPosition(0.5);
        }


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
        telemetry.addData("Flywheel Motor Power", motorFlywheel.getPower());
        telemetry.addData("P,I,D,F (modified)", "P: %.4f, I: %.4f, D: %.4f, F: %.4f",
                pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);

        telemetry.update();

    }

    public void stop() {
        limelight.stop();
    }

}