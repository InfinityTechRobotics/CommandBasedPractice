package org.firstinspires.ftc.teamcode.Disabled;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.Drive;
import org.firstinspires.ftc.teamcode.Hardware.Pinpoint;


import java.util.List;

@Disabled
@Configurable
@TeleOp
public class TeleOpRedSetVelocityPanels extends OpMode {

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    Drive drive = new Drive();
    Pinpoint pinpoint = new Pinpoint();
    Pose2D pose2D;

    double h1 = 14.25;
    double h2 = 29.5;
    double a1 = 19.0;
    double a2 = 0.;

    double angleToGoalDegrees, angleToGoalRadians, distanceToGoalInches;
    double targetRPM;

    private Limelight3A limelight;

    private Servo servoStop;

    private Servo RPMIndicatorLeft, RPMIndicatorRight;

    private static final int DESIRED_TAG_ID = 24;

    private Servo servoTurret;

    double error, currentPos, newPos;

    double bearing;

    boolean targetFound = false;

    boolean turretTracking = true;
    public boolean lastBPress;

    double DRIVE_POWER_FACTOR = 0.8;
    double DRIVE_POWER_FACTOR_LOW = 0.5;
    double DRIVE_POWER_FACTOR_HIGH = 1;
    double SERVO_STOP_OPEN_POS = 0.15;
    double SERVO_STOP_CLOSE_POS = 0.33;

    double SERVO_PADDLE_SHOOT_POS = 0.3;
    double SERVO_PADDLE_DOWN_POS = 0.5;

    double powerFactor = DRIVE_POWER_FACTOR;


    public DcMotor motorIntake, motorTransfer;
    public DcMotorEx motorFlywheel;

    public Servo servoPaddleLeft;
    public boolean lastRightBump, lastLeftBump;
    public boolean lastDpadUp, lastDpadDown;
    public boolean lastDpadLeft, lastDpadRight;

    private VoltageSensor battery;

    boolean autoRPM = true;

    public boolean intakeOn, transferOn;

    private static ElapsedTime timer = new ElapsedTime();
    double currentTicks = 0.;
    double previousTicks = 0.;
    double CPR = 28.;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;
    double deltaRev = 0.;
    double previousTime = 0.;
    double currentTime = 0.;
    double deltaTime = 0.;
    double flywheelRPM = 0.;
    double TPS;
    int i = 0;

//    public static testP, testI, testD, testF;


    public void init() {

        drive.init(hardwareMap);
        pinpoint.init(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        servoTurret = hardwareMap.get(Servo.class, "servoWebcam");

        servoStop = hardwareMap.get(Servo.class, "servoStop");

        RPMIndicatorLeft = hardwareMap.get(Servo.class, "RPMIndicatorLeft");

        RPMIndicatorRight = hardwareMap.get(Servo.class, "RPMIndicatorRight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();

        servoTurret.setPosition(0.5);
        currentPos = servoTurret.getPosition();

        servoStop.setPosition(SERVO_STOP_CLOSE_POS);

        motorIntake = hardwareMap.dcMotor.get("motorIntake");
        motorTransfer = hardwareMap.dcMotor.get("motorTransfer");
        motorFlywheel = hardwareMap.get(DcMotorEx.class, "motorFlywheel");

        motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        motorTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorTransfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        servoPaddleLeft = hardwareMap.servo.get("servoPaddleLeft");

        battery = hardwareMap.get(VoltageSensor.class, "Control Hub");

        timer.reset();

        panelsTelemetry.debug("Init was ran!");
        panelsTelemetry.update(telemetry);


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

        if (targetFound) {
            error = bearing;
            currentPos = servoTurret.getPosition();

            angleToGoalDegrees = a1 + a2;
            angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

            distanceToGoalInches = (h2 - h1) / Math.tan(angleToGoalRadians);

            telemetry.addData("Distance To AprilTag", distanceToGoalInches);
            telemetry.addData("Bearing Error", error);
        } else {
            telemetry.addData("\n>", "Target Not Found\n");
            distanceToGoalInches = 36;
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
            targetRPM = 12.1 * distanceToGoalInches + 1725;
        } else {
            // manually set RPM distance
            if (gamepad2.x) {
                targetRPM = 0.;
            } else if (gamepad2.b) {
                targetRPM = 2400.;
            } else if (gamepad2.a) {
                targetRPM -= 50.;
            } else if (gamepad2.y) {
                targetRPM += 50.;
            }
        }

        // Calculate and set flywheel motor velocity
        TPS = targetRPM / 60. * CPR;
        motorFlywheel.setVelocity(TPS);


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
            intakeOn = false;
            transferOn = false;
        }
        lastDpadUp = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !lastDpadDown) {
            motorTransfer.setPower(0.);
            motorIntake.setPower(0.);
            motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
            motorTransfer.setDirection(DcMotorSimple.Direction.REVERSE);
            intakeOn = false;
            transferOn = false;
        }
        lastDpadDown = gamepad2.dpad_down;

        // Control Paddle Servo
        if (gamepad2.right_trigger > 0.25) {
            servoPaddleLeft.setPosition(SERVO_PADDLE_SHOOT_POS);
            motorTransfer.setPower(0.);
            transferOn = false;
        } else {
            servoPaddleLeft.setPosition(SERVO_PADDLE_DOWN_POS);
        }

        if (gamepad2.dpad_right && !lastDpadRight) {
            SERVO_PADDLE_SHOOT_POS -= 0.01;
        }
        lastDpadRight = gamepad2.dpad_right;

        if (gamepad2.dpad_left && !lastDpadLeft) {
            SERVO_PADDLE_DOWN_POS += 0.01;
        }
        lastDpadLeft = gamepad2.dpad_left;


        // Control Servo Stop
        if (gamepad2.left_trigger > 0.25) {
            servoStop.setPosition(SERVO_STOP_OPEN_POS);
        } else {
            servoStop.setPosition(SERVO_STOP_CLOSE_POS);
        }

        // Calculate Flywheel RPM
        if (i % 20 == 0) {
            currentTicks = motorFlywheel.getCurrentPosition();
            currentTime = timer.time();

            deltaRev = (currentTicks - previousTicks) / CPR;
            deltaTime = currentTime - previousTime;

            flywheelRPM = deltaRev / deltaTime * 60;

            previousTicks = currentTicks;
            previousTime = currentTime;
        }


        if (flywheelRPM < (targetRPM - 100)) { // turns the RGB lights blue if the flywheel speed is too low
            RPMIndicatorLeft.setPosition(0.611);
            RPMIndicatorRight.setPosition(0.611);
        } else if (flywheelRPM > (targetRPM + 250)) { // turns the RGB lights orange if the flywheel speed is too high
            RPMIndicatorLeft.setPosition(0.3);
            RPMIndicatorRight.setPosition(0.3);
        } else { // turns the RGB indicator green if the flywheel speed is correct
            RPMIndicatorLeft.setPosition(0.5);
            RPMIndicatorRight.setPosition(0.5);
        }

        i += 1;


        // Telemetry Data
        telemetry.addData("Timer", timer.seconds());
        telemetry.addData("Drive Power Factor", powerFactor);
        telemetry.addData("Intake Motor Power", motorIntake.getPower());
        telemetry.addData("Transfer Motor Power", motorTransfer.getPower());
        telemetry.addData("Stop Servo Position", servoStop.getPosition());
        telemetry.addData("Paddle Servo Position", servoPaddleLeft.getPosition());
        telemetry.addData("Voltage", battery.getVoltage());
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Flywheel RPM", flywheelRPM);
        telemetry.addData("Flywheel Motor Power", motorFlywheel.getPower());
        telemetry.addData("TPS", TPS);
        telemetry.addData("Flywheel Motor Velocity", motorFlywheel.getVelocity());

        telemetry.update();


        // Panels Telemetry Data
        panelsTelemetry.debug("Target RPM: $targetRPM");
        panelsTelemetry.debug("Flywheel RPM: $flywheelRPM");
        panelsTelemetry.addData("Target RPM", targetRPM);
        panelsTelemetry.addData("Flywheel RPM", flywheelRPM);

        panelsTelemetry.update(telemetry);

    }

    public void stop() {
        limelight.stop();
    }

}