package org.firstinspires.ftc.teamcode.Disabled;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.Drive;
import org.firstinspires.ftc.teamcode.Hardware.Pinpoint;
import org.firstinspires.ftc.teamcode.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.Hardware.Spintake;

import java.util.List;

@Disabled
@TeleOp
public class TeleOpRedStatesSISHardwareRev4 extends OpMode {

    Pinpoint pinpoint = new Pinpoint();
    Shooter shooter = new Shooter();
    Spintake spintake = new Spintake();
    Drive drive = new Drive();

    Pose2D pose2D;

    public static double NEW_P = 100.;   // 10.
    public static double NEW_I = 1.;    // 3.
    public static double NEW_D = 20.;    // 0.
    public static double NEW_F = 3.5;    // 0.

    double h1 = 13.3;  // 13.25 // 14.25
    double h2 = 29.5;
    double a1 = 11.5; // 11.0
    double a2 = 0.;
    double x1 = -1.0;    // Distance between camera and ramp

    double angleToGoalDegrees, angleToGoalRadians, distanceToGoalInches;

    private Limelight3A limelight;

    private DigitalChannel laserInput;

    private Servo RPMIndicatorLeft, RPMIndicatorRight;

    private Servo artifactIndicator3, artifactIndicator2, artifactIndicator1;

    public DigitalChannel ledTransferGreen, ledTransferRed;

    private static final int DESIRED_TAG_ID = 24; // Red = 24; Blue = 20;

    double LONG_DIST_ANGLE_CORRECTION = 4; // Red = 4; Blue = -4;

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

    public DcMotorEx motorFlywheel;

    public boolean lastRightBump, lastLeftBump;
    public boolean lastDpadUp, lastDpadDown;
    public boolean lastDpadLeft, lastDpadRight;

    boolean autoRPM = true;
    boolean fieldCentric = true;
    boolean y1AlreadyPressed;

    public boolean intakeOn, transferOn;

    public boolean prevIntake, prevTransfer;

    boolean paddleOn;
    boolean stopOn;

    boolean prevPaddle;
    boolean prevStop;

    boolean aAlreadyPressed;
    boolean yAlreadyPressed;

    boolean activeDetecting = false;
    boolean stateHigh;
    int counter = 0;
    boolean stopAlreadyEngaged = false;

    double CPR = 28.;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;
    double targetRPM = 0.;
    double flywheelRPM = 0.;
    double TPS;

    PIDFCoefficients pidfModified;

//    public static double INTAKE_POWER = 0.8;
//    public static double INTAKE_CURRENT_ALERT = 6.;
//
//    public static double TRANSFER_POWER = 0.8;
//    public static double TRANSFER_CURRENT_ALERT = 6.;
//
//    public static double FLYWHEEL_CURRENT_ALERT = 9.;

    double flywheelCurrent, totalCurrent;

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private static ElapsedTime timer = new ElapsedTime();
    double currentTime, prevTime, elapsedTime;

    int i = 0;


//    private VoltageSensor battery;


    public void init() {

        drive.init(hardwareMap);
        pinpoint.init(hardwareMap);
        shooter.init(hardwareMap);
        spintake.init(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        laserInput = hardwareMap.get(DigitalChannel.class, "laserDigitalInput");

        RPMIndicatorLeft = hardwareMap.get(Servo.class, "RPMIndicatorLeft");

        RPMIndicatorRight = hardwareMap.get(Servo.class, "RPMIndicatorRight");

        artifactIndicator3 = hardwareMap.get(Servo.class, "artifactIndicator3");
        artifactIndicator2 = hardwareMap.get(Servo.class, "artifactIndicator2");
        artifactIndicator1 = hardwareMap.get(Servo.class, "artifactIndicator1");


        ledTransferGreen = hardwareMap.get(DigitalChannel.class, "ledTransferGreen");
        ledTransferRed = hardwareMap.get(DigitalChannel.class, "ledTransferRed");

        ledTransferGreen.setMode(DigitalChannel.Mode.OUTPUT);
        ledTransferRed.setMode(DigitalChannel.Mode.OUTPUT);

        laserInput.setMode(DigitalChannel.Mode.INPUT);

        limelight.pipelineSwitch(0);

        limelight.start();

        shooter.centerServoTurret();
        currentPos = shooter.servoTurretGetPosition();

        shooter.closeServoStop();
        shooter.downServoPaddle();

        motorFlywheel = hardwareMap.get(DcMotorEx.class, "motorFlywheel");

        motorFlywheel.setDirection(DcMotorEx.Direction.FORWARD);

        motorFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

            // Display detection state
            if (stateHigh) {
                telemetry.addLine("Object detected!");
            } else {
                telemetry.addLine("No object detected");
            }

        // Set Flywheel Motor PIDF coefficients
        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        motorFlywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

        //motorFlywheel.setCurrentAlert(FLYWHEEL_CURRENT_ALERT, CurrentUnit.AMPS);

//        battery = hardwareMap.get(VoltageSensor.class, "Control Hub");

        panelsTelemetry.debug("Init was ran!");
        panelsTelemetry.update(telemetry);


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

    }

    public void start() {
        timer.reset();
        prevTime = 0.;
    }

    public void loop() {

        stateHigh = laserInput.getState();

        /* detected true -->
        counter +1
        detecting false -->
        detected false
        detecting true */

        if (stateHigh) {
            if (!activeDetecting) {
                counter += 1;
            };
            telemetry.addLine("Object detected!");
        } else {
            telemetry.addLine("No object detected");
        }

        activeDetecting = stateHigh;


        // Driver Controls
        double y = drive.squareInputWithSign(-gamepad1.left_stick_y);
        double x = drive.squareInputWithSign(gamepad1.left_stick_x * 1.1);
        double rx = drive.squareInputWithSign(gamepad1.right_stick_x);

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


            angleToGoalDegrees = a1 + a2;
            angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

            distanceToGoalInches = ((h2 - h1) / Math.tan(angleToGoalRadians)) + x1;

            if (distanceToGoalInches < 80.) {
                error = bearing;
            } else {
                error = bearing + LONG_DIST_ANGLE_CORRECTION;
            }

            telemetry.addData("Distance To AprilTag", distanceToGoalInches);
            telemetry.addData("Bearing Error", error);
        } else {
            error = 0;
            distanceToGoalInches = 54.;

            telemetry.addData("Target", "Not Found\n");
        }

        // Toggle turret auto tracking when B is pressed on gamepad 1
        if (gamepad1.b && !lastBPress) {
            turretTracking = !turretTracking;
        }

        lastBPress = gamepad1.b;

        if (turretTracking) {
            currentPos = shooter.servoTurretGetPosition();
            if (Math.abs(error) > 1.0) {
                newPos = currentPos + error * 0.0016; // 0.0004
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
            if (distanceToGoalInches < 80. || distanceToGoalInches > 100) {
                targetRPM = 11.7 * distanceToGoalInches + 1743;     // targetRPM = 12.1 * distanceToGoalInches + 1725;
            }
            else {
                targetRPM = 2400;
            }
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

        // Control Direction of Intake and Transfer Motors
        if (gamepad2.dpad_up && !lastDpadUp) {
            spintake.forwardSpintakes();
            spintake.turnIntakeOff();
            spintake.turnTransferOff();
            intakeOn = false;
            transferOn = false;
        }
        lastDpadUp = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !lastDpadDown) {
            spintake.reverseSpintakes();
            spintake.turnIntakeOff();
            spintake.turnTransferOff();
            intakeOn = false;
            transferOn = false;
        }
        lastDpadDown = gamepad2.dpad_down;

        // Toggle intake when right_bumper is pressed
        if (gamepad2.right_bumper && !lastRightBump) {
            intakeOn = !intakeOn;
        }

        lastRightBump = gamepad2.right_bumper;

        if (intakeOn != prevIntake) {
            if (intakeOn) {
                spintake.turnIntakeOn();
            } else {
                spintake.turnIntakeOff();
            }
        }

        prevIntake = intakeOn;

        // Toggle transfer when left_bumper is pressed
        if (gamepad2.left_bumper && !lastLeftBump) {
            transferOn = !transferOn;
        }

        lastLeftBump = gamepad2.left_bumper;

        if (transferOn != prevTransfer) {
            if (transferOn) {
                spintake.turnTransferOn();
                spintake.setTransferLEDOn();
            } else {
                spintake.turnTransferOff();
                spintake.setTransferLEDOff();
            }
        }

        prevTransfer = transferOn;
        
        // Control Paddle Servo
        if(gamepad2.right_trigger > 0.25) {
            paddleOn = true;
        }
        else {
            paddleOn = false;
        }

        if (paddleOn != prevPaddle) {
            if (paddleOn) {
                shooter.shootServoPaddle();
                counter = 0;
            } else {
                shooter.downServoPaddle();
            }
        }

        prevPaddle = paddleOn;

        // Control Servo Stop
        if (gamepad2.left_trigger > 0.25) {
            stopOn = true;
        }
        else {
            stopOn = false;
        }

        if (stopOn != prevStop) {
            if (stopOn) {
                shooter.openServoStop();
                if (!stopAlreadyEngaged) {
                    counter -= 1;
                }
                stopAlreadyEngaged = true;
            } else {
                shooter.closeServoStop();
                stopAlreadyEngaged = false;
            }
        }

        prevStop = stopOn;


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

        // indicates when we have 3 artifacts

        if (counter >= 3) {
            artifactIndicator3.setPosition(0.555);
        } else {
            artifactIndicator3.setPosition(0.);
        }

        if (counter >= 2) {
            artifactIndicator2.setPosition(0.555);
        } else {
            artifactIndicator2.setPosition(0.);
        }

        if (counter >= 1) {
            artifactIndicator1.setPosition(0.555);
        }else {
            artifactIndicator1.setPosition(0.);
        }

        // Intake Motor Current
//        spintake.checkIntakeCurrent();

        // Transfer Motor Current
//        spintake.checkTransferCurrent();

        // Flywheel Motor Current
//        flywheelCurrent = motorFlywheel.getCurrent(CurrentUnit.AMPS);
//
//        if (motorFlywheel.isOverCurrent()) {
//            motorFlywheel.setVelocity(0.);
//        }

        i += 1;

        if (i % 100 == 0) {
            currentTime = timer.seconds();
            elapsedTime = currentTime - prevTime;
            prevTime = currentTime;
        }

//        totalCurrent = intakeCurrent + transferCurrent + flywheelCurrent + frontLeftMotor.getCurrent(CurrentUnit.AMPS) + backLeftMotor.getCurrent(CurrentUnit.AMPS) + frontRightMotor.getCurrent(CurrentUnit.AMPS) + backRightMotor.getCurrent(CurrentUnit.AMPS);

        // Panels Telemetry Data
        panelsTelemetry.addData("Timer", timer.seconds());
        panelsTelemetry.addData("Elapsed Time (100 loops)", elapsedTime);
        panelsTelemetry.addData("Target RPM", targetRPM);
        panelsTelemetry.addData("Flywheel RPM", flywheelRPM);
        panelsTelemetry.addData("Field Centric", fieldCentric);
        panelsTelemetry.addData("Drive Power Factor", powerFactor);
        panelsTelemetry.addData("Auto Turret", turretTracking);
        panelsTelemetry.addData("Auto RPM", autoRPM);
        panelsTelemetry.addData("Intake On", intakeOn);
//        panelsTelemetry.addData("Transfer On", transferOn);
//        panelsTelemetry.addData("Stop Servo Position", shooter.servoStopPosition());
//        panelsTelemetry.addData("Paddle Servo Position", shooter.servoPaddlePosition());
        panelsTelemetry.update(telemetry);
//        panelsTelemetry.addData("Intake Current", intakeCurrent);
//        panelsTelemetry.addData("Transfer Current", transferCurrent);
//        panelsTelemetry.addData("Flywheel Current", flywheelCurrent);
//        panelsTelemetry.addData("FL Current", frontLeftMotor.getCurrent(CurrentUnit.AMPS));
//        panelsTelemetry.addData("FR Current", frontRightMotor.getCurrent(CurrentUnit.AMPS));
//        panelsTelemetry.addData("RL Current", backLeftMotor.getCurrent(CurrentUnit.AMPS));
//        panelsTelemetry.addData("RR Current", backRightMotor.getCurrent(CurrentUnit.AMPS));
//        panelsTelemetry.addData("Total Current", totalCurrent);
//        panelsTelemetry.addData("Voltage", battery.getVoltage());

    }

    public void stop() {
        limelight.stop();
    }

}