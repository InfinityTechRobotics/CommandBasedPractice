package org.firstinspires.ftc.teamcode.Practice;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.Drive;
import org.firstinspires.ftc.teamcode.Hardware.Flywheel;
import org.firstinspires.ftc.teamcode.Hardware.Pinpoint;
import org.firstinspires.ftc.teamcode.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.Hardware.Spintake;

import java.util.List;

@Configurable
@TeleOp
public class TeleOpRedWorldsTestmk2 extends OpMode {

    Pinpoint pinpoint = new Pinpoint();
    Shooter shooter = new Shooter();
    Flywheel flywheel = new Flywheel();
    Spintake spintake = new Spintake();
    Drive drive = new Drive();

    Pose2D pose2D;
    double a2 = 0.;

    double distanceToGoalInches;

    private Limelight3A limelight;

    private DigitalChannel laserInput;

    private static ElapsedTime laserTimer = new ElapsedTime();

    double laserTime;

    private static final int DESIRED_TAG_ID = 24; // Red = 24; Blue = 20;

    double LONG_DIST_ANGLE_CORRECTION = 4; // Red = 4; Blue = -4;

    // Turret variables
    double error, currentPos, newPos;

    double prevError;
    double turretTimer;

    double bearing;

    double aprilTagTimer;

    public static double TURRET_TRACKING_TIMER_THRESHOLD = 1.0;

    public static double SERVO_TURRET_PROPORTIONAL_TERM = 0.0016;

    public static double SERVO_TURRET_DERIVATIVE_TERM = 0.0;

    double botHeading;

    boolean targetFound = false;

    boolean turretTracking = true;

    double DRIVE_POWER_FACTOR = 0.95;
    double DRIVE_POWER_FACTOR_LOW = 0.6;
    double DRIVE_POWER_FACTOR_HIGH = 1;

    double powerFactor = DRIVE_POWER_FACTOR;

    double prevX, prevY, prevRX;

    boolean autoRPM = true;
    boolean fieldCentric = true;

    public boolean intakeOn, transferOn;

    public boolean prevIntake, prevTransfer;

    public static double SPINTAKE_AUTO_SHUTOFF_THRESHOLD = 0.25;

    boolean paddleOn;
    boolean stopOn;

    boolean prevPaddle;
    boolean prevStop;

    boolean activeDetecting = false;
    boolean stateHigh;

    int counter = 0;
    int prevCount = 0;

    boolean stopAlreadyEngaged = false;

    double targetRPM = 0.;
    double flywheelRPM = 0.;

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private static final ElapsedTime timer = new ElapsedTime();
    double currentTime, prevTime, elapsedTime;

    double prevTime1000, elapsedTime1000;

    int i = 0;

    public void init() {

        drive.init(hardwareMap);
        pinpoint.init(hardwareMap);
        shooter.init(hardwareMap);
        flywheel.init(hardwareMap);
        spintake.init(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        laserInput = hardwareMap.get(DigitalChannel.class, "laserDigitalInput");

        laserInput.setMode(DigitalChannel.Mode.INPUT);

        limelight.pipelineSwitch(0);

        limelight.start();

        shooter.centerServoTurret();
        currentPos = shooter.servoTurretGetPosition();

        shooter.closeServoStop();
        shooter.downServoPaddle();

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
        turretTimer = timer.seconds();
        aprilTagTimer = timer.seconds();
    }

    public void loop() {

        // Laser Artifact Detection (Detected = TRUE --> counter +1)
        stateHigh = laserInput.getState();

        if (stateHigh) {
            if (!activeDetecting) {
                laserTimer.reset();
                counter += 1;
            }
            if (activeDetecting) {
                if (laserTimer.seconds() > SPINTAKE_AUTO_SHUTOFF_THRESHOLD) {
                    intakeOn = false;
                    transferOn = false;
                    laserTimer.reset();
                }
            }
        } else {        // not detecting
            if (activeDetecting) {
                laserTime = laserTimer.seconds();
            }
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

        if (gamepad1.yWasPressed()) {
            fieldCentric = !fieldCentric;
        }

        pose2D = pinpoint.getPinpointPose();

        if (fieldCentric) {
            botHeading = pose2D.getHeading(AngleUnit.RADIANS);
        }
        else {
            botHeading = 0;
        }

        if (x != prevX || y != prevY || rx != prevRX) {
            drive.moveRobotFC(y, x, rx, botHeading, powerFactor);
        }

        prevX = x;
        prevY = y;
        prevRX = rx;

        LLResult result = limelight.getLatestResult();
        targetFound = false;

        if (result.isValid()) {
            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Limelight", "ID: %d, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                if (fr.getFiducialId() == DESIRED_TAG_ID) {
                    bearing = fr.getTargetXDegrees();
                    a2 = fr.getTargetYDegrees();
                    targetFound = true;
                    aprilTagTimer = timer.seconds();
                    break;
                }
            }
        } else {
            telemetry.addData("Limelight", "No data available");
        }

        if (targetFound) {
            distanceToGoalInches = flywheel.distanceToGoalCalc(a2);
            if (distanceToGoalInches < 80.) {
                error = bearing;
            } else {
                error = bearing + LONG_DIST_ANGLE_CORRECTION;
            }
        } else {
            error = 0;
            distanceToGoalInches = 54.;
        }

        turretTimer = timer.seconds() - turretTimer;

        // Toggle turret auto tracking when B is pressed on gamepad 1
        if (gamepad1.bWasPressed()) {
            turretTracking = !turretTracking;
        }

        if (turretTracking) {
            if ((timer.seconds() - aprilTagTimer < TURRET_TRACKING_TIMER_THRESHOLD)) {
                currentPos = shooter.servoTurretGetPosition();
                newPos = shooter.newTurretPDCalc(currentPos, error, prevError, turretTimer, SERVO_TURRET_PROPORTIONAL_TERM, SERVO_TURRET_DERIVATIVE_TERM);
                shooter.servoTurretSetPosition(newPos);
            } else {
                shooter.centerServoTurret();
            }
        } else {
            shooter.centerServoTurret();
        }

        prevError = error;

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
                targetRPM = flywheel.targetRPMCalc(distanceToGoalInches);
            }
            else {
                targetRPM = 2400;
            }
        } else {
            // manually set RPM distance
            if (gamepad2.x) {
                targetRPM = 0.;
            } else if (gamepad2.b) {
                targetRPM = 2400.;
            } else if (gamepad2.aWasPressed()) {
                targetRPM -= 50.;
            } else if (gamepad2.yWasPressed()) {
                targetRPM += 50.;
            }
        }

        // Calculate and set flywheel motor velocity
        flywheel.setFlywheelVel(targetRPM);

        // Control Direction of Intake and Transfer Motors
        if (gamepad2.dpadUpWasPressed()) {
            spintake.forwardSpintakes();
            spintake.turnIntakeOff();
            spintake.turnTransferOff();
            intakeOn = false;
            transferOn = false;
        }

        if (gamepad2.dpadDownWasPressed()) {
            spintake.reverseSpintakes();
            spintake.turnIntakeOff();
            spintake.turnTransferOff();
            intakeOn = false;
            transferOn = false;
        }

        // Toggle intake when right_bumper is pressed
        if (gamepad2.rightBumperWasPressed()) {
            intakeOn = !intakeOn;
        }

        if (intakeOn != prevIntake) {
            if (intakeOn) {
                spintake.turnIntakeOn();
            } else {
                spintake.turnIntakeOff();
            }
        }

        prevIntake = intakeOn;

        // Toggle transfer when left_bumper is pressed
        if (gamepad2.leftBumperWasPressed()) {
            transferOn = !transferOn;
        }

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
        paddleOn = (gamepad2.right_trigger > 0.25);

        if (counter != prevCount) {
            spintake.setArtifactIndicator(counter);
        }

        prevCount = counter;

        if (paddleOn != prevPaddle) {
            if (paddleOn) {
                shooter.shootServoPaddle();
                counter = 0;
            } else {
                shooter.downServoPaddle();
            }
        }

        prevPaddle = paddleOn;

        // Control Servo Stop and turn intake and transfer on
        if (gamepad2.left_trigger > 0.25) {
            stopOn = true;
            intakeOn = true;
            transferOn = true;
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


        //flywheelRPM = motorFlywheel.getVelocity() / CPR * 60;
        flywheelRPM = flywheel.getFlywheelVel();

        // RGB Indicator Lights
        flywheel.setFlywheelRGB(flywheelRPM, targetRPM);

        // indicates when we have 3 artifacts


        i += 1;

        if (i % 100 == 0) {
            currentTime = timer.seconds();
            elapsedTime = currentTime - prevTime;
            prevTime = currentTime;
        }

        if (i % 1000 == 0) {
            currentTime = timer.seconds();
            elapsedTime1000 = currentTime - prevTime1000;
            prevTime1000 = currentTime;
        }

        // Panels Telemetry Data
        panelsTelemetry.addData("Timer", timer.seconds());
        panelsTelemetry.addData("Elapsed Time (100 loops)", elapsedTime);
        panelsTelemetry.addData("Elapsed Time (1000 loops)", elapsedTime1000);
        panelsTelemetry.addData("Laser Detection Time", laserTime);
        panelsTelemetry.addData("Object Detected", stateHigh);
        panelsTelemetry.addData("Distance To AprilTag", distanceToGoalInches);
        panelsTelemetry.addData("Bearing Error", error);
        panelsTelemetry.addData("Target RPM", targetRPM);
        panelsTelemetry.addData("Flywheel RPM", flywheelRPM);
        panelsTelemetry.addData("Field Centric", fieldCentric);
        panelsTelemetry.addData("Drive Power Factor", powerFactor);
        panelsTelemetry.addData("Auto Turret", turretTracking);
        panelsTelemetry.addData("Auto RPM", autoRPM);
        panelsTelemetry.addData("Intake On", intakeOn);
        panelsTelemetry.addData("Intake Current", spintake.getIntakeMotorCurrent());
        panelsTelemetry.addData("Transfer Current", spintake.getTransferMotorCurrent());
        panelsTelemetry.addData("Flywheel Current", flywheel.getMotorFlywheelCurrent());
//        panelsTelemetry.addData("Transfer On", transferOn);
//        panelsTelemetry.addData("Stop Servo Position", shooter.servoStopPosition());
//        panelsTelemetry.addData("Paddle Servo Position", shooter.servoPaddlePosition());
        panelsTelemetry.update(telemetry);

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