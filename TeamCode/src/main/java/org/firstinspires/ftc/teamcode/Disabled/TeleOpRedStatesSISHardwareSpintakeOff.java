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
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
public class TeleOpRedStatesSISHardwareSpintakeOff extends OpMode {

    Pinpoint pinpoint = new Pinpoint();
    Shooter shooter = new Shooter();
    Flywheel flywheel = new Flywheel();
    Spintake spintake = new Spintake();
    Drive drive = new Drive();

    Pose2D pose2D;
    double a2 = 0.;

    double angleToGoalDegrees, angleToGoalRadians, distanceToGoalInches;

    private Limelight3A limelight;

    private DigitalChannel laserInput;

    private static ElapsedTime lasertimer = new ElapsedTime();

    double lasertime;

    boolean prevDetect = false;

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

    double prevX, prevY, prevRX;

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

            // Display detection state
            if (stateHigh) {
                telemetry.addLine("Object detected!");
            } else {
                telemetry.addLine("No object detected");
            }

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

        //when detecting an artifact
        if (stateHigh) {
            if (!activeDetecting) {
                lasertimer.reset();
                counter += 1;
            }
            if (activeDetecting) {
                if (lasertimer.seconds() > 0.25) {
                    intakeOn = false;
                    transferOn = false;
                    lasertimer.reset();
                }
            }
            telemetry.addLine("Object detected!");
        } else {        // not detecting
            if (activeDetecting) {
                lasertime = lasertimer.seconds();
            }
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

        if (x != prevX || y != prevY || rx != prevRX) {
            drive.moveRobotFC(y, x, rx, botHeading, powerFactor);
        }

        prevX = x;
        prevY = y;
        prevRX = rx;

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
            distanceToGoalInches = flywheel.distanceToGoalCalc(a2);

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
        flywheel.setFlywheelVel(targetRPM);

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
        panelsTelemetry.addData("Laser Detection Time", lasertime);
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