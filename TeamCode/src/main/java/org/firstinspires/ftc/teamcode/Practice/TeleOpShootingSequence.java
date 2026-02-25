package org.firstinspires.ftc.teamcode.Practice;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.util.Timer;
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
//@Configurable
@TeleOp
public class TeleOpShootingSequence extends OpMode {

    Pinpoint pinpoint = new Pinpoint();
    Shooter shooter = new Shooter();
    Spintake spintake = new Spintake();
    Drive drive = new Drive();

    Pose2D pose2D;

    double START_DELAY_TIME = 2.;

    double shootingTime = 0.;

    private Timer pathTimer, opmodeTimer;
    private Timer shootTimer;
    private int pathState;

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

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        shootTimer = new Timer();

    }

    public void start() {
        timer.reset();
        prevTime = 0.;
        opmodeTimer.resetTimer();
    }

    public void loop() {
        targetRPM = 2400;

        autonomousPathUpdate();
//
//        // Calculate and set flywheel motor velocity
        TPS = targetRPM / 60. * CPR;
        motorFlywheel.setVelocity(TPS);

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
        if (gamepad2.right_trigger > 0.25) {
            paddleOn = true;
        } else {
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


//

//
//        // Control Servo Stop
//        if (gamepad2.left_trigger > 0.25) {
//            stopOn = true;
//        }
//        else {
//            stopOn = false;
//        }
//
//        if (stopOn != prevStop) {
//            if (stopOn) {
//                shooter.openServoStop();
//                if (!stopAlreadyEngaged) {
//                    counter -= 1;
//                }
//                stopAlreadyEngaged = true;
//            } else {
//                shooter.closeServoStop();
//                stopAlreadyEngaged = false;
//            }
//        }
//
//        prevStop = stopOn;
//
//
//        flywheelRPM = motorFlywheel.getVelocity() / CPR * 60;
//
//        // RGB Indicator Lights
//        if (flywheelRPM < (targetRPM - 150)) { // turns the RGB lights blue if the flywheel speed is too low
//            RPMIndicatorLeft.setPosition(0.611);
//            RPMIndicatorRight.setPosition(0.611);
//        } else if (flywheelRPM > (targetRPM + 150)) { // turns the RGB lights orange if the flywheel speed is too high
//            RPMIndicatorLeft.setPosition(0.3);
//            RPMIndicatorRight.setPosition(0.3);
//        } else { // turns the RGB indicator green if the flywheel speed is correct
//            RPMIndicatorLeft.setPosition(0.5);
//            RPMIndicatorRight.setPosition(0.5);
//        }
//
//        // indicates when we have 3 artifacts
//
//        if (counter >= 3) {
//            artifactIndicator3.setPosition(0.555);
//        } else {
//            artifactIndicator3.setPosition(0.);
//        }
//
//        if (counter >= 2) {
//            artifactIndicator2.setPosition(0.555);
//        } else {
//            artifactIndicator2.setPosition(0.);
//        }
//
//        if (counter >= 1) {
//            artifactIndicator1.setPosition(0.555);
//        }else {
//            artifactIndicator1.setPosition(0.);
//        }
//
//        // Intake Motor Current
////        spintake.checkIntakeCurrent();
//
//        // Transfer Motor Current
////        spintake.checkTransferCurrent();
//
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

        if (gamepad2.right_trigger > 0.25) {
            setPathState(10);
            }
//
////        totalCurrent = intakeCurrent + transferCurrent + flywheelCurrent + frontLeftMotor.getCurrent(CurrentUnit.AMPS) + backLeftMotor.getCurrent(CurrentUnit.AMPS) + frontRightMotor.getCurrent(CurrentUnit.AMPS) + backRightMotor.getCurrent(CurrentUnit.AMPS);

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
            panelsTelemetry.addData("Path State", pathState);
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

        public void stop () {
            limelight.stop();
        }

        public void setPathState (int pState){

            pathState = pState;
            pathTimer.resetTimer();

        }

        public void autonomousPathUpdate () {
            switch (pathState) {
                case 10:
                    shooter.closeServoStop();
                    shooter.downServoPaddle();
                    setPathState(1001);
                    break;
                case 1001:
                    spintake.turnIntakeOn();
                    spintake.turnTransferOn();
                    setPathState(10001);
                    break;
                case 10001:
                    if (pathTimer.getElapsedTimeSeconds() > 0.25) { // changed from 0.5 to 0.25
                        shootTimer.resetTimer();
                        shooter.openServoStop();
                        setPathState(10006);
                    }
                    break;
                case 10006:
                    if (pathTimer.getElapsedTimeSeconds() > 0.6) {
                        shooter.shootServoPaddle();
                        setPathState(10007);
                    }
                    break;
                case 10007:
                    if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                        spintake.turnIntakeOff();
                        shootingTime = shootTimer.getElapsedTimeSeconds();
                        shooter.downServoPaddle();
                        shooter.closeServoStop();
                        spintake.turnIntakeOff();
                        setPathState(10008);
                    }
                    break;
            }
        }


}
