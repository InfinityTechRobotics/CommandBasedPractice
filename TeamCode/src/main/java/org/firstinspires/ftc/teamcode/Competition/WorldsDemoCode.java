package org.firstinspires.ftc.teamcode.Competition;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.FlywheelSpinfinityDuo;
import org.firstinspires.ftc.teamcode.Hardware.ShooterSpinfinityDuo;
import org.firstinspires.ftc.teamcode.Hardware.SpintakeSpinfinity;

import java.util.List;

@TeleOp
public class WorldsDemoCode extends OpMode {

    ShooterSpinfinityDuo shooter = new ShooterSpinfinityDuo();
    FlywheelSpinfinityDuo flywheel = new FlywheelSpinfinityDuo();
    SpintakeSpinfinity spintake = new SpintakeSpinfinity();

    double a2 = 0.;

    double distanceToGoalInches;

    private Limelight3A limelight;

    private DigitalChannel laserInput;

    private static ElapsedTime laserTimer = new ElapsedTime();

    double laserTime;

    public static int DESIRED_TAG_ID_BLUE = 20; // Red = 24; Blue = 20;
    public static int DESIRED_TAG_ID_RED = 24; // Red = 24; Blue = 20;

    public double FALLBACK_APRILTAG_DISTANCE = 54.;

    // Turret variables
    double error;

    int currentPos, newPos, newPosePos;

    double bearing;

    double aprilTagTimer;

    public static double TURRET_TRACKING_TIMER_THRESHOLD = .25;

    // Pinpoint Robot Positions
    public static double robotXPos, robotYPos, botHeading;

    boolean targetFound = false;

    boolean turretTracking = true;

    boolean autoRPM = false;

    public boolean intakeOn;

    public boolean prevIntake;

    public static double SPINTAKE_AUTO_SHUTOFF_THRESHOLD = 0.4; //0.25

    boolean activeDetecting = false;
    boolean stateHigh;

    int counter = 0;
    int prevCount = 0;

    double targetRPM = 0.;
    double flywheelRPM = 0.;

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private static final ElapsedTime timer = new ElapsedTime();

    int i = 0;

    private Timer pathTimer;

    private int pathState;

    double FAR_ZONE_HOOD_POS;

    boolean rightTriggerPressed;
    boolean leftTriggerPressed;
    boolean prevRightTrigger = false;
    boolean prevLeftTrigger = false;

    public void init() {

        shooter.init(hardwareMap);
        flywheel.init(hardwareMap);
        spintake.init(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();

        laserInput = hardwareMap.get(DigitalChannel.class, "laserDigitalInput");

        laserInput.setMode(DigitalChannel.Mode.INPUT);

        shooter.centerMotorTurret();
        currentPos = shooter.motorTurretGetPosition();

        shooter.closeServoStop();
        shooter.downServoPaddle();

        panelsTelemetry.debug("Init was ran!");
        panelsTelemetry.update(telemetry);

        shooter.setServoHoodDownPos();

        pathTimer = new Timer();

    }

    public void start() {
        timer.reset();
        aprilTagTimer = timer.seconds();
    }

    public void loop() {

        if (gamepad2.dpadLeftWasPressed()) {
            FAR_ZONE_HOOD_POS = 0.5;
        } else if (gamepad2.leftBumperWasPressed()) {
            FAR_ZONE_HOOD_POS = 0.4;
        } else if (gamepad2.dpadRightWasPressed()) {
            FAR_ZONE_HOOD_POS = 0.2;;
        }

        autonomousPathUpdate();

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
                    laserTimer.reset();
                }
            }
        } else {        // not detecting
            if (activeDetecting) {
                laserTime = laserTimer.seconds();
            }
        }

        activeDetecting = stateHigh;

        LLResult result = limelight.getLatestResult();
        targetFound = false;

        if (result != null && result.isValid()) {
            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Limelight", "ID: %d, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                if (fr.getFiducialId() == DESIRED_TAG_ID_BLUE || fr.getFiducialId() == DESIRED_TAG_ID_RED) {
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
            error = -bearing;
        } else {
            distanceToGoalInches = FALLBACK_APRILTAG_DISTANCE;
            error = 0;
        }

        // Toggle turret auto tracking when B is pressed on gamepad 1
        if (gamepad1.bWasPressed()) {
            turretTracking = !turretTracking;
        }

        // Rotate turret based on limelight reading or pose
        if (turretTracking) {
            currentPos = shooter.motorTurretGetPosition();
            newPos = shooter.newTurretPositionClampedCalc(currentPos, error);
            shooter.motorTurretSetPosition(newPos);
        } else {
            shooter.centerMotorTurret();
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
            targetRPM = flywheel.targetRPMCalc(distanceToGoalInches);
            shooter.setServoHoodManual(FAR_ZONE_HOOD_POS);
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
            intakeOn = false;
        }

        if (gamepad2.dpadDownWasPressed()) {
            spintake.reverseSpintakes();
            spintake.turnIntakeOff();
            intakeOn = false;
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

        //start shooting sequence
        rightTriggerPressed = gamepad2.right_trigger > 0.25;
        leftTriggerPressed  = gamepad2.left_trigger > 0.25;

        if (rightTriggerPressed && !prevRightTrigger) {
            counter = 0;
            laserTimer.reset();
            setPathState(10);

        }

        if (leftTriggerPressed && !prevLeftTrigger) {
            counter = 0;
            laserTimer.reset();
            setPathState(10100); //10100
        }

        prevRightTrigger = rightTriggerPressed;
        prevLeftTrigger = leftTriggerPressed;


        if (counter != prevCount) {
            spintake.setArtifactIndicator(counter);
        }

        prevCount = counter;

        //flywheelRPM = motorFlywheel.getVelocity() / CPR * 60;
        flywheelRPM = flywheel.getFlywheelVel();

        // RGB Indicator Lights
        // indicates when we have 3 artifacts
        flywheel.setFlywheelRGB(flywheelRPM, targetRPM);


        // Panels Telemetry Data
        panelsTelemetry.addData("Auto Turret", turretTracking);
        panelsTelemetry.addData("Auto RPM", autoRPM);
        panelsTelemetry.addData("Distance To AprilTag", distanceToGoalInches);
        panelsTelemetry.addData("Target RPM", targetRPM);
        panelsTelemetry.addData("Flywheel RPM", flywheelRPM);
        panelsTelemetry.update(telemetry);

    }

    public void stop() {
        shooter.centerMotorTurret();
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
                setPathState(10001);
                break;
            case 10001:
                if (pathTimer.getElapsedTimeSeconds() > 0.01) { // changed from 0.5 to 0.25
                    shooter.openServoStop();
                    counter = 0;
                    setPathState(10006);
                }
                break;
            case 10006:
                if (pathTimer.getElapsedTimeSeconds() > 0.2){
                    gamepad1.rumble(0.5, 0.5, 200);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.6) {
                    shooter.shootServoPaddle();
                    setPathState(10007);
                }
                break;
            case 10007:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    shooter.downServoPaddle();
                    shooter.closeServoStop();
                    setPathState(999);
                }
                break;
            case 10100:
                shooter.openServoStop();
                if (pathTimer.getElapsedTimeSeconds() > 0.05) {
                    setPathState(10101);
                }
                break;
            case 10101:
                shooter.shootServoPaddle();
                setPathState(10102);
                break;
            case 10102:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    shooter.downServoPaddle();
                    shooter.closeServoStop();
                    setPathState(999);
                }
                break;
        }
    }
}