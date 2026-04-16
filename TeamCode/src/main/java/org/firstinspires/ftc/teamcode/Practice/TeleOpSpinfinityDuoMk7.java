package org.firstinspires.ftc.teamcode.Practice;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.FlywheelSpinfinityDuo;
import org.firstinspires.ftc.teamcode.Hardware.Pinpoint;
import org.firstinspires.ftc.teamcode.Hardware.ShooterSpinfinityDuo;
import org.firstinspires.ftc.teamcode.Hardware.SpintakeSpinfinity;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.function.Supplier;

@Configurable
@TeleOp
public class TeleOpSpinfinityDuoMk7 extends OpMode {

    Pinpoint pinpoint = new Pinpoint();
    ShooterSpinfinityDuo shooter = new ShooterSpinfinityDuo();
    FlywheelSpinfinityDuo flywheel = new FlywheelSpinfinityDuo();
    SpintakeSpinfinity spintake = new SpintakeSpinfinity();
//    Drive drive = new Drive();

    Pose2D pose2D;
    double a2 = 0.;

    double distanceToGoalInches;

    private Limelight3A limelight;

    private DigitalChannel laserInput;

    private static ElapsedTime laserTimer = new ElapsedTime();

    double laserTime;

    private static final int DESIRED_TAG_ID = 24; // Red = 24; Blue = 20;

    double LONG_DIST_ANGLE_CORRECTION = 2; // Red = 4; Blue = -4;

    public Servo servoHood;

    // Turret variables
    double error;

    int currentPos, newPos, newPosePos;

    double prevError;
    double turretTimer;

    double bearing;

    double aprilTagTimer;

    public static double TURRET_TRACKING_TIMER_THRESHOLD = .25;

    public static double MOTOR_TURRET_PROPORTIONAL_TERM = 5;

    public static double MOTOR_TURRET_DERIVATIVE_TERM = 0.; //0.0

    // Pinpoint Robot Positions
    public static double robotXPos, robotYPos, botHeading;

    //MegaTag Robot Positions
    public static double MT1XPos, MT1YPos, MT1botHeading;

    double robotToGoalRelativeAngle;

    boolean targetFound = false;

    boolean turretTracking = true;

    double DRIVE_POWER_FACTOR = 0.95;
    double DRIVE_POWER_FACTOR_LOW = 0.6;
    double DRIVE_POWER_FACTOR_HIGH = 1;

    double powerFactor = DRIVE_POWER_FACTOR;

    boolean autoRPM = true;
    boolean robotCentric = false;

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
    double currentTime, prevTime, elapsedTime;

    double prevTime1000, elapsedTime1000;

    int i = 0;

    double shootingTime = 0.;
    private Timer pathTimer, opmodeTimer;

    private Timer shootTimer;

    private int pathState;

    private Follower follower;
    public static Pose startingPose; // Start Pose of our robot.

    private Supplier<PathChain> pathChain;

    private boolean automatedDrive;

    double robotHeading;

    boolean endgameRumbleFlag;
    boolean parkRumbleFlag;

    public void init() {

        servoHood = hardwareMap.get(Servo.class, "servoHood");

        servoHood.setPosition(0.1);

//        drive.init(hardwareMap);
        pinpoint.init(hardwareMap);
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

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        shootTimer = new Timer();

        startingPose = new Pose(91, 108, Math.toRadians(30));
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(90, 102))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(37.5), 0.8))
                .build();

    }

    public void start() {
        timer.reset();
        prevTime = 0.;
        turretTimer = timer.seconds();
        aprilTagTimer = timer.seconds();

        follower.startTeleopDrive();
    }

    public void loop() {

        if (gamepad2.dpadLeftWasPressed()) {
            servoHood.setPosition(0.5);
        } else if (gamepad2.leftBumperWasPressed()) {
            servoHood.setPosition(0.3);
        } else if (gamepad2.dpadRightWasPressed()) {
            servoHood.setPosition(0.1);

        }

        autonomousPathUpdate();

        follower.update();

        if (timer.seconds() > 98 && !endgameRumbleFlag) {
            gamepad1.rumble(1, 1,200);
            endgameRumbleFlag = true;
        }
        if (timer.seconds() > 110 && !parkRumbleFlag) {
            gamepad1.rumble(1, 1,500);
            parkRumbleFlag = true;
        }

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

        // Driver Controls
//        double y = drive.squareInputWithSign(-gamepad1.left_stick_y);
//        double x = drive.squareInputWithSign(gamepad1.left_stick_x * 1.1);
//        double rx = drive.squareInputWithSign(gamepad1.right_stick_x);

        if (gamepad1.aWasPressed()) {
            follower.setPose(new Pose(72, 72, Math.toRadians(0)));
            //pinpoint.pinpointReset();
        }

        if (gamepad1.left_bumper) {
            powerFactor = DRIVE_POWER_FACTOR_LOW;
        } else if (gamepad1.right_bumper) {
            powerFactor = DRIVE_POWER_FACTOR_HIGH;
        } else {
            powerFactor = DRIVE_POWER_FACTOR;
        }

        if (gamepad1.yWasPressed()) {
            robotCentric = !robotCentric;
        }


        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * powerFactor,
                    -gamepad1.left_stick_x * powerFactor,
                    -gamepad1.right_stick_x * powerFactor,
                    robotCentric // Field Centric
            );
        }

        //Automated PathFollowing
        if (gamepad1.rightBumperWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }
        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.leftBumperWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

//        pose2D = pinpoint.getPinpointPose();

//        if (robotCentric) {
//            botHeading = pose2D.getHeading(AngleUnit.RADIANS);
//        }
//        else {
//            botHeading = 0;
//        }

//        if (x != prevX || y != prevY || rx != prevRX) {
//            drive.moveRobotFC(y, x, rx, botHeading, powerFactor);
//        }
//
//        prevX = x;
//        prevY = y;
//        prevRX = rx;

        //

//        robotHeading = follower.getHeading();

//        botHeading = pose2D.getHeading(AngleUnit.RADIANS);



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

        // MegaTag Robot Position Determination
//        if (result != null && result.isValid()) {
//            Pose3D botpose = result.getBotpose();
//            if (botpose != null) {
//                double xpos = botpose.getPosition().x;
//                double ypos = botpose.getPosition().y;
//                double headingpos = botpose.getOrientation().getYaw();
//                telemetry.addData("MT1 Location", "(" + xpos + ", " + ypos + ", " + headingpos + ")");
//
//                //follower.setPose(new Pose(xpos, ypos, headingpos, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE));
//                //Pose convertedPos = new Pose(xpos, ypos, headingpos, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
//                //telemetry.addData("Converted Location", "(" + convertedPos.getX() + ", " + convertedPos.getY() + ", " + convertedPos.getHeading() + ")");
//                MT1XPos = 72 + (ypos * 39.37);
//                MT1YPos = 72 - (xpos * 39.37);
//                MT1botHeading = headingpos - 90;
//            }
//        }

        if (targetFound) {
            distanceToGoalInches = flywheel.distanceToGoalCalc(a2);
            if (distanceToGoalInches < 80.) {
                error = -bearing;
            } else {
                error = -bearing - LONG_DIST_ANGLE_CORRECTION;
            }
        } else {
            error = 0;
            distanceToGoalInches = 54.;
        }

        // Calculate robot angle to AprilTag
        robotXPos = follower.getPose().getX();
        robotYPos = follower.getPose().getY();
        botHeading = follower.getHeading();
        robotToGoalRelativeAngle = shooter.newTurretPoseCalc(robotXPos, robotYPos, botHeading);

        turretTimer = timer.seconds() - turretTimer;

        // Toggle turret auto tracking when B is pressed on gamepad 1
        if (gamepad1.bWasPressed()) {
            turretTracking = !turretTracking;
        }

        // Rotate turret based on limelight reading or pose
        if (turretTracking) {
            if ((timer.seconds() - aprilTagTimer < TURRET_TRACKING_TIMER_THRESHOLD)) {
                currentPos = shooter.motorTurretGetPosition();
                newPos = shooter.newTurretPDCalc(currentPos, error, prevError, turretTimer, MOTOR_TURRET_PROPORTIONAL_TERM, MOTOR_TURRET_DERIVATIVE_TERM);
                shooter.motorTurretSetPosition(newPos);
            } else {
                newPosePos = shooter.turretPosEncoderCalc(robotToGoalRelativeAngle);
                shooter.motorTurretSetPosition(newPosePos);
//                shooter.centerMotorTurret();
            }
        } else {
            shooter.centerMotorTurret();
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
//            if (distanceToGoalInches < 80. || distanceToGoalInches > 100) {
            if (distanceToGoalInches > 85) {
                targetRPM = 3325;
            } else {
                targetRPM = flywheel.targetRPMCalc(distanceToGoalInches);
            }
//            }
//            else {
//                targetRPM = 2400;
//            }
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
//        flywheel.setFlywheelVel(0);

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
        if (gamepad2.right_trigger > 0.25) {
            setPathState(10);
            counter = 0;
        }

        if (counter != prevCount) {
            spintake.setArtifactIndicator(counter);
        }

        prevCount = counter;

        //start sequence for shooting paddle
        if (gamepad2.left_trigger > 0.25) {
            setPathState(10100);
            counter = 0;
        }

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
        panelsTelemetry.addData("Auto Turret", turretTracking);
        panelsTelemetry.addData("Auto RPM", autoRPM);
        panelsTelemetry.addData("Distance To AprilTag", distanceToGoalInches);
        panelsTelemetry.addData("Target RPM", targetRPM);
        panelsTelemetry.addData("Flywheel RPM", flywheelRPM);
        panelsTelemetry.addData("Robot Centric", robotCentric);
        //        panelsTelemetry.addData("Timer", timer.seconds());
//        panelsTelemetry.addData("Pinpoint Robot X Position", robotXPos);
//        panelsTelemetry.addData("Pinpoint Robot Y Position", robotYPos);
//        panelsTelemetry.addData("Pinpoint Robot Heading", Math.toDegrees(botHeading));
//        panelsTelemetry.addData("MegaTag Robot X Position", MT1XPos);
//        panelsTelemetry.addData("MegaTag Robot Y Position", MT1YPos);
//        panelsTelemetry.addData("MegaTag Robot Heading", MT1botHeading);
//        panelsTelemetry.addData("Elapsed Time (100 loops)", elapsedTime);
//        panelsTelemetry.addData("Elapsed Time (1000 loops)", elapsedTime1000);
////        panelsTelemetry.addData("Shooting Sequence State", pathState);
////        panelsTelemetry.addData("Artifact Counter", counter);
////        panelsTelemetry.addData("Laser Detection Time", laserTime);
////        panelsTelemetry.addData("Object Detected", stateHigh);
//        panelsTelemetry.addData("Angle To Goal", robotToGoalRelativeAngle);
//        panelsTelemetry.addData("A2 Angle", a2);
//        panelsTelemetry.addData("Drive Power Factor", powerFactor);
//        panelsTelemetry.addData("Intake On", intakeOn);
//        panelsTelemetry.addData("Bearing Error", error);
//        panelsTelemetry.addData("Turret Target Pos", newPos);
//        panelsTelemetry.addData("Turret Target Pose Pos", newPosePos);
//        panelsTelemetry.addData("Turret Current Pos", currentPos);
//        panelsTelemetry.debug("Pinpoint Velocity", follower.getVelocity());
//        panelsTelemetry.debug("Automated Drive", automatedDrive);
//        panelsTelemetry.debug("Slow Mode", slowMode);
//        panelsTelemetry.debug("Slow Mode Multiplier", slowModeMultiplier);
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
                    shootTimer.resetTimer();
                    shooter.openServoStop();
                    counter = 0;
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
                    shootingTime = shootTimer.getElapsedTimeSeconds();
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