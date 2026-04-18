package org.firstinspires.ftc.teamcode.DisabledAuton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.FlywheelSpinfinityDuo;
import org.firstinspires.ftc.teamcode.Hardware.ShooterSpinfinityDuo;
import org.firstinspires.ftc.teamcode.Hardware.SpintakeSpinfinity;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous
public class bleh_XVI_Spinfinity extends OpMode {

    ShooterSpinfinityDuo shooter = new ShooterSpinfinityDuo();
    FlywheelSpinfinityDuo flywheel = new FlywheelSpinfinityDuo();
    SpintakeSpinfinity spintake = new SpintakeSpinfinity();

    private int obeliskResult = 0;

    double START_DELAY_TIME = 2.0;

    private Follower follower;
    private Timer pathTimer, opmodeTimer, shootTimer;
    private int pathState;

    private DigitalChannel laserInput;

    private static ElapsedTime laserTimer = new ElapsedTime();

    double laserTime;

    public static double SPINTAKE_AUTO_SHUTOFF_THRESHOLD = 0.4; //0.25

    boolean activeDetecting = false;
    boolean stateHigh;

    int counter = 3;
    int prevCount = 0;

    public static double NEW_P = 150.0;   // 10.0
    public static double NEW_I = 5.0;    // 3.0
    public static double NEW_D = 40.0;    // 0.0
    public static double NEW_F = 1.25;    // 0.0

    double shootingTime = 0.0;

    private final Pose startPose = new Pose(28, 129, Math.toRadians(144)); // Reflected over x = 72
// private final Pose viewPose = new Pose(64, 120, Math.toRadians(90)); // Reflected over x = 72

    private final Pose motifDetection = new Pose(59, 110, Math.toRadians(90));

    private final Pose scorePose = new Pose(48, 96, Math.toRadians(135)); // Reflected heading: 180 - 45 = 135

//    private  final Pose preScorePose = new Pose(48, 96, Math.toRadians(125));

    private final Pose prePickupPose21 = new Pose(52, 41, Math.toRadians(180)); // Reflected heading: 180 - 0 = 180

    private final Pose pickupPose21 = new Pose(20, 41, Math.toRadians(180));

    private final Pose prePickupPose22 = new Pose(52, 63, Math.toRadians(180));

    private final Pose pickupPose22 = new Pose(14, 63, Math.toRadians(180));

    private final Pose prePickupPose23 = new Pose(52, 88.75, Math.toRadians(180));

    private final Pose pickupPose23 = new Pose(23, 88.75, Math.toRadians(180));

    private final Pose gatePickup = new Pose(16, 61.5, Math.toRadians(150));

    private final Pose downGate = new Pose(11, 52.5, Math.toRadians(140));

    private final Pose controlPointDriveToGate = new Pose(54, 54);

    private final Pose controlPointDriveToAwayFromGate = new Pose(59, 50);

    private final Pose controlPoint22 = new Pose(64, 60);

    private final Pose endPose = new Pose(54, 67, Math.toRadians(180));

    private final Pose finalShootPose = new Pose(54, 110, Math.toRadians(150));

    private PathChain driveToGoal, driveToPickup23, driveToGoal23, driveToPickup22, driveToGatePickup, driveDownFromGate, driveToAwayFromGate, driveToGoal22, driveToPickup21, driveToGoal21, driveToEnd;

//        DcMotorEx motorIntake;
//        DcMotorEx motorFlywheel;
//        DcMotorEx motorFlywheel2;

    double targetRPM = 0.0;
//        double flywheelRPM = 0.0;
//        double flywheelRPM2 = 0.0;
//        double TPS;
//        double CPR = 28.0;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;
//
//        double intakeOn = 0.8;
//        double intakeOff = 0.0;

    double TARGET_AUTON_RPM = 2325.0; //2475.0

    int shootingSequenceFlag = 1;

    PIDFCoefficients pidfModified;

    int telemtryUpdate = 0;


    private void buildPaths() {

        driveToGoal = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToPickup23 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, prePickupPose23))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickupPose23.getHeading())
                .addPath(new BezierLine(prePickupPose23, pickupPose23))
                .setLinearHeadingInterpolation(prePickupPose23.getHeading(), pickupPose23.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToGoal23 = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose23, scorePose))
                .setLinearHeadingInterpolation(pickupPose23.getHeading(), scorePose.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToPickup22 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, prePickupPose22))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickupPose22.getHeading())
                .addPath(new BezierLine(prePickupPose22, pickupPose22))
                .setLinearHeadingInterpolation(prePickupPose22.getHeading(), pickupPose22.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToGoal22 = follower.pathBuilder()
                .addPath(new BezierCurve(pickupPose22, controlPoint22, scorePose))
                .setLinearHeadingInterpolation(pickupPose22.getHeading(), scorePose.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToGatePickup = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, controlPointDriveToGate, gatePickup))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gatePickup.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveDownFromGate = follower.pathBuilder()
                .addPath(new BezierLine(gatePickup, downGate))
                .setLinearHeadingInterpolation(gatePickup.getHeading(), downGate.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToAwayFromGate = follower.pathBuilder()
                .addPath(new BezierCurve(downGate, controlPointDriveToAwayFromGate, scorePose))
                .setLinearHeadingInterpolation(downGate.getHeading(), scorePose.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToPickup21 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, prePickupPose21))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickupPose21.getHeading())
                .addPath(new BezierLine(prePickupPose21, pickupPose21))
                .setLinearHeadingInterpolation(prePickupPose21.getHeading(), pickupPose21.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToGoal21 = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose21, finalShootPose))
                .setLinearHeadingInterpolation(pickupPose21.getHeading(), finalShootPose.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToEnd = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .setTimeoutConstraint(0)
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10:
                obeliskResult = 23;
                shooter.closeServoStop();
                shooter.downServoPaddle();
                spintake.turnIntakeOn();
                follower.followPath(driveToGoal);
                setPathState(11);
                break;
            case 11:
                if (!follower.isBusy()) {
                    setPathState(1000);
                }
                break;
            case 1000:
                if (shootingSequenceFlag == 22102321) {
                    shootingSequenceFlag = 777;
                } else if (shootingSequenceFlag == 221023) {
                    shootingSequenceFlag = 22102321;
                } else if (shootingSequenceFlag == 2210) {
                    shootingSequenceFlag = 221023;
                } else if (shootingSequenceFlag == 1) {
                    shootingSequenceFlag = 22;
                } else if (shootingSequenceFlag == 22) {
                    shootingSequenceFlag = 2210;
                }
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
                    setPathState(10006);
                }
                break;
            case 10006:
                if (pathTimer.getElapsedTimeSeconds() > 0.6) {
                    counter = 0;
                    shooter.shootServoPaddle();
                    setPathState(10007);
                }
                break;
            case 10007:
                //motorTransfer.setPower(transferOff);
                shootingTime = shootTimer.getElapsedTimeSeconds();
                shooter.downServoPaddle();
                shooter.closeServoStop();
                spintake.turnIntakeOff();
                setPathState(10010);
                break;
            case 10008: // updates shooting sequence flag
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    setPathState(10010);
                }
                break;
            case 10010:  // sends to next driving path
                if (shootingSequenceFlag == 22) {
                    setPathState(220);
                } else if (shootingSequenceFlag == 2210) {
                    setPathState(300);
                } else if (shootingSequenceFlag == 221023) {
                    setPathState(230);
                } else if (shootingSequenceFlag == 22102321) {
                    setPathState(210);
                } else if (shootingSequenceFlag == 777) {
                    setPathState(999);
                }
                //motorTransfer.setPower(transferOn);
                break;
            case 210: // beginning of set of actions for spike mark 21, gets ready to pickup
                spintake.turnIntakeOn();
                if (!follower.isBusy()) {
                    follower.followPath(driveToPickup21);
                    setPathState(211);
                }
                break;
            case 211: // drives toward goal to score first set of artifacts (21)
                if (!follower.isBusy()) {
                    follower.followPath(driveToGoal21);
                    setPathState(214);
                }
                break;
            case 214: // case for shooting
                if (!follower.isBusy()) {
                    setPathState(1000);
                }
                break;
            case 220: // beginning of set of actions for spike mark 22, gets ready to pickup
                spintake.turnIntakeOn();
                if (!follower.isBusy()) {
                    follower.followPath(driveToPickup22);
                    setPathState(221);
                }
                break;
            case 221: // drives toward goal to score second set of artifacts (22)
                if (!follower.isBusy()) {
                    follower.followPath(driveToGoal22);
                    setPathState(224);
                }

                break;
            case 224: // case for shooting
                if (!follower.isBusy()) {
                    setPathState(1000);
                }
                break;
            case 230: // beginning of actions for spike mark 23, gets ready to pickup
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    if (!follower.isBusy()) {
                        spintake.turnIntakeOn();
                        setPathState(231);
                    }
                }
                break;
            case 231:
                if (!follower.isBusy()) {
                    follower.followPath(driveToPickup23);
                    setPathState(234);
                }
                break;
            case 234:
                if (!follower.isBusy()) {
                    follower.followPath(driveToGoal23);
                    setPathState(235);
                }
                break;
            case 235: // case for shooting
                if (!follower.isBusy()) {
                    setPathState(1000);
                }
                break;
            case 300: // case for "gobbling" from the gate
                if (!follower.isBusy()) {
                    follower.followPath(driveToGatePickup);
                    spintake.turnIntakeOn();
                    setPathState(301);
                }
                break;
            case 301:
                if (!follower.isBusy()) {
                    follower.followPath(driveDownFromGate);
                    setPathState(302);
                }
                break;
            case 302:
                if (pathTimer.getElapsedTimeSeconds() > 2)  {
                    follower.followPath(driveToAwayFromGate);
                    setPathState(303);
                }
                break;
            case 303:
                if (!follower.isBusy()) {
                    setPathState(1000);
                }
                break;
            case 777:
                if (!follower.isBusy()) {
                    follower.followPath(driveToEnd);
                    TARGET_AUTON_RPM = 0.0;
                    setPathState(999);
                }
                break;
            case 999: // last state, just stops and waits
                if(pathTimer.getElapsedTimeSeconds() > 1) {
                    TARGET_AUTON_RPM = 0.0;
                    spintake.turnIntakeOff();
                    setPathState(912);
                }
                break;
        }
    }

    private void setPathState(int pState) {

        pathState = pState;
        pathTimer.resetTimer();

    }

    @Override
    public void loop() {

        // Laser Artifact Detection (Detected = TRUE --> counter +1)
        stateHigh = laserInput.getState();

        if (stateHigh) {
            if (!activeDetecting) {
                laserTimer.reset();
                counter += 1;
            }
        } else {        // not detecting
            if (activeDetecting) {
                laserTime = laserTimer.seconds();
            }
        }

        activeDetecting = stateHigh;

        if (counter != prevCount) {
            spintake.setArtifactIndicator(counter);
        }

        prevCount = counter;

        follower.update();
        autonomousPathUpdate();

        targetRPM = TARGET_AUTON_RPM;
        flywheel.setFlywheelVel(targetRPM);

        telemetry.addData("Obelisk ID", obeliskResult); // telemetry for which motif was detected.
        telemetry.addData("Path State", pathState); // the current path the code is running
        telemetry.addData("X", follower.getPose().getX()); // x pos
        telemetry.addData("Y", follower.getPose().getY()); // y pos
        telemetry.addData("Heading", follower.getPose().getHeading()); // heading
        telemetry.addData("Path Timer",pathTimer.getElapsedTimeSeconds());
        telemetry.addData("OpMode Timer", opmodeTimer.getElapsedTimeSeconds());
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Flywheel RPM", flywheel.getFlywheelVel());
        telemetry.addData("Flywheel RPM2", flywheel.getFlywheelVel2());
        telemetry.addData("Shooting Sequence", shootingSequenceFlag);

        if (telemtryUpdate == 49){
            telemtryUpdate = 0;
            telemetry.update();
        } else {
            telemtryUpdate +=1;
        }



    }

    @Override
    public void init() {

        laserInput = hardwareMap.get(DigitalChannel.class, "laserDigitalInput");

        laserInput.setMode(DigitalChannel.Mode.INPUT);

        shooter.init(hardwareMap);
        flywheel.init(hardwareMap);
        spintake.init(hardwareMap);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        shootTimer = new Timer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        shooter.closeServoStop();
        shooter.downServoPaddle();

        shooter.centerMotorTurret();

    }

    @Override
    public void init_loop() {
        // Nothing
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(10);
    }


    @Override
    public void stop() {
        // Nothing
    }

}