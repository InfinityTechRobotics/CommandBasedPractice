package org.firstinspires.ftc.teamcode.DisabledCompetition;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.DisabledHardware.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous
    public class altAutoBlueAnnArbor extends OpMode {

    private Limelight3A limelight;

    Shooter shooter = new Shooter();

    double START_DELAY_TIME = 8.;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    public static double NEW_P = 100.;   // 10.
    public static double NEW_I = 1.;    // 3.
    public static double NEW_D = 20.;    // 0.
    public static double NEW_F = 3.5;    // 0.

    private final Pose startPose = new Pose(64, 8, Math.toRadians(110));
    private final Pose leavePose = new Pose(52, 32, Math.toRadians(110));

    private PathChain driveToLeave;

    DcMotor motorIntake, motorTransfer;
    DcMotorEx motorFlywheel;

    Servo servoTurret;

    double targetRPM = 0.;
    double flywheelRPM = 0.;
    double TPS;
    double CPR = 28.;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;

    double transferOn = 0.8;
    double transferOff = 0.;

    double intakeOn = 0.8;
    double intakeOff = 0.0;

    double TARGET_AUTON_RPM = 3100.;

    int shootingSequenceFlag = 1;

    PIDFCoefficients pidfModified;



    private void buildPaths() {

        driveToLeave = follower.pathBuilder()
                .addPath(new BezierLine(startPose, leavePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), leavePose.getHeading())
                .setTimeoutConstraint(0)
                .build();

    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                setPathState(10);
                break;
            case 10:
                shooter.closeServoStop();
                shooter.downServoPaddle();
                setPathState(1001);
                break;
            case 1001:
                if (pathTimer.getElapsedTimeSeconds() > START_DELAY_TIME) {
                    motorIntake.setPower(intakeOn);
                    motorTransfer.setPower(transferOn);
                    setPathState(10001);
                }
                break;
            case 10001:
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    shooter.openServoStop();
                    setPathState(10002);
                }
                break;
            case 10002:
                if (pathTimer.getElapsedTimeSeconds() > 0.25 ) {
                    shooter.closeServoStop();
                    setPathState(10003);
                }
                break;
            case 10003:
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    shooter.closeServoStop();
                    setPathState(10004);
                }
                break;
            case 10004:
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                    shooter.closeServoStop();
                    setPathState(10005);
                }
                break;
            case 10005:
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    shooter.openServoStop();
                    setPathState(10006);
                }
                break;
            case 10006:
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                    shooter.shootServoPaddle();
                    setPathState(10007);
                }
                break;
            case 10007:
                if (pathTimer.getElapsedTimeSeconds() > 0.75) {
                    //motorTransfer.setPower(transferOff);
                    shooter.downServoPaddle();
                    shooter.closeServoStop();
                    setPathState(10008);
                }
                break;
            case 10008:
                follower.followPath(driveToLeave);
                setPathState(10009);
                break;
            case 10009: // updates shooting sequence flag
                if (pathTimer.getElapsedTimeSeconds() > 0.1 ) {
                    setPathState(777);
                }
                break;
            case 777:
                if (!follower.isBusy()) {
                    TARGET_AUTON_RPM = 0.;
                    setPathState(999);
                }
                break;
            case 999: // last state, just stops and waits
                if(pathTimer.getElapsedTimeSeconds() > 1) {
                    motorTransfer.setPower(transferOff);
                    motorIntake.setPower(intakeOff);
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

        follower.update();
        autonomousPathUpdate();

        targetRPM = TARGET_AUTON_RPM;
        TPS = targetRPM / 60. * CPR;
        motorFlywheel.setVelocity(TPS);
        flywheelRPM = motorFlywheel.getVelocity() * 60 / CPR;

        telemetry.addData("Path State", pathState); // the current path the code is running
        telemetry.addData("X", follower.getPose().getX()); // x pos
        telemetry.addData("Y", follower.getPose().getY()); // y pos
        telemetry.addData("Heading", follower.getPose().getHeading()); // heading
        telemetry.addData("Path Timer",pathTimer.getElapsedTimeSeconds());
        telemetry.addData("OpMode Timer", opmodeTimer.getElapsedTimeSeconds());
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Flywheel RPM", flywheelRPM);
        telemetry.addData("Flywheel Motor Power", motorFlywheel.getPower());
        telemetry.addData("Shooting Sequence", shootingSequenceFlag);
        telemetry.addData("P,I,D,F (modified)", "P: %.4f, I: %.4f, D: %.4f, F: %.4f",
                pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);
        telemetry.update();

    }

    @Override
    public void init() {

        shooter.init(hardwareMap);

        //battery = hardwareMap.get(VoltageSensor.class, "Control Hub");

        motorIntake = hardwareMap.dcMotor.get("motorIntake");
        motorTransfer = hardwareMap.dcMotor.get("motorTransfer");
        motorFlywheel = hardwareMap.get(DcMotorEx.class, "motorFlywheel");

        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        motorTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        motorFlywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

        pidfModified = motorFlywheel.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);


        servoTurret = hardwareMap.get(Servo.class, "servoWebcam");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.update();

        limelight.pipelineSwitch(0);

        limelight.start();

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        shooter.closeServoStop();
        shooter.downServoPaddle();

        servoTurret.setPosition(0.5);

    }

    @Override
    public void init_loop() {
        // Nothing
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }


    @Override
    public void stop() {
        limelight.stop();
    }

}