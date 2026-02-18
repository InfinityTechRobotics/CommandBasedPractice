package org.firstinspires.ftc.teamcode.DisabledAuton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Disabled
@Autonomous(name = "FifthPedroTestWithLimelightSRVGATIN", group = "pedroPathing")
    public class theFifthPedroOfDecodeWithLimeLightSRVGATINHF extends OpMode {

    boolean targetFound = false;
    private Limelight3A limelight;

    private int obeliskResult = 0;

    private static final int DESIRED_TAG_ID_1 = 21;

    private static final int DESIRED_TAG_ID_2 = 22;

    private static final int DESIRED_TAG_ID_3 = 23;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(80, 8, Math.toRadians(90)); // Start Pose of our robot.

    private final Pose viewPose = new Pose(80, 120, Math.toRadians(90));

    private final Pose scorePose = new Pose(80, 102, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private final Pose prePickupPose1 = new Pose(100, 82, Math.toRadians(0));

    private final Pose pickup1Pose = new Pose(124, 82, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose prePickupPose2 = new Pose(100, 58, Math.toRadians(0));

    private final Pose pickup2Pose = new Pose(124, 58, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.

    private final Pose prePickupPose3 = new Pose(100, 36, Math.toRadians(0));

    private final Pose pickup3Pose = new Pose(124, 36, Math.toRadians(0));

    private Path first;

    private PathChain driveToObelisk, driveToGoal, driveToPrePickup1, driveToPickup1, driveToGoal2, driveToPrePickup2, driveToPickup2, driveToGoal3, driveToPrePickup3, driveToPickup3, driveToGoal4;

    private void buildPaths() {

        driveToObelisk = follower.pathBuilder()
                .addPath(new BezierLine(startPose, viewPose))
                .setConstantHeadingInterpolation(startPose.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToGoal = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToPrePickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, prePickupPose1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickupPose1.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(prePickupPose1, pickup1Pose))
                .setLinearHeadingInterpolation(prePickupPose1.getHeading(), pickup1Pose.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToGoal2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToPrePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, prePickupPose2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickupPose2.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(prePickupPose2, pickup2Pose))
                .setLinearHeadingInterpolation(prePickupPose2.getHeading(), pickup2Pose.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToGoal3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToPrePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, prePickupPose3))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickupPose3.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(prePickupPose3, pickup3Pose))
                .setLinearHeadingInterpolation(prePickupPose3.getHeading(), pickup3Pose.getHeading())
                .setTimeoutConstraint(0)
                .build();


    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                LLResult result = limelight.getLatestResult();
                if (result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        obeliskResult = fr.getFiducialId();
                        if (obeliskResult == DESIRED_TAG_ID_1 || obeliskResult == DESIRED_TAG_ID_2 || obeliskResult == DESIRED_TAG_ID_3) {
                            setPathState(10);
                        } else if (opmodeTimer.getElapsedTimeSeconds() > 5) {
                            obeliskResult = 22;
                            setPathState(10);
                        } else {
                            setPathState(0);
                        }
                    }
                }
                else if (opmodeTimer.getElapsedTimeSeconds() > 5) {
                    obeliskResult = 22;
                    setPathState(10);
                }
                else {
                    obeliskResult = 0;
                    setPathState(0);
                }
                break;
            case 10:
                follower.followPath(driveToGoal);
                if(!follower.isBusy()) {
                    setPathState(11);
                }
                break;
            case 11:
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setPathState(12);
                }
                break;
            case 12:
                setPathState(19);
                break;
            case 19:
                if (obeliskResult == 22) {
                    setPathState(50);
            }   else  {
                    setPathState(101);
            }
                break;
            case 20:
                if(!follower.isBusy()) {
                    follower.followPath(driveToPrePickup1);
                    setPathState(21);
                }
                break;
            case 21:
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setPathState(30);
                }
            case 30:
                if(!follower.isBusy()) {
                    follower.followPath((driveToPickup1));
                    setPathState(31);
                }
                break;
            case 31:
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setPathState(40);
                }
                break;
            case 40:
                if(!follower.isBusy()) {
                    follower.followPath(driveToGoal2);
                    setPathState(41);
                }
                break;
            case 41:
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setPathState(50);
                }
                break;
            case 50:
                if(!follower.isBusy()) {
                    follower.followPath(driveToPrePickup2);
                    setPathState(51);
                }
                break;
            case 51:
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setPathState(60);
                }
                break;
            case 60:
                if(!follower.isBusy()) {
                    follower.followPath(driveToPickup2);
                    setPathState(61);
                }
                break;
            case 61:
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setPathState(70);
                }
                break;
            case 70:
                if(!follower.isBusy()) {
                    follower.followPath(driveToGoal3);
                    setPathState(71);
                }
                break;
            case 71:
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setPathState(80);
                }
                break;
            case 80:
                if(!follower.isBusy()) {
                    follower.followPath(driveToPrePickup3);
                    setPathState(81);
                }
                break;
            case 81:
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setPathState(90);
                }
                break;
            case 90:
                if(!follower.isBusy()) {
                    follower.followPath(driveToPickup3);
                    setPathState(100);
                }
            case 100:
                if(!follower.isBusy()) {
                    follower.followPath(driveToGoal4);
                    setPathState(101);
                }
            case 101:
               if(pathTimer.getElapsedTimeSeconds() > 20) {
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

        telemetry.addData("Obelisk ID", obeliskResult);
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path Timer",pathTimer.getElapsedTimeSeconds());
        telemetry.addData("OpMode Timer", opmodeTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    @Override
    public void init() {

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

    }

    @Override
    public void init_loop() {}

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