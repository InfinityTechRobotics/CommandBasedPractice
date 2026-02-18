package org.firstinspires.ftc.teamcode.DisabledAuton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous(name = "PedroTestWithTimers", group = "pedroPathing")
    public class theThirdPedroOfDecodeWithTimersVGAT extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(80, 8, Math.toRadians(90)); // Start Pose of our robot.

    private final Pose scorePose = new Pose(80, 102, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private final Pose prePickupPose1 = new Pose(100, 82, Math.toRadians(0));

    private final Pose pickup1Pose = new Pose(124, 82, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose prePickupPose2 = new Pose(100, 58, Math.toRadians(0));

    private final Pose pickup2Pose = new Pose(124, 58, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.

    private Path first;

    private PathChain driveToGoal, driveToPrePickup1, driveToPickup1, driveToGoal2, driveToPrePickup2, driveToPickup2, driveToGoal3;

    private void buildPaths() {

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

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(driveToGoal);
                setPathState(11);
                break;
            case 11:
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setPathState(1);
                }
                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(driveToPrePickup1);
                    setPathState(22);
                }
                break;
            case 22:
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setPathState(2);
                }
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath((driveToPickup1));
                    setPathState(33);
                }
                break;
            case 33:
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(driveToGoal2);
                    setPathState(44);
                }
                break;
            case 44:
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(driveToPrePickup2);
                    setPathState(55);
                }
                break;
            case 55:
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(driveToPickup2);
                    setPathState(66);
                }
                break;
            case 66:
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(driveToGoal3);
                    setPathState(7);
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

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Timer",pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    @Override
    public void init() {

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
    public void stop() {}

}