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
@Autonomous(name = "PedroTest2", group = "pedroPathing")
    public class theSecondPedroOfDecodeVGAT extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(84, 8, Math.toRadians(90)); // Start Pose of our robot.

    private final Pose scorePose = new Pose(84, 56, Math.toRadians(90)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private final Pose pickup1Pose = new Pose(84, 36, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.

    private final Pose pickup2Pose = new Pose(90, 34, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.

    private Path first;

    private PathChain driveToMountain, driveToPickup1, driveToMountain2, driveToPickup2, driveToMountain3;

    private void buildPaths() {

        driveToMountain = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .setTimeoutConstraint(0)
                .build();

        driveToMountain2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1Pose, pickup2Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup2Pose.getHeading())
                .setTimeoutConstraint(0)
                .build();


    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(driveToMountain);
                if (pathTimer.getElapsedTime() > 3000) {
                    setPathState(1);
                }

                break;
            case 1:
                if(!follower.isBusy()) {
                    follower.followPath(driveToPickup1);
                    if (pathTimer.getElapsedTime() > 3000) {
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath((driveToMountain2));
                    if (pathTimer.getElapsedTime() > 3000) {
                        setPathState(3);
                    }
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