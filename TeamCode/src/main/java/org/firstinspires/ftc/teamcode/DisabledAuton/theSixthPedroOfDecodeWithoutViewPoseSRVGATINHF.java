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
@Autonomous(name = "SixthPedroTestWithoutViewPoseSRVGATINHF", group = "pedroPathing")
    public class theSixthPedroOfDecodeWithoutViewPoseSRVGATINHF extends OpMode {

    boolean targetFound = false;
    private Limelight3A limelight;

    private int obeliskResult = 0;

    private int caseOrder = 0;

    private static final int DESIRED_TAG_ID_1 = 21;

    private static final int DESIRED_TAG_ID_2 = 22;

    private static final int DESIRED_TAG_ID_3 = 23;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(80, 8, Math.toRadians(90)); // Start Pose of our robot.

    private final Pose viewPose = new Pose(80, 120, Math.toRadians(90)); // Pose to read the Obelisk.

    private final Pose scorePose = new Pose(80, 102, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private final Pose prePickupPose1 = new Pose(100, 82, Math.toRadians(0)); // Preparing to intake first set of artifacts.

    private final Pose pickup1Pose = new Pose(124, 82, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark(PPG).

    private final Pose prePickupPose2 = new Pose(100, 58, Math.toRadians(0)); // Preparing to intake second set of artifacts.

    private final Pose pickup2Pose = new Pose(124, 58, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark(PGP).

    private final Pose prePickupPose3 = new Pose(100, 36, Math.toRadians(0)); // Preparing to intake third set of artifacts.

    private final Pose pickup3Pose = new Pose(124, 36, Math.toRadians(0)); // Last (Third Set) of Artifacts from the Spike Mark(GPP).

    private Path first; // what is this? - Arisha

    private PathChain driveToObelisk, driveToGoal, driveToPrePickup1, driveToPickup1, driveToGoal2, driveToPrePickup2, driveToPickup2, driveToGoal3, driveToPrePickup3, driveToPickup3, driveToGoal4;

    private void buildPaths() {
        // The beginning paths(regardless of motif pattern) are below
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
        // The paths for the first set of artifacts(PPG) are below
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
        // The paths for the second set of artifacts(PGP) are below
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
        // The paths for the third set of artifacts(GPP) are below
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

        driveToGoal4 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .setTimeoutConstraint(0)
                .build();


    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: //Reads Obelisk
                LLResult result = limelight.getLatestResult(); // takes limelight reading
                if (result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults(); // AprilTag numbers are fiducial markers
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        obeliskResult = fr.getFiducialId(); // Sets obeliskResult to 21, 22, or 23 depending on the AprilTag
                        if (obeliskResult == DESIRED_TAG_ID_1 || obeliskResult == DESIRED_TAG_ID_2 || obeliskResult == DESIRED_TAG_ID_3) {
                            setPathState(10); // No matter what motif it is, we go score our preloaded artifacts
                        } else if (opmodeTimer.getElapsedTimeSeconds() > 5) {
                            obeliskResult = 22; // If we can't read the obelisk within 5 seconds, we'll assume it's pattern PGP
                            setPathState(10); // We go score our preloaded artifacts
                        } else {
                            setPathState(0); // Until five seconds are up, we keep checking the limelight reading
                        }
                    }
                }
                else if (opmodeTimer.getElapsedTimeSeconds() > 5) {
                    obeliskResult = 22; // Assumes PGP if it doesn't detect within five seconds
                    setPathState(10); // Goes to score preloaded artifacts
                }
                else {
                    obeliskResult = 0; // We don't set an obelisk result until five seconds are up
                    setPathState(0); // Keeps checking limelight reading
                }
                break;
            case 10: // drives toward goal to score preloaded artifacts
                follower.followPath(driveToGoal);
                if(!follower.isBusy()) {
                    setPathState(11);
                }
                break;
            case 11: // timer case
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setPathState(12);
                }
                break;
            case 12: // placeholder state for shooter mechanism
                setPathState(19);
                break;
            case 19: // decides which spike mark to go to based on obelisk reading and sets variable caseOrder accordingly
                if (obeliskResult == 21)  {
                    caseOrder = 1;
                    setPathState(20);
                } else if (obeliskResult == 22)  {
                    caseOrder = 2;
                    setPathState(50);
                } else if (obeliskResult == 23) {
                    caseOrder = 3;
                    setPathState(80);
                    } else {
                    setPathState(50);
                }
                break;
            case 20: // beginning of first set of actions, gets ready to pickup
                if(!follower.isBusy()) {
                    follower.followPath(driveToPrePickup1);
                    setPathState(21);
                }
                break;
            case 21: // timer case
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setPathState(30);
                }
            case 30: // picks up first set of artifacts
                if(!follower.isBusy()) {
                    follower.followPath((driveToPickup1));
                    setPathState(31);
                }
                break;
            case 31: // timer case
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setPathState(40);
                }
                break;
            case 40: // drives toward goal to score first set of artifacts
                if(!follower.isBusy()) {
                    follower.followPath(driveToGoal2);
                    setPathState(41);
                }
                break;
            case 41: // checks caseOrder variable and decides whether or not to move on
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    if ((caseOrder == 1) || (caseOrder == 3)) {
                        setPathState(50);
                    } else if (caseOrder == 2) {
                        setPathState(102);
                    } else {
                        setPathState(102);
                    }
                }
                break;
            case 50: // beginning of second set of actions, gets ready to pickup
                if(!follower.isBusy()) {
                    follower.followPath(driveToPrePickup2);
                    setPathState(51);
                }
                break;
            case 51: // timer case
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setPathState(60);
                }
                break;
            case 60: // picks up second set of artifacts
                if(!follower.isBusy()) {
                    follower.followPath(driveToPickup2);
                    setPathState(61);
                }
                break;
            case 61: // timer case
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setPathState(70);
                }
                break;
            case 70: // drives toward goal to score second set of artifacts
                if(!follower.isBusy()) {
                    follower.followPath(driveToGoal3);
                    setPathState(71);
                }
                break;
            case 71: // checks caseOrder variable and decides whether or not to move on
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    if ((caseOrder == 1) || (caseOrder == 2)) {
                        setPathState(80);
                    } else if (caseOrder == 3) {
                        setPathState(102);
                    } else {
                        setPathState(102);
                    }
                }
                break;
            case 80: // beginning of second set of actions, gets ready to pickup
                if(!follower.isBusy()) {
                    follower.followPath(driveToPrePickup3);
                    setPathState(81);
                }
                break;
            case 81: // timer case
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    setPathState(90);
                }
                break;
            case 90: // picks up third set of artifacts
                if(!follower.isBusy()) {
                    follower.followPath(driveToPickup3);
                    setPathState(100);
                }
            case 100: // drives toward goal to score first set of artifacts
                if(!follower.isBusy()) {
                    follower.followPath(driveToGoal4);
                    setPathState(101);
                }
            case 101: // checks caseOrder variable and decides whether or not to move on
                if(pathTimer.getElapsedTimeSeconds() > 5) {
                    if ((caseOrder == 2) || (caseOrder == 3)) {
                        setPathState(20);
                    } else if (caseOrder == 1) {
                        setPathState(102);
                    } else {
                        setPathState(102);
                    }
                }
            case 102: // last state, just stops and waits
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

        telemetry.addData("Obelisk ID", obeliskResult); // telemetry for which motif was detected.
        telemetry.addData("path state", pathState); // the current path the code is running
        telemetry.addData("x", follower.getPose().getX()); // x pos
        telemetry.addData("y", follower.getPose().getY()); // y pos
        telemetry.addData("heading", follower.getPose().getHeading()); // heading
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