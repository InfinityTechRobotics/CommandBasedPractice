package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.ivy.Scheduler;

@TeleOp(name = "Ivy Test")
public class IvyTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.createFollower(hardwareMap);

        RobotContainer robot = new RobotContainer(
                hardwareMap,
                follower,
                gamepad1
        );

        waitForStart();

        robot.fieldCentric.schedule();

        while (opModeIsActive()) {
            Scheduler.execute();

            telemetry.addData("Status", "Ivy Running");
            telemetry.update();
        }
    }
}
