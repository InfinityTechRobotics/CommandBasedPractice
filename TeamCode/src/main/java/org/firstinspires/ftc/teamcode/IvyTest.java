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

        Robot robot = new Robot(hardwareMap);

        RobotContainer container = new RobotContainer(
                robot,
                follower,
                gamepad1
        );

        waitForStart();

        container.robotCentric.schedule();
        container.shootingControl.schedule();

        while (opModeIsActive()) {

            follower.update();

            Scheduler.execute();

            telemetry.update();
        }
    }
}