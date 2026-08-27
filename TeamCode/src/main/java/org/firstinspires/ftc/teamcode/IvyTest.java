package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import static com.pedropathing.ivy.commands.Commands.*;

import com.pedropathing.ivy.Scheduler;

@TeleOp(name = "Ivy Test")
public class IvyTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Follower follower = Constants.createFollower(hardwareMap);

        RobotContainer container = new RobotContainer(
                hardwareMap,
                follower,
                gamepad1
        );

        waitForStart();

        container.fieldCentric.schedule();

        while (opModeIsActive()) {

            follower.update();

            Scheduler.execute();

            telemetry.update();
        }
    }
}