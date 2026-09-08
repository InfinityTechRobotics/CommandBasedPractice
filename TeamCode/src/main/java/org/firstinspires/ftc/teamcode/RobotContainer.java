package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Commands.ShootingSequence;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.loop;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

public class RobotContainer {
    public final Command robotCentric;
    public final Command shootingControl;
    public final Command openStop;
    public final Command closeStop;
    public final Command shootPaddle;
    public final Command downPaddle;
    public final Command shootingSequence;

    public RobotContainer(
            Robot robot,
            Follower follower,
            Gamepad gamepad1
    ) {

        robotCentric = robot.drive.robotCentric(
                gamepad1
        );
        openStop = robot.shooter.openStop();
        closeStop = robot.shooter.closeStop();
        shootPaddle = robot.shooter.shootPaddle();
        downPaddle = robot.shooter.downPaddle();

        shootingSequence = conditional(
                () -> gamepad1.a,
                sequential(openStop,
                        waitMs(600),
                        shootPaddle,
                        waitMs(200),
                        parallel(
                                closeStop, downPaddle
                        )
                        ),
                closeStop
        );


        shootingControl =
                parallel(
                        robot.flywheel.setFlywheel(700),
                        robot.spintake.intakeOn(),
                        shootingSequence);

    }
}