package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Commands.ShootingSequence;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.loop;

public class RobotContainer {
    public final Command robotCentric;
    public final Command shooting;
    public final Command shootingControl;

    public RobotContainer(
            Robot robot,
            Follower follower,
            Gamepad gamepad1
    ) {

        robotCentric = robot.drive.robotCentric(
                gamepad1
        );

        // Shooting command
        shooting = new ShootingSequence(
                robot.shooter,
                robot.flywheel,
                robot.spintake,
                new Timer()
        );

        shootingControl = loop(
                waitUntil(() -> gamepad1.a)
                        .then(
                                shooting.until(() -> gamepad1.b)
                        )
        );
    }
}