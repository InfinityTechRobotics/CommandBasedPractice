package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Commands.FieldCentric;
import org.firstinspires.ftc.teamcode.Commands.RobotCentric;
import org.firstinspires.ftc.teamcode.Commands.ShootingSequence;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SpintakeSubsystem;

public class RobotContainer {

    // Subsystems
    private final DriveSubsystem drive;
    private final ShooterSubsystem shooter;
    private final FlywheelSubsystem flywheel;
    private final SpintakeSubsystem spintake;

    // Commands
    public final FieldCentric fieldCentric;
    public final RobotCentric robotCentric;
    public final ShootingSequence shooting;

    public RobotContainer(
            HardwareMap hardwareMap,
            Follower follower,
            Gamepad gamepad1
    ) {

        // Create subsystems
        drive = new DriveSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        flywheel = new FlywheelSubsystem(hardwareMap);
        spintake = new SpintakeSubsystem(hardwareMap);

        // Create commands
        fieldCentric = new FieldCentric(
                drive,
                follower,
                gamepad1
        );

        robotCentric = new RobotCentric(
                drive,
                follower,
                gamepad1
        );

        shooting = new ShootingSequence(
                shooter,
                flywheel,
                spintake,
                new Timer()
        );
    }
}