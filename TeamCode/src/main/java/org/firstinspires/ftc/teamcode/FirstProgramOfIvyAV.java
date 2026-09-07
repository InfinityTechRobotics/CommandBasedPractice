package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Commands.ShootingSequence;
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SpintakeSubsystem;

public class FirstProgramOfIvyAV {

    private final ShooterSubsystem shooter;
    public final ShootingSequence shooting;
    private final FlywheelSubsystem flywheel;
    private final SpintakeSubsystem spintake;

    public FirstProgramOfIvyAV(
            HardwareMap hardwareMap,
            Follower follower,
            Gamepad gamepad1
    ) {

        shooter = new ShooterSubsystem(hardwareMap);
        flywheel = new FlywheelSubsystem(hardwareMap);
        spintake = new SpintakeSubsystem(hardwareMap);

        shooting = new ShootingSequence(
                shooter,
                flywheel,
                spintake,
                new Timer()
        );

    }
}
