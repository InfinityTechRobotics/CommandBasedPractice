package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SpintakeSubsystem;

public class Robot {

    public final DriveSubsystem drive;
    public final ShooterSubsystem shooter;
    public final FlywheelSubsystem flywheel;
    public final SpintakeSubsystem spintake;

    public Robot(HardwareMap hardwareMap) {

        drive = new DriveSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        flywheel = new FlywheelSubsystem(hardwareMap);
        spintake = new SpintakeSubsystem(hardwareMap);
    }
}
