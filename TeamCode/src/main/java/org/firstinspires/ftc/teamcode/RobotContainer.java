package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Commands.FieldCentric;
import org.firstinspires.ftc.teamcode.Commands.RobotCentric;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

public class RobotContainer {

    private final DriveSubsystem drive;
    private final ShooterSubsystem shooter;

    public final FieldCentric fieldCentric;
    public final RobotCentric robotCentric;

    public RobotContainer(HardwareMap hardwareMap,  Follower follower, Gamepad gamepad1) {

        // Create the subsystems
        drive = new DriveSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);

        fieldCentric = new FieldCentric(drive, follower, gamepad1);

        robotCentric = new RobotCentric(drive, gamepad1);
    }
}