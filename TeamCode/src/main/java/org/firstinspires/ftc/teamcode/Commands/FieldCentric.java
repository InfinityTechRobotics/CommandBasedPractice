package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.*;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

import java.util.Collections;
import java.util.Set;

public class FieldCentric implements Command {

    private final DriveSubsystem drive;
    private final Follower follower;
    private final Gamepad gamepad1;
    private final double powerFactor = 0.95;



    public FieldCentric (DriveSubsystem drive, Follower follower, Gamepad gamepad1){
        this.drive = drive;
        this.follower = follower;
        this.gamepad1 = gamepad1;
    }


    @Override
    public Set<Object> requirements() {
        return Set.of(drive);
    }

    @Override
    public int priority() {
        return 0;
    }

    @Override
    public InterruptedBehavior interruptedBehavior() {
        return null;
    }

    @Override
    public ConflictBehavior conflictBehavior() {
        return null;
    }

    @Override
    public BlockedBehavior blockedBehavior() {
        return null;
    }

    @Override
    public void start() {

    }

    @Override
    public boolean done() {
        return false;
    }

    @Override
    public void execute() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double botHeading = follower.getHeading();

        drive.moveRobotFC(y, x, rx, botHeading, powerFactor);
    }

    @Override
    public void end(EndCondition endCondition) {
        drive.stop();
    }
}
