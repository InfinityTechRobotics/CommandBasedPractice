package org.firstinspires.ftc.teamcode.Commands;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

import java.util.Set;

public class RobotCentric implements Command {

    private DriveSubsystem drive;
    private Gamepad gamepad1;
    private double powerFactor = 0.95;

    public RobotCentric(DriveSubsystem drive, Gamepad gamepad1){
        this.drive = drive;
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

        drive.moveRobotRC(y,x,rx,powerFactor);
    }

    @Override
    public void end(EndCondition endCondition) {
        drive.stop();
    }
}
