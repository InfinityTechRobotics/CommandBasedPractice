package org.firstinspires.ftc.teamcode.Commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.SpintakeSubsystem;

import java.util.Set;


public class stopShooting implements Command {
    private final ShooterSubsystem shooter;
    private final FlywheelSubsystem flywheel;
    private final SpintakeSubsystem spintake;
    private boolean finished = false;

    public stopShooting(ShooterSubsystem shooter, FlywheelSubsystem flywheel, SpintakeSubsystem spintake){
        this.shooter = shooter;
        this.flywheel = flywheel;
        this.spintake = spintake;
    }

    @Override
    public Set<Object> requirements() {
        return Set.of(shooter, flywheel, spintake);
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
        shooter.closeServoStop();
        shooter.downServoPaddle();
        flywheel.setFlywheelVel(0);
        spintake.turnIntakeOff();
    }

    @Override
    public boolean done() {
        return finished;
    }

    @Override
    public void execute() {
    }
    @Override
    public void end(EndCondition endCondition) {

    }
}
