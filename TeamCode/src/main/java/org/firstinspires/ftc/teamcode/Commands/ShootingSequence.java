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


public class ShootingSequence implements Command {
    private final ShooterSubsystem shooter;
    private final FlywheelSubsystem flywheel;
    private final SpintakeSubsystem spintake;

    private Timer time;

    public boolean shotStarted = false;
    private boolean finished = false;

    private double targetRPM = 1000;

    public ShootingSequence(ShooterSubsystem shooter, FlywheelSubsystem flywheel, SpintakeSubsystem spintake, Timer time){
        this.shooter = shooter;
        this.flywheel = flywheel;
        this.spintake = spintake;
        this.time = time;
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
        time.resetTimer();

        shooter.openServoStop();
        shooter.downServoPaddle();

    }

    @Override
    public boolean done() {
        return finished;
    }

    @Override
    public void execute() {
        flywheel.setFlywheelVel(targetRPM);
        spintake.turnIntakeOn();
    }

    @Override
    public void end(EndCondition endCondition) {
    }
}
