package org.firstinspires.ftc.teamcode.Commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

import java.util.Set;


public class ShootingSequence implements Command {
    private final ShooterSubsystem shooter;

    private Timer time;

    public boolean shotStarted = false;
    public boolean paddleUp = false;
    private boolean finished = false;

    public ShootingSequence(ShooterSubsystem shooter){
        this.shooter = shooter;
    }

    @Override
    public Set<Object> requirements() {
        return Set.of(shooter);
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

        shooter.closeServoStop();
        shooter.downServoPaddle();
    }

    @Override
    public boolean done() {
        return finished;
    }

    @Override
    public void execute() {

        if (!paddleUp && time.getElapsedTime() > 600){
            shooter.shootServoPaddle();

            paddleUp = true;

            time.resetTimer();
        }

        if(paddleUp && time.getElapsedTime() > 200){
            shooter.downServoPaddle();
            shooter.closeServoStop();

            finished = true;
        }
    }

    @Override
    public void end(EndCondition endCondition) {

    }
}
