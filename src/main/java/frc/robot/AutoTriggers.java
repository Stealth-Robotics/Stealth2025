package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class AutoTriggers {
    private final double time = 0.1;
    private final Trigger preScoreTrigger;
    private final Trigger scoreTrigger;
    private final Trigger intakeTrigger;
    private final Trigger outtakeTrigger;
    private final Trigger stowTrigger;

    private boolean preScore = false;
    private boolean score = false;
    private boolean intake = false;
    private boolean outtake = false;
    private boolean stow = false;

    public AutoTriggers() {
        preScoreTrigger = new Trigger(() -> preScore);
        scoreTrigger = new Trigger(() -> score);
        intakeTrigger = new Trigger(() -> intake);
        outtakeTrigger = new Trigger(() -> outtake);
        stowTrigger = new Trigger(() -> stow);
    }

    public Command preScore() {
        return Commands.runOnce(() -> preScore = true).andThen(new WaitCommand(time),
                Commands.runOnce(() -> preScore = false));
    }

    public Command score() {
        return Commands.runOnce(() -> score = true).andThen(new WaitCommand(time),
                Commands.runOnce(() -> score = false));
    }

    public Command intake() {
        return Commands.runOnce(() -> intake = true).andThen(new WaitCommand(time),
                Commands.runOnce(() -> intake = false));
    }

    public Command outtake() {
        return Commands.runOnce(() -> outtake = true).andThen(new WaitCommand(time),
                Commands.runOnce(() -> outtake = false));
    }

    public Command stow() {
        return Commands.runOnce(() -> stow = true).andThen(new WaitCommand(time), Commands.runOnce(() -> stow = false));
    }

    public Trigger preScoreTrigger() {
        return preScoreTrigger;
    }

    public Trigger scoreTrigger() {
        return scoreTrigger;
    }

    public Trigger intakeTrigger() {
        return intakeTrigger;
    }

    public Trigger outtakeTrigger() {
        return outtakeTrigger;
    }

    public Trigger stowTrigger() {
        return stowTrigger;
    }

}
