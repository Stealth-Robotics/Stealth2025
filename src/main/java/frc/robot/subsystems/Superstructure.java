package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer.LevelTarget;

public class Superstructure {

    public enum SuperState {
        INTAKE_HP,
        READY_SCORE_CORAL,
        PRE_L1,
        PRE_L2,
        PRE_L3,
        PRE_L4,
        SCORE_CORAL,
        SPIT_CORAL,
        REMOVE_ALGAE_LOW,
        REMOVE_ALGAE_HIGH,
        IDLE
    }

    private final Supplier<LevelTarget> levelTarget;

    // trigger for states. these should be things that cause robot to move, not
    // buttons that change target level
    private final Trigger preScoreTrigger;
    private final Trigger scoreTrigger;
    private final Trigger intakeTrigger;
    private final Trigger outtakeTrigger;
    private final Trigger missedScoreTrigger;
    private final Trigger removeAlgaeHighTrigger;
    private final Trigger removeAlgaeLowTrigger;
    private final Trigger cancelScoringTrigger;

    @Logged
    private SuperState state = SuperState.IDLE;
    @Logged
    private SuperState prevState = SuperState.IDLE;
    private Map<SuperState, Trigger> stateTriggers = new HashMap<>();

    // will need to add other subsystems here
    private final Elevator elevator;
    private final Rollers rollers;

    public Superstructure(Elevator elevator, Rollers rollers, Supplier<LevelTarget> levelTarget,
            Trigger preScoreTrigger,
            Trigger scoreTrigger, Trigger intakeTrigger, Trigger outtakeTrigger,
            Trigger missedScoreTrigger, Trigger removeAlgaeHighTrigger, Trigger removeAlgaeLowTrigger,
            Trigger cancelScoringTrigger) {
        this.elevator = elevator;
        this.rollers = rollers;
        this.levelTarget = levelTarget;
        this.preScoreTrigger = preScoreTrigger;
        this.scoreTrigger = scoreTrigger;
        this.intakeTrigger = intakeTrigger;
        this.outtakeTrigger = outtakeTrigger;
        this.missedScoreTrigger = missedScoreTrigger;
        this.removeAlgaeHighTrigger = removeAlgaeHighTrigger;
        this.removeAlgaeLowTrigger = removeAlgaeLowTrigger;
        this.cancelScoringTrigger = cancelScoringTrigger;

        // add all states to stateTriggers map
        // for each state in the enum, construct a new trigger that will return true
        // when the superstructure state is set to that state
        for (var state : SuperState.values()) {
            stateTriggers.put(state, new Trigger(() -> this.state == state));
        }
        configureStateTransitions();
    }

    private void configureStateTransitions() {
        // allows us to cancel if anything goes wrong
        cancelScoringTrigger
                .onTrue(this.forceState(SuperState.IDLE));

        // Idle -> intakehp
        stateTriggers.get(SuperState.IDLE)
                // stop subsystems when we are in idle state
                .whileTrue(elevator.stopElevator())
                .and(intakeTrigger)
                .onTrue(this.forceState(SuperState.INTAKE_HP));

        stateTriggers.get(SuperState.INTAKE_HP)
                .whileTrue(elevator.goToPositionInInches(() -> Elevator.INTAKE_HP_INCHES))
                .onTrue(rollers.setRollerVoltage(3))
                .and(() -> rollers.getHasGamepiece())
                .onTrue(this.forceState(SuperState.READY_SCORE_CORAL)
                        // need to experiment with good voltage that will keep coral in
                        .alongWith(rollers.setRollerVoltage(0.5)));

        // Idle -> removing algae states
        stateTriggers.get(SuperState.IDLE)
                .and(removeAlgaeHighTrigger)
                .onTrue(this.forceState(SuperState.REMOVE_ALGAE_HIGH));

        stateTriggers.get(SuperState.IDLE)
                .and(removeAlgaeLowTrigger)
                .onTrue(this.forceState(SuperState.REMOVE_ALGAE_LOW));

        stateTriggers.get(SuperState.REMOVE_ALGAE_HIGH)
                .whileTrue(elevator.goToPositionInInches(() -> Elevator.REMOVE_ALGAE_HIGH_INCHES))
                .whileTrue(rollers.setRollerVoltage(3))
                .and(() -> elevator.isElevatorAtTarget())
                .and(() -> rollers.getHasGamepiece())
                .onTrue(this.forceState(SuperState.IDLE)
                        // need to experiment with good voltage that will keep algae in
                        .alongWith(rollers.setRollerVoltage(0.5)));

        stateTriggers.get(SuperState.REMOVE_ALGAE_LOW)
                .whileTrue(elevator.goToPositionInInches(() -> Elevator.REMOVE_ALGAE_LOW_INCHES))
                .whileTrue(rollers.setRollerVoltage(3))
                .and(() -> elevator.isElevatorAtTarget())
                .and(() -> rollers.getHasGamepiece())
                .onTrue(this.forceState(SuperState.IDLE)
                        // again, experiment
                        .alongWith(rollers.setRollerVoltage(0.5)));

        // when we are in ready mode, meaning arm has a coral, we now need to check for
        // when driver presses button to actually go to scoring location
        stateTriggers.get(SuperState.READY_SCORE_CORAL)
                .and(() -> levelTarget.get() == LevelTarget.L1)
                .and(preScoreTrigger)
                .onTrue(this.forceState(SuperState.PRE_L1));

        stateTriggers.get(SuperState.READY_SCORE_CORAL)
                .and(() -> levelTarget.get() == LevelTarget.L2)
                .and(preScoreTrigger)
                .onTrue(this.forceState(SuperState.PRE_L2));

        stateTriggers.get(SuperState.READY_SCORE_CORAL)
                .and(() -> levelTarget.get() == LevelTarget.L3)
                .and(preScoreTrigger)
                .onTrue(this.forceState(SuperState.PRE_L3));

        stateTriggers.get(SuperState.READY_SCORE_CORAL)
                .and(() -> levelTarget.get() == LevelTarget.L4)
                .and(preScoreTrigger)
                .onTrue(this.forceState(SuperState.PRE_L4));

        /*
         * send elevator/arm to correct level for pre-level scoring, transitions state
         * when driver presses scoring button
         */
        stateTriggers.get(SuperState.PRE_L1)
                .whileTrue(elevator.goToPositionInInches(() -> Elevator.PRE_L1_INCHES))
                .and(() -> elevator.isElevatorAtTarget())
                .and(scoreTrigger)
                .onTrue(this.forceState(SuperState.SCORE_CORAL));

        stateTriggers.get(SuperState.PRE_L2)
                .whileTrue(elevator.goToPositionInInches(() -> Elevator.PRE_L2_INCHES))
                .and(() -> elevator.isElevatorAtTarget())
                .and(scoreTrigger)
                .onTrue(this.forceState(SuperState.SCORE_CORAL));

        stateTriggers.get(SuperState.PRE_L3)
                .whileTrue(elevator.goToPositionInInches(() -> Elevator.PRE_L3_INCHES))
                .and(() -> elevator.isElevatorAtTarget())
                .and(scoreTrigger)
                .onTrue(this.forceState(SuperState.SCORE_CORAL));

        stateTriggers.get(SuperState.PRE_L4)
                .whileTrue(elevator.goToPositionInInches(() -> Elevator.PRE_L4_INCHES))
                .and(() -> elevator.isElevatorAtTarget())
                .and(scoreTrigger)
                .onTrue(this.forceState(SuperState.SCORE_CORAL));

        // when driver presses button to score, we lower elevator/arm to scoring level,
        // then command rollers to spit
        stateTriggers.get(SuperState.SCORE_CORAL)
                .and(() -> levelTarget.get() == LevelTarget.L1)
                .whileTrue(elevator.goToPositionInInches(() -> Elevator.SCORE_L1_INCHES))
                .and(() -> elevator.isElevatorAtTarget())
                .and(outtakeTrigger)
                .onTrue(this.forceState(SuperState.SPIT_CORAL));

        stateTriggers.get(SuperState.SCORE_CORAL)
                .and(() -> levelTarget.get() == LevelTarget.L2)
                .whileTrue(elevator.goToPositionInInches(() -> Elevator.SCORE_L2_INCHES))
                .and(() -> elevator.isElevatorAtTarget())
                .and(outtakeTrigger)
                .onTrue(this.forceState(SuperState.SPIT_CORAL));

        stateTriggers.get(SuperState.SCORE_CORAL)
                .and(() -> levelTarget.get() == LevelTarget.L3)
                .whileTrue(elevator.goToPositionInInches(() -> Elevator.SCORE_L3_INCHES))
                .and(() -> elevator.isElevatorAtTarget())
                .and(outtakeTrigger)
                .onTrue(this.forceState(SuperState.SPIT_CORAL));

        stateTriggers.get(SuperState.SCORE_CORAL)
                .and(() -> levelTarget.get() == LevelTarget.L4)
                .whileTrue(elevator.goToPositionInInches(() -> Elevator.SCORE_L4_INCHES))
                .and(() -> elevator.isElevatorAtTarget())
                .and(outtakeTrigger)
                .onTrue(this.forceState(SuperState.SPIT_CORAL));

        // section for missed score, go back to pre state
        stateTriggers.get(SuperState.SCORE_CORAL)
                .and(() -> levelTarget.get() == LevelTarget.L1)
                .and(missedScoreTrigger)
                .onTrue(this.forceState(SuperState.PRE_L1));

        stateTriggers.get(SuperState.SCORE_CORAL)
                .and(() -> levelTarget.get() == LevelTarget.L2)
                .and(missedScoreTrigger)
                .onTrue(this.forceState(SuperState.PRE_L2));

        stateTriggers.get(SuperState.SCORE_CORAL)
                .and(() -> levelTarget.get() == LevelTarget.L3)
                .and(missedScoreTrigger)
                .onTrue(this.forceState(SuperState.PRE_L3));

        stateTriggers.get(SuperState.SCORE_CORAL)
                .and(() -> levelTarget.get() == LevelTarget.L4)
                .and(missedScoreTrigger)
                .onTrue(this.forceState(SuperState.PRE_L4));

        // ejecting gamepiece
        stateTriggers.get(SuperState.SPIT_CORAL)
                .onTrue(rollers.setRollerVoltage(-3))
                .and(() -> !rollers.getHasGamepiece())
                .onTrue(this.forceState(SuperState.IDLE));

    }

    public void printState() {
        System.out.println(levelTarget.get());
    }

    public SuperState getPrevState() {
        return prevState;
    }

    public SuperState getState() {
        return state;
    }

    public Command forceState(SuperState nextState) {
        return Commands.runOnce(() -> {
            this.prevState = this.state;
            this.state = nextState;
        });
    }

}
