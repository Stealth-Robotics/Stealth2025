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
                IDLE
        }

        private final Supplier<LevelTarget> levelTarget;

        // trigger for states. these should be things that cause robot to move, not
        // buttons that change target level
        private final Trigger preScoreTrigger;
        private final Trigger scoreTrigger;

        @Logged
        private SuperState state = SuperState.IDLE;
        @Logged
        private SuperState prevState = SuperState.IDLE;
        private Map<SuperState, Trigger> stateTriggers = new HashMap<>();

        // will need to add other subsystems here
        private final Elevator elevator;

        public Superstructure(Elevator elevator, Supplier<LevelTarget> levelTarget, Trigger preScoreTrigger,
                        Trigger scoreTrigger) {
                this.elevator = elevator;
                this.levelTarget = levelTarget;
                this.preScoreTrigger = preScoreTrigger;
                this.scoreTrigger = scoreTrigger;

                // add all states to stateTriggers map
                // for each state in the enum, construct a new trigger that will return true
                // when the superstructure state is set to that state
                for (var state : SuperState.values()) {
                        stateTriggers.put(state, new Trigger(() -> this.state == state));
                }
                configureStateTransitions();
        }

        private void configureStateTransitions() {
                stateTriggers.get(SuperState.INTAKE_HP)
                                .whileTrue(elevator.goToPositionInInches(() -> Elevator.INTAKE_HP_INCHES))
                                // can supply something like () -> arm.hasgamepeice to force robot to ready to
                                // score gamepiece after intaking
                                .and(() -> true)
                                .onTrue(this.forceState(SuperState.READY_SCORE_CORAL));

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

                // send elevator/arm to correct level for pre-level scoring, transitions state
                // when
                // driver presses scoring button
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
                                .onTrue(this.forceState(SuperState.SPIT_CORAL));

                stateTriggers.get(SuperState.SCORE_CORAL)
                                .and(() -> levelTarget.get() == LevelTarget.L2)
                                .whileTrue(elevator.goToPositionInInches(() -> Elevator.SCORE_L2_INCHES))
                                .and(() -> elevator.isElevatorAtTarget())
                                .onTrue(this.forceState(SuperState.SPIT_CORAL));

                stateTriggers.get(SuperState.SCORE_CORAL)
                                .and(() -> levelTarget.get() == LevelTarget.L3)
                                .whileTrue(elevator.goToPositionInInches(() -> Elevator.SCORE_L3_INCHES))
                                .and(() -> elevator.isElevatorAtTarget())
                                .onTrue(this.forceState(SuperState.SPIT_CORAL));

                stateTriggers.get(SuperState.SCORE_CORAL)
                                .and(() -> levelTarget.get() == LevelTarget.L4)
                                .whileTrue(elevator.goToPositionInInches(() -> Elevator.SCORE_L4_INCHES))
                                .and(() -> elevator.isElevatorAtTarget())
                                .onTrue(this.forceState(SuperState.SPIT_CORAL));

                stateTriggers.get(SuperState.SPIT_CORAL)
                                // make it eject coral with rollers once we have code for that
                                .whileTrue(Commands.none())
                                // make it check when beambreak is unbroken, meaning gamepiece is gone
                                .and(() -> true)
                                .onTrue(this.forceState(SuperState.IDLE));

        }

        public Command forceState(SuperState nextState) {
                return Commands.runOnce(() -> {
                        this.prevState = this.state;
                        this.state = nextState;
                });
        }
}
