package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer.LevelTarget;
import frc.robot.RobotContainer.AlgaeTarget;

public class Superstructure {

        public enum SuperState {
                INTAKE_HP,
                GRAB_CORAL,
                READY_SCORE_CORAL,
                PRE_L1,
                PRE_L2,
                PRE_L3,
                PRE_L4,
                SCORE_CORAL,
                SPIT,
                REMOVE_ALGAE_LOW,
                REMOVE_ALGAE_HIGH,
                READY_SCORE_ALGAE,
                PRE_PROCESSOR,
                PRE_NET,
                STOW,
                IDLE
        }

        private final Supplier<LevelTarget> levelTarget;
        private final Supplier<AlgaeTarget> algaeTarget;

        // trigger for states. these should be things that cause robot to move, not
        // buttons that change target level
        private Trigger preScoreTrigger;
        private Trigger scoreTrigger;
        private Trigger intakeTrigger;
        private Trigger outtakeTrigger;
        private Trigger missedScoreTrigger;
        private Trigger stowTrigger;
        private Trigger removeAlgaeHighTrigger;
        private Trigger removeAlgaeLowTrigger;
        private Trigger cancelScoringTrigger;

        private final Trigger gamepieceDetectedInStagingArea;

        private final TimeOfFlight tof;

        @Logged
        private SuperState state = SuperState.IDLE;
        @Logged
        private SuperState prevState = SuperState.IDLE;
        private Map<SuperState, Trigger> stateTriggers = new HashMap<>();

        // will need to add other subsystems here
        private final Elevator elevator;
        private final Rollers rollers;
        private final Arm arm;
        private final Transfer transfer;

        public Superstructure(Elevator elevator,
                        Rollers rollers,
                        Arm arm,
                        Transfer transfer,
                        Supplier<LevelTarget> levelTarget,
                        Supplier<AlgaeTarget> algaeTarget,
                        Trigger preScoreTrigger,
                        Trigger scoreTrigger,
                        Trigger intakeTrigger,
                        Trigger outtakeTrigger,
                        Trigger missedScoreTrigger,
                        Trigger stowTrigger,
                        Trigger removeAlgaeHighTrigger,
                        Trigger removeAlgaeLowTrigger,
                        Trigger cancelScoringTrigger) {
                this.elevator = elevator;
                this.rollers = rollers;
                this.arm = arm;
                this.transfer = transfer;
                this.levelTarget = levelTarget;
                this.algaeTarget = algaeTarget;
                this.preScoreTrigger = preScoreTrigger;
                this.scoreTrigger = scoreTrigger;
                this.intakeTrigger = intakeTrigger;
                this.outtakeTrigger = outtakeTrigger;
                this.missedScoreTrigger = missedScoreTrigger;
                this.stowTrigger = stowTrigger;
                this.removeAlgaeHighTrigger = removeAlgaeHighTrigger;
                this.removeAlgaeLowTrigger = removeAlgaeLowTrigger;
                this.cancelScoringTrigger = cancelScoringTrigger;

                // TODO FIND CAN ID
                tof = new TimeOfFlight(0);
                // TODO FIND RANGE THAT WORKS WITH TIME OF FLIGHT
                gamepieceDetectedInStagingArea = new Trigger(() -> tof.getRange() < 10);

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

                // Idle -> stow
                stateTriggers.get(SuperState.IDLE)
                                // stop subsystems when we are in idle state
                                .whileTrue(elevator.stopElevator())
                                // if we choose to stow, go to stow state
                                .and(stowTrigger)
                                .onFalse(this.forceState(SuperState.STOW));

                // Idle -> removing algae states
                stateTriggers.get(SuperState.IDLE)
                                .and(removeAlgaeHighTrigger)
                                .onTrue(this.forceState(SuperState.REMOVE_ALGAE_HIGH));

                stateTriggers.get(SuperState.IDLE)
                                .and(removeAlgaeLowTrigger)
                                .onTrue(this.forceState(SuperState.REMOVE_ALGAE_LOW));

                stateTriggers.get(SuperState.STOW)
                                .whileTrue(elevator.goToPositionInInches(() -> Elevator.STOWED_INCHES))
                                .whileTrue(arm.rotateToPositionCommand(() -> Arm.STOWED_DEGREES))
                                // if we are stowing, we dont really care if we reach stow position before going
                                // to intake
                                // so, we just check intake trigger if we need to intake
                                .and(intakeTrigger)
                                .onTrue(this.forceState(SuperState.INTAKE_HP));

                // Stow -> removing algae states
                stateTriggers.get(SuperState.STOW)
                                .and(removeAlgaeHighTrigger)
                                .onTrue(this.forceState(SuperState.REMOVE_ALGAE_HIGH));
                stateTriggers.get(SuperState.STOW)
                                .and(removeAlgaeLowTrigger)
                                .onTrue(this.forceState(SuperState.REMOVE_ALGAE_LOW));

                stateTriggers.get(SuperState.INTAKE_HP)
                                .whileTrue(elevator.goToPositionInInches(() -> Elevator.INTAKE_HP_INCHES))
                                .whileTrue(arm.rotateToPositionCommand(() -> Arm.INTAKE_HP_DEGREES))
                                .whileTrue(transfer.setVoltage(() -> 3)) // todo: test voltage that works
                                .and(() -> arm.isMotorAtTarget())
                                .and(gamepieceDetectedInStagingArea)
                                .onTrue(this.forceState(SuperState.GRAB_CORAL));

                stateTriggers.get(SuperState.GRAB_CORAL)
                                .whileTrue(elevator.goToPositionInInches(() -> Elevator.GRAB_CORAL_INCHES))
                                .whileTrue(rollers.setRollerVoltage(3))
                                .and(() -> rollers.getHasGamepiece())
                                // need to experiment with good voltage that will keep coral in
                                .onTrue(rollers.setRollerVoltage(0.5))
                                .onTrue(transfer.setVoltage(() -> 0))
                                .onTrue(this.forceState(SuperState.READY_SCORE_CORAL));

                stateTriggers.get(SuperState.REMOVE_ALGAE_HIGH)
                                .whileTrue(elevator.goToPositionInInches(() -> Elevator.REMOVE_ALGAE_HIGH_INCHES))
                                .whileTrue(rollers.setRollerVoltage(3))
                                .whileTrue(arm.rotateToPositionCommand(() -> Arm.REMOVE_ALGAE_HIGH_DEGREES))
                                .and(() -> rollers.getHasGamepiece())
                                .onTrue(this.forceState(SuperState.READY_SCORE_ALGAE)
                                                // need to experiment with good voltage that will keep algae in
                                                .alongWith(rollers.setRollerVoltage(0.5)));

                stateTriggers.get(SuperState.REMOVE_ALGAE_LOW)
                                .whileTrue(elevator.goToPositionInInches(() -> Elevator.REMOVE_ALGAE_LOW_INCHES))
                                .whileTrue(rollers.setRollerVoltage(3))
                                .whileTrue(arm.rotateToPositionCommand(() -> Arm.REMOVE_ALGAE_LOW_DEGREES))
                                .and(() -> rollers.getHasGamepiece())
                                .onTrue(this.forceState(SuperState.READY_SCORE_ALGAE)
                                                // again, experiment
                                                .alongWith(rollers.setRollerVoltage(0.5)));

                // when we are in ready mode, meaning arm has a coral, we now need to check for
                // when driver presses button to actually go to scoring location

                // THE TRIGGER IS ON FALSE BECAUSE IF IT IS ONTRUE FOR THIS ONE, IT IMMEDIATELY
                // JUMPS PAST THE PRE LEVELS
                // THIS IS BECAUSE EVERYTHING IS TRUE AT ONCE AND IT IMMEDIATELY GOES TO THE
                // NEXT STATE
                // AT FALSE FORCES TO WAIT UNTIL BUTTON IS ACTUALLY RELEASED, ON FALSE ONLY
                // TRIGGERS TRUE WHEN SWITCHINGG FROM TRUE TO FALSE
                stateTriggers.get(SuperState.READY_SCORE_CORAL)
                                .and(() -> levelTarget.get() == LevelTarget.L1)
                                .and(preScoreTrigger)
                                .onFalse(this.forceState(SuperState.PRE_L1));

                stateTriggers.get(SuperState.READY_SCORE_CORAL)
                                .and(() -> levelTarget.get() == LevelTarget.L2)
                                .and(preScoreTrigger)
                                .onFalse(this.forceState(SuperState.PRE_L2));

                stateTriggers.get(SuperState.READY_SCORE_CORAL)
                                .and(() -> levelTarget.get() == LevelTarget.L3)
                                .and(preScoreTrigger)
                                .onFalse(this.forceState(SuperState.PRE_L3));

                stateTriggers.get(SuperState.READY_SCORE_CORAL)
                                .and(() -> levelTarget.get() == LevelTarget.L4)
                                .and(preScoreTrigger)
                                .onFalse(this.forceState(SuperState.PRE_L4));

                /*
                 * send elevator/arm to correct level for pre-level scoring, transitions state
                 * when driver presses scoring button
                 */
                stateTriggers.get(SuperState.PRE_L1)
                                .whileTrue(elevator.goToPositionInInches(() -> Elevator.PRE_L1_INCHES))
                                .whileTrue(arm.rotateToPositionCommand(() -> Arm.PRE_L1_DEGREES))
                                .and(() -> elevator.isElevatorAtTarget())
                                .and(() -> arm.isMotorAtTarget())
                                .and(scoreTrigger)
                                .onFalse(this.forceState(SuperState.SCORE_CORAL));

                stateTriggers.get(SuperState.PRE_L2)
                                .whileTrue(elevator.goToPositionInInches(() -> Elevator.PRE_L2_INCHES))
                                .whileTrue(arm.rotateToPositionCommand(() -> Arm.PRE_L2_DEGREES))
                                .and(() -> elevator.isElevatorAtTarget())
                                .and(() -> arm.isMotorAtTarget())
                                .and(scoreTrigger)
                                .onFalse(this.forceState(SuperState.SCORE_CORAL));

                stateTriggers.get(SuperState.PRE_L3)
                                .whileTrue(elevator.goToPositionInInches(() -> Elevator.PRE_L3_INCHES))
                                .whileTrue(arm.rotateToPositionCommand(() -> Arm.PRE_L3_DEGREES))
                                .and(() -> elevator.isElevatorAtTarget())
                                .and(() -> arm.isMotorAtTarget())
                                .and(scoreTrigger)
                                .onFalse(this.forceState(SuperState.SCORE_CORAL));

                stateTriggers.get(SuperState.PRE_L4)
                                .whileTrue(elevator.goToPositionInInches(() -> Elevator.PRE_L4_INCHES))
                                .whileTrue(arm.rotateToPositionCommand(() -> Arm.PRE_L4_DEGREES))
                                .and(() -> elevator.isElevatorAtTarget())
                                .and(() -> arm.isMotorAtTarget())
                                .and(scoreTrigger)
                                .onFalse(this.forceState(SuperState.SCORE_CORAL));

                // when driver presses button to score, we lower elevator/arm to scoring level,
                // then command rollers to spit
                stateTriggers.get(SuperState.SCORE_CORAL)
                                .and(() -> levelTarget.get() == LevelTarget.L1)
                                .whileTrue(elevator.goToPositionInInches(() -> Elevator.SCORE_L1_INCHES))
                                .whileTrue(arm.rotateToPositionCommand(() -> Arm.SCORE_L1_DEGREES))
                                .and(() -> elevator.isElevatorAtTarget())
                                .and(() -> arm.isMotorAtTarget())
                                .and(outtakeTrigger)
                                .onTrue(this.forceState(SuperState.SPIT));

                stateTriggers.get(SuperState.SCORE_CORAL)
                                .and(() -> levelTarget.get() == LevelTarget.L2)
                                .whileTrue(elevator.goToPositionInInches(() -> Elevator.SCORE_L2_INCHES))
                                .whileTrue(arm.rotateToPositionCommand(() -> Arm.SCORE_L2_DEGREES))
                                .and(() -> elevator.isElevatorAtTarget())
                                .and(() -> arm.isMotorAtTarget())
                                .and(outtakeTrigger)
                                .onTrue(this.forceState(SuperState.SPIT));

                stateTriggers.get(SuperState.SCORE_CORAL)
                                .and(() -> levelTarget.get() == LevelTarget.L3)
                                .whileTrue(elevator.goToPositionInInches(() -> Elevator.SCORE_L3_INCHES))
                                .whileTrue(arm.rotateToPositionCommand(() -> Arm.SCORE_L3_DEGREES))
                                .and(() -> elevator.isElevatorAtTarget())
                                .and(() -> arm.isMotorAtTarget())
                                .and(outtakeTrigger)
                                .onTrue(this.forceState(SuperState.SPIT));

                stateTriggers.get(SuperState.SCORE_CORAL)
                                .and(() -> levelTarget.get() == LevelTarget.L4)
                                .whileTrue(elevator.goToPositionInInches(() -> Elevator.SCORE_L4_INCHES))
                                .whileTrue(arm.rotateToPositionCommand(() -> Arm.SCORE_L4_DEGREES))
                                .and(() -> elevator.isElevatorAtTarget())
                                .and(() -> arm.isMotorAtTarget())
                                .and(outtakeTrigger)
                                .onTrue(this.forceState(SuperState.SPIT));

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
                stateTriggers.get(SuperState.SPIT)
                                .onTrue(rollers.setRollerVoltage(-3))
                                .and(() -> !rollers.getHasGamepiece())
                                .and(() -> true)
                                .onTrue(this.forceState(SuperState.IDLE));

                // algae scoring
                stateTriggers.get(SuperState.READY_SCORE_ALGAE)
                                .and(() -> algaeTarget.get() == AlgaeTarget.PROCESSOR)
                                .and(preScoreTrigger)
                                .onFalse(this.forceState(SuperState.PRE_PROCESSOR));

                stateTriggers.get(SuperState.READY_SCORE_ALGAE)
                                .and(() -> algaeTarget.get() == AlgaeTarget.NET)
                                .and(preScoreTrigger)
                                .onFalse(this.forceState(SuperState.PRE_NET));

                stateTriggers.get(SuperState.PRE_PROCESSOR)
                                .whileTrue(elevator.goToPositionInInches(() -> Elevator.PRE_PROCESSOR_INCHES))
                                .whileTrue(arm.rotateToPositionCommand(() -> Arm.PRE_PROCESSOR_DEGREES))
                                .and(() -> elevator.isElevatorAtTarget())
                                .and(() -> arm.isMotorAtTarget())
                                .and(scoreTrigger)
                                .onFalse(this.forceState(SuperState.SPIT));

                stateTriggers.get(SuperState.PRE_NET)
                                .whileTrue(elevator.goToPositionInInches(() -> Elevator.PRE_NET_INCHES))
                                .whileTrue(arm.rotateToPositionCommand(() -> Arm.PRE_NET_DEGREES))
                                .and(() -> elevator.isElevatorAtTarget())
                                .and(() -> arm.isMotorAtTarget())
                                .and(scoreTrigger)
                                .onFalse(this.forceState(SuperState.SPIT));

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

        public void configureTriggers(Trigger preScoreTrigger, Trigger scoreTrigger, Trigger intakeTrigger,
                        Trigger outtakeTrigger, Trigger missedScoreTrigger, Trigger stowTrigger,
                        Trigger removeAlgaeHighTrigger, Trigger removeAlgaeLowTrigger,
                        Trigger cancelScoringTrigger) {
                this.preScoreTrigger = preScoreTrigger;
                this.scoreTrigger = scoreTrigger;
                this.intakeTrigger = intakeTrigger;
                this.outtakeTrigger = outtakeTrigger;
                this.missedScoreTrigger = missedScoreTrigger;
                this.stowTrigger = stowTrigger;
                this.removeAlgaeHighTrigger = removeAlgaeHighTrigger;
                this.removeAlgaeLowTrigger = removeAlgaeLowTrigger;
                this.cancelScoringTrigger = cancelScoringTrigger;

        }

}
