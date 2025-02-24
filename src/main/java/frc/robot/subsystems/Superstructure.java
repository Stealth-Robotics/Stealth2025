package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
		IDLE,
		UNJAM,
		HOMING
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
	private Trigger overrideBeamBreakTrigger;
	private Trigger homeTrigger;
	@Logged
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
	private final Leds leds;

	public Superstructure(Elevator elevator,
			Rollers rollers,
			Arm arm,
			Transfer transfer,
			Leds leds,
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
			Trigger cancelScoringTrigger,
			Trigger overrideBeamBreakTrigger,
			Trigger homeTrigger) {
		this.elevator = elevator;
		this.rollers = rollers;
		this.arm = arm;
		this.transfer = transfer;
		this.leds = leds;
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
		this.overrideBeamBreakTrigger = overrideBeamBreakTrigger;
		this.homeTrigger = homeTrigger;

		// TODO FIND CAN ID
		tof = new TimeOfFlight(0);
		tof.setRangingMode(RangingMode.Short, 0.02);
		// TODO FIND RANGE THAT WORKS WITH TIME OF FLIGHT

		gamepieceDetectedInStagingArea = new Trigger(() -> tof.getRange() < 80).debounce(0.1);

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
				.whileTrue(transfer.setVoltage(() -> 0))
				.whileTrue(rollers.setRollerVoltage(() -> 0))
				// stop subsystems when we are in idle state
				// .whileTrue(elevator.stopElevator())
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

		stateTriggers.get(SuperState.IDLE)
				.and(homeTrigger)
				.onFalse(this.forceState(SuperState.HOMING));

		stateTriggers.get(SuperState.HOMING)
				.whileTrue(arm.rotateToPositionCommand(() -> Arm.STOWED_DEGREES))
				.and(() -> arm.isMotorAtTarget())
				.whileTrue(elevator.homeElevator())
				.and(() -> elevator.getIsHomed()).debounce(0.2)
				.onTrue(this.forceState(SuperState.IDLE));

		stateTriggers.get(SuperState.STOW)
				.whileTrue(elevator.goToPosition(() -> Elevator.STOWED_ROTATIONS))
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
				.whileTrue(elevator.goToPosition(() -> Elevator.INTAKE_HP_ROTATIONS))
				.whileTrue(arm.rotateToPositionCommand(() -> Arm.INTAKE_HP_DEGREES))
				.whileTrue(transfer.setVoltage(() -> 1)) // todo: test voltage that works
				.and(gamepieceDetectedInStagingArea.or(overrideBeamBreakTrigger))
				.onTrue(transfer.setVoltage(() -> 0))
				.and(() -> arm.isMotorAtTarget())
				.onTrue(this.forceState(SuperState.GRAB_CORAL));

		stateTriggers.get(SuperState.INTAKE_HP)
				.and(cancelScoringTrigger)
				.onFalse(this.forceState(SuperState.UNJAM));

		stateTriggers.get(SuperState.UNJAM)
				.whileTrue(arm.rotateToPositionCommand(() -> 0))
				.whileTrue(transfer.setVoltage(() -> -2))
				.onTrue(Commands.sequence(new WaitCommand(1), this.forceState(SuperState.INTAKE_HP)));

		stateTriggers.get(SuperState.GRAB_CORAL)
				.whileTrue(elevator.goToPosition(() -> Elevator.GRAB_CORAL_ROTATIONS))
				.whileTrue(rollers.setRollerVoltage(6))
				// .onTrue(leds.blink())

				// .and(() -> rollers.getHasGamepiece())
				// need to experiment with good voltage that will keep coral in
				.and(() -> elevator.isElevatorAtTarget())
				.onTrue(rollers.setRollerVoltage(0.6))

				.onTrue(this.forceState(SuperState.READY_SCORE_CORAL));

		stateTriggers.get(SuperState.REMOVE_ALGAE_HIGH)
				.whileTrue(elevator.goToPosition(() -> Elevator.REMOVE_ALGAE_HIGH_ROTATIONS))
				.whileTrue(rollers.setRollerVoltage(9))
				.whileTrue(arm.rotateToPositionCommand(() -> Arm.REMOVE_ALGAE_HIGH_DEGREES))
				.and(() -> rollers.getHasGamepiece())
				.onTrue(this.forceState(SuperState.READY_SCORE_ALGAE)
						// need to experiment with good voltage that will keep algae in
						.alongWith(rollers.setRollerVoltage(8)));

		stateTriggers.get(SuperState.REMOVE_ALGAE_LOW)
				.whileTrue(elevator.goToPosition(() -> Elevator.REMOVE_ALGAE_LOW_ROTATIONS))
				.whileTrue(rollers.setRollerVoltage(9))
				.whileTrue(arm.rotateToPositionCommand(() -> Arm.REMOVE_ALGAE_LOW_DEGREES))
				.and(() -> rollers.getHasGamepiece())
				.onTrue(this.forceState(SuperState.READY_SCORE_ALGAE)
						// again, experiment
						.alongWith(rollers.setRollerVoltage(8)));

		// when we are in ready mode, meaning arm has a coral, we now need to check for
		// when driver presses button to actually go to scoring location

		// THE TRIGGER IS ON FALSE BECAUSE IF IT IS ONTRUE FOR THIS ONE, IT IMMEDIATELY
		// JUMPS PAST THE PRE LEVELS
		// THIS IS BECAUSE EVERYTHING IS TRUE AT ONCE AND IT IMMEDIATELY GOES TO THE
		// NEXT STATE
		// AT FALSE FORCES TO WAIT UNTIL BUTTON IS ACTUALLY RELEASED, ON FALSE ONLY
		// TRIGGERS TRUE WHEN SWITCHINGG FROM TRUE TO FALSE
		stateTriggers.get(SuperState.READY_SCORE_CORAL)
				.whileTrue(elevator.goToPosition(() -> Elevator.STOWED_ROTATIONS))
				.and(() -> levelTarget.get() == LevelTarget.L1)
				.and(preScoreTrigger)
				.onFalse(this.forceState(SuperState.PRE_L1));

		stateTriggers.get(SuperState.READY_SCORE_CORAL)
				.whileTrue(elevator.goToPosition(() -> Elevator.STOWED_ROTATIONS))
				.and(() -> levelTarget.get() == LevelTarget.L2)
				.and(preScoreTrigger)
				.onFalse(this.forceState(SuperState.PRE_L2));

		stateTriggers.get(SuperState.READY_SCORE_CORAL)
				.whileTrue(elevator.goToPosition(() -> Elevator.STOWED_ROTATIONS))
				.and(() -> levelTarget.get() == LevelTarget.L3)
				.and(preScoreTrigger)
				.onFalse(this.forceState(SuperState.PRE_L3));

		stateTriggers.get(SuperState.READY_SCORE_CORAL)
				.whileTrue(elevator.goToPosition(() -> Elevator.STOWED_ROTATIONS))
				.and(() -> levelTarget.get() == LevelTarget.L4)
				.and(preScoreTrigger)
				.onFalse(this.forceState(SuperState.PRE_L4));

		/*
		 * send elevator/arm to correct level for pre-level scoring, transitions state
		 * when driver presses scoring button
		 */
		stateTriggers.get(SuperState.PRE_L1)
				.whileTrue(elevator.goToPosition(() -> Elevator.PRE_L1_ROTATIONS))
				.whileTrue(arm.rotateToPositionCommand(() -> Arm.PRE_L1_DEGREES))
				.and(() -> elevator.isElevatorAtTarget())
				.and(() -> arm.isMotorAtTarget())
				.and(scoreTrigger)
				.onFalse(this.forceState(SuperState.SCORE_CORAL));

		stateTriggers.get(SuperState.PRE_L2)
				.whileTrue(elevator.goToPosition(() -> Elevator.INTAKE_HP_ROTATIONS))

				.and(() -> elevator.isElevatorAtTarget())
				.whileTrue(arm.rotateToPositionCommand(() -> Arm.PRE_L2_DEGREES))
				.whileTrue(elevator.goToPosition(() -> Elevator.PRE_L2_ROTATIONS))
				.and(() -> arm.isMotorAtTarget())
				.and(scoreTrigger)
				.onFalse(this.forceState(SuperState.SCORE_CORAL));

		stateTriggers.get(SuperState.PRE_L3)
				.whileTrue(elevator.goToPosition(() -> Elevator.PRE_L3_ROTATIONS))
				.whileTrue(arm.rotateToPositionCommand(() -> Arm.PRE_L3_DEGREES))
				.and(() -> elevator.isElevatorAtTarget())
				.and(() -> arm.isMotorAtTarget())
				.and(scoreTrigger)
				.onFalse(this.forceState(SuperState.SCORE_CORAL));

		stateTriggers.get(SuperState.PRE_L4)
				.whileTrue(elevator.goToPosition(() -> Elevator.PRE_L4_ROTATIONS))
				.whileTrue(arm.rotateToPositionCommand(() -> Arm.PRE_L4_DEGREES))
				.and(() -> elevator.isElevatorAtTarget())
				// .and(() -> arm.isMotorAtTarget())
				.and(scoreTrigger)
				.onFalse(this.forceState(SuperState.SCORE_CORAL));

		// when driver presses button to score, we lower elevator/arm to scoring level,
		// then command rollers to spit
		stateTriggers.get(SuperState.SCORE_CORAL)
				.and(() -> levelTarget.get() == LevelTarget.L1)
				.whileTrue(elevator.goToPosition(() -> Elevator.SCORE_L1_ROTATIONS))
				.whileTrue(arm.rotateToPositionCommand(() -> Arm.SCORE_L1_DEGREES))
				.and(() -> elevator.isElevatorAtTarget())
				.and(() -> arm.isMotorAtTarget())
				.and(outtakeTrigger)
				.onTrue(this.forceState(SuperState.SPIT));

		stateTriggers.get(SuperState.SCORE_CORAL)
				.and(() -> levelTarget.get() == LevelTarget.L2)
				.whileTrue(elevator.goToPosition(() -> Elevator.SCORE_L2_ROTATIONS))
				.whileTrue(arm.rotateToPositionCommand(() -> Arm.SCORE_L2_DEGREES))
				.and(() -> elevator.isElevatorAtTarget())
				.and(() -> arm.isMotorAtTarget())
				.and(outtakeTrigger)
				.onTrue(this.forceState(SuperState.SPIT));

		stateTriggers.get(SuperState.SCORE_CORAL)
				.and(() -> levelTarget.get() == LevelTarget.L3)
				.whileTrue(elevator.goToPosition(() -> Elevator.SCORE_L3_ROTATIONS))
				.whileTrue(arm.rotateToPositionCommand(() -> Arm.SCORE_L3_DEGREES))
				.and(() -> elevator.isElevatorAtTarget())
				.and(() -> arm.isMotorAtTarget())
				.and(outtakeTrigger)
				.onTrue(this.forceState(SuperState.SPIT));

		stateTriggers.get(SuperState.SCORE_CORAL)
				.and(() -> levelTarget.get() == LevelTarget.L4)
				.whileTrue(elevator.goToPosition(() -> Elevator.SCORE_L4_ROTATIONS))
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
				.onTrue(rollers.setRollerVoltage(-1.5)
						.andThen(new WaitCommand(0.5).andThen(rollers.setRollerVoltage(0))
								.andThen(this.forceState(SuperState.IDLE))));

		// algae scoring
		stateTriggers.get(SuperState.READY_SCORE_ALGAE)
				// .onTrue(arm.rotateToPositionCommand(() -> Arm.READY_SCORE_ALGAE))
				.and(() -> algaeTarget.get() == AlgaeTarget.PROCESSOR)
				.and(preScoreTrigger)
				.onFalse(this.forceState(SuperState.PRE_PROCESSOR));

		stateTriggers.get(SuperState.READY_SCORE_ALGAE)
				.and(() -> algaeTarget.get() == AlgaeTarget.NET)
				.and(preScoreTrigger)
				.onFalse(this.forceState(SuperState.PRE_NET));

		stateTriggers.get(SuperState.PRE_PROCESSOR)
				.whileTrue(elevator.goToPosition(() -> Elevator.PRE_PROCESSOR_ROTATIONS))
				.whileTrue(arm.rotateToPositionCommand(() -> Arm.PRE_PROCESSOR_DEGREES))
				.and(() -> elevator.isElevatorAtTarget())
				.and(() -> arm.isMotorAtTarget())
				.and(scoreTrigger)
				.onFalse(this.forceState(SuperState.SPIT));

		stateTriggers.get(SuperState.PRE_NET)
				.whileTrue(elevator.goToPosition(() -> Elevator.PRE_NET_ROTATIONS))
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

	@Logged
	public double getTOFDistance() {
		return tof.getRange();
	}

}
