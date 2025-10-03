package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.playingwithfusion.TimeOfFlight.Status;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer.LevelTarget;
import frc.robot.RobotContainer.AlgaeTarget;

@Logged
public class Superstructure {

	public enum SuperState {
		INTAKE,
		INTAKE_RESET,
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
		HOMING,
		SPIT_ALGAE,
		SAFE_MODE,
		TRANSFER,
		THROW
	}

	private final Supplier<LevelTarget> levelTarget;
	private final Supplier<AlgaeTarget> algaeTarget;

	// trigger for states. these should be things that cause robot to move, not
	// buttons that change target level
	private final Trigger preScoreTrigger;
	private final Trigger scoreTrigger;
	private final Trigger intakeTrigger;
	private final Trigger outtakeTrigger;
	private final Trigger missedScoreTrigger;
	private final Trigger stowTrigger;
	private final Trigger removeAlgaeHighTrigger;
	private final Trigger removeAlgaeLowTrigger;
	private final Trigger forceIdleTrigger;
	private final Trigger overrideBeamBreakTrigger;
	private final Trigger homeTrigger;
	private final DoubleSupplier intakeSpeed;
	private final Trigger intakeResetTrigger;

	@Logged
	private final Trigger gamepieceDetectedInStagingArea;

	private final DoubleConsumer rumble;

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
	private final Intake intake;

	public Superstructure(Elevator elevator,
			Rollers rollers,
			Arm arm,
			Transfer transfer,
			Leds leds,
			Intake intake,
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
			Trigger forceIdleTrigger,
			Trigger overrideBeamBreakTrigger,
			Trigger homeTrigger,
			DoubleSupplier intakeSpeed,
			DoubleConsumer rumble,
			Trigger intakeResetTrigger) {
		this.elevator = elevator;
		this.rollers = rollers;
		this.arm = arm;
		this.transfer = transfer;
		this.leds = leds;
		this.intake = intake;

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
		this.forceIdleTrigger = forceIdleTrigger;
		this.overrideBeamBreakTrigger = overrideBeamBreakTrigger;
		this.homeTrigger = homeTrigger;
		this.intakeSpeed = intakeSpeed;
		this.rumble = rumble;
		this.intakeResetTrigger = intakeResetTrigger;

		tof = new TimeOfFlight(0);
		tof.setRangingMode(RangingMode.Short, 24);
		gamepieceDetectedInStagingArea = new Trigger(() -> tof.getRange() < 80)
				.and(() -> tof.getStatus() == Status.Valid).debounce(0.2);

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
		forceIdleTrigger
				.onTrue(this.forceState(SuperState.IDLE)
						.alongWith(elevator.stopElevator())
						.alongWith(arm.neutral())
						.alongWith(rollers.setRollerVoltage(-1.5)));

		// Idle -> stow
		stateTriggers.get(SuperState.IDLE)
				.whileTrue(transfer.setVoltage(() -> 0))
				// ! .whileTrue(rollers.setRollerVoltage(() -> -4))
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

		// home elevator
		stateTriggers.get(SuperState.HOMING)
				// make sure arm isnt down before we force the elevator to low hardstops
				.whileTrue(arm.rotateToPositionCommand(() -> Arm.STOWED_DEGREES))
				.and(() -> arm.isMotorAtTarget())
				// home elevator and force back to idle
				.whileTrue(elevator.homeElevator())
				.and(() -> elevator.getIsHomed())
				.onTrue(this.forceState(SuperState.IDLE));

		stateTriggers.get(SuperState.STOW)
				.whileTrue(elevator.goToPosition(() -> Elevator.STOWED_ROTATIONS))
				.and(() -> elevator.getElevatorMotorPosition() > 11.5)
				.onTrue(arm.rotateToPositionCommand(() -> Arm.STOWED_DEGREES))
				// if we are stowing, we dont really care if we reach stow position before going
				// to intake
				// so, we just check intake trigger if we need to intake
				.and(intakeTrigger)
				.onTrue(this.forceState(SuperState.INTAKE));

		// Stow -> removing algae state
		stateTriggers.get(SuperState.STOW)
				.and(removeAlgaeHighTrigger)
				.onTrue(this.forceState(SuperState.REMOVE_ALGAE_HIGH));
		stateTriggers.get(SuperState.STOW)
				.and(removeAlgaeLowTrigger)
				.onTrue(this.forceState(SuperState.REMOVE_ALGAE_LOW));

		stateTriggers.get(SuperState.INTAKE)
				.whileTrue(elevator.goToPosition(() -> Elevator.STOWED_ROTATIONS))
				.and(() -> elevator.isElevatorAtTarget())
				.whileTrue(transfer.pulseVoltage(1)) // todo: test voltage that works
				.whileTrue(arm.rotateToPositionCommand(() -> Arm.INTAKE_HP_DEGREES))
				.and(() -> arm.isMotorAtTarget())
				.whileTrue(elevator.goToPosition(() -> Elevator.INTAKE_HP_ROTATIONS))
				.whileTrue(rollers.setRollerVoltage(() -> 1.5))
				.and(gamepieceDetectedInStagingArea)
				.onTrue(this.forceState(SuperState.GRAB_CORAL));

		stateTriggers.get(SuperState.INTAKE)
				.and(() -> DriverStation.isAutonomous())
				.whileTrue(intake.setIntakeVoltage(() -> 12));

		stateTriggers.get(SuperState.INTAKE)
				.and(() -> DriverStation.isTeleop())
				.and(() -> Math.abs(intakeSpeed.getAsDouble()) > 0.1)
				.whileTrue(intake.rotateToPositionCommand(Intake.DEPLOYED_ANGLE))
				// .and(() -> intake.atPosition())
				.whileTrue(intake.setIntakeVoltage(() -> (intakeSpeed.getAsDouble() * 12)));

		stateTriggers.get(SuperState.INTAKE)
				.and(() -> DriverStation.isTeleop())
				.and(() -> Math.abs(intakeSpeed.getAsDouble()) < 0.1)
				.onTrue(intake.setIntakeVoltage(() -> 0))
				.onTrue(intake.rotateToPositionCommand(Intake.STOWED_ANGLE))
				.and(() -> intake.atPosition());
		

		// ! Manual intake resetting
		stateTriggers.get(SuperState.INTAKE)
				.and(() -> DriverStation.isTeleop())
				.and(intakeResetTrigger)
				.onTrue(forceState(SuperState.INTAKE_RESET));

		// intake unjam state
		stateTriggers.get(SuperState.INTAKE)
				.and(missedScoreTrigger)
				.onFalse(this.forceState(SuperState.UNJAM));

		stateTriggers.get(SuperState.INTAKE)
				.and(() -> transfer.getLeftStatorCurrent() > 30 || transfer.getRightStatorCurrent() > 30).debounce(0.15)
				.onTrue(this.forceState(SuperState.UNJAM));

		stateTriggers.get(SuperState.TRANSFER)
				.onTrue(intake.setIntakeVoltage(() -> 0))
				.onTrue(
						transfer.setVoltage(() -> -1).andThen(
								new WaitCommand(0.5),
								transfer.pulseVoltage(1)))
				.and(gamepieceDetectedInStagingArea.or(overrideBeamBreakTrigger))
				.onTrue(transfer.setVoltage(() -> 0))
				.onTrue(this.forceState(SuperState.GRAB_CORAL));

		stateTriggers.get(SuperState.UNJAM)
				// move arm up to flat out and reverse transfer
				// then we try to intake again 1 second later
				.whileTrue(elevator.goToPosition(() -> 12.75))
				.and(() -> elevator.isElevatorAtTarget())
				.whileTrue(arm.rotateToPositionCommand(() -> -60))
				.whileTrue(transfer.setVoltage(() -> -1.5))
				.onTrue(Commands.sequence(new WaitCommand(0.5), transfer.setVoltage(() -> 0),
						this.forceState(SuperState.INTAKE)));

		stateTriggers.get(SuperState.READY_SCORE_CORAL)
				.and(gamepieceDetectedInStagingArea).debounce(0.5)
				.onTrue(this.forceState(SuperState.GRAB_CORAL));

		stateTriggers.get(SuperState.READY_SCORE_CORAL)
				.and(intakeTrigger)
				.onFalse(this.forceState(SuperState.GRAB_CORAL));

		stateTriggers.get(SuperState.GRAB_CORAL)
				// just drop elevator down and bring it back up
				.whileTrue(elevator.goToPosition(() -> Elevator.GRAB_CORAL_ROTATIONS))
				.whileTrue(rollers.setRollerVoltage(9))
				.whileTrue(intake.setIntakeVoltage(0))
				.whileTrue(intake.rotateToPositionCommand(Intake.STOWED_ANGLE))
				.onTrue(Commands.runOnce(() -> rumble.accept(0.5)).andThen(new WaitCommand(0.5),
						Commands.runOnce(() -> rumble.accept(0))))
				.onTrue(leds.blink())

				.and(() -> elevator.isElevatorAtTarget())
				.onTrue(new WaitCommand(0.25).andThen(rollers.setRollerVoltage(0.75)))

				.onTrue(this.forceState(SuperState.READY_SCORE_CORAL));

		stateTriggers.get(SuperState.GRAB_CORAL)
				.and(() -> elevator.getStatorCurrent() > 40).debounce(0.5)
				.onTrue(this.forceState(SuperState.SAFE_MODE));

		stateTriggers.get(SuperState.SAFE_MODE)
				.whileTrue(elevator.goToPosition(() -> Elevator.STOWED_ROTATIONS))
				.and(() -> elevator.getElevatorMotorPosition() > 11.5)
				.onTrue(arm.rotateToPositionCommand(() -> Arm.STOWED_DEGREES))
				.and(preScoreTrigger)
				.onTrue(this.forceState(SuperState.READY_SCORE_CORAL));

		stateTriggers.get(SuperState.SAFE_MODE)
				.and(intakeTrigger)
				.onTrue(this.forceState(SuperState.INTAKE));

		stateTriggers.get(SuperState.REMOVE_ALGAE_HIGH)
				.whileTrue(elevator.goToPosition(() -> Elevator.REMOVE_ALGAE_HIGH_ROTATIONS))
				.whileTrue(rollers.setRollerVoltage(12))
				.whileTrue(arm.rotateToPositionCommand(() -> Arm.REMOVE_ALGAE_HIGH_DEGREES))
				.and(() -> rollers.getHasGamepiece() || overrideBeamBreakTrigger.getAsBoolean())
				.onTrue(this.forceState(SuperState.READY_SCORE_ALGAE)



						// need to experiment with good voltage that will keep algae in
						.alongWith(rollers.setRollerVoltage(8)));

		stateTriggers.get(SuperState.REMOVE_ALGAE_LOW)
				.whileTrue(elevator.goToPosition(() -> Elevator.REMOVE_ALGAE_LOW_ROTATIONS))
				.whileTrue(rollers.setRollerVoltage(12))
				.whileTrue(arm.rotateToPositionCommand(() -> Arm.REMOVE_ALGAE_LOW_DEGREES))
				.and(() -> rollers.getHasGamepiece() || overrideBeamBreakTrigger.getAsBoolean())
				.onTrue(this.forceState(SuperState.READY_SCORE_ALGAE)
						// again, experiment
						.alongWith(rollers.setRollerVoltage(8)));

		// flip flop between states
		stateTriggers.get(SuperState.REMOVE_ALGAE_LOW)
				.and(removeAlgaeHighTrigger)
				.onTrue(this.forceState(SuperState.REMOVE_ALGAE_HIGH));

		stateTriggers.get(SuperState.REMOVE_ALGAE_HIGH)
				.and(removeAlgaeLowTrigger)
				.onTrue(this.forceState(SuperState.REMOVE_ALGAE_LOW));

		// when we are in ready mode, meaning arm has a coral, we now need to check for
		// when driver presses button to actually go to scoring location

		// THE TRIGGER IS ONFALSE BECAUSE IF IT IS ONTRUE FOR THIS ONE, IT IMMEDIATELY
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
				// .whileTrue(elevator.goToPosition(() -> 10.5))
				// // make sure arm doesnt collide with anything
				// .and(() -> elevator.isElevatorAtTarget())
				.onTrue(arm.rotateToPositionCommand(() -> Arm.PRE_L2_DEGREES))
				.and(() -> arm.getArmPosition() > 0)
				.onTrue(elevator.goToPosition(() -> Elevator.PRE_L2_ROTATIONS))
				// .and(() -> arm.isMotorAtTarget())
				// .and(() -> elevator.isElevatorAtTarget())
				.and(scoreTrigger)
				.onFalse(this.forceState(SuperState.SCORE_CORAL));

		stateTriggers.get(SuperState.PRE_L3)
				.whileTrue(elevator.goToPosition(() -> Elevator.PRE_L3_ROTATIONS))
				.onTrue(arm.rotateToPositionCommand(() -> Arm.PRE_L3_DEGREES))
				// .and(() -> elevator.isElevatorAtTarget())
				// .and(() -> arm.isMotorAtTarget())
				.and(scoreTrigger)
				.onFalse(this.forceState(SuperState.SCORE_CORAL));

		stateTriggers.get(SuperState.PRE_L4)
				.whileTrue(elevator.goToPosition(() -> Elevator.PRE_L4_ROTATIONS))
				.whileTrue(arm.rotateToPositionCommand(() -> Arm.PRE_L4_DEGREES))
				// .and(() -> elevator.isElevatorAtTarget())
				// .and(() -> arm.isMotorAtTarget())
				.and(scoreTrigger)
				.onFalse(this.forceState(SuperState.SCORE_CORAL));

		// pre-level back to ready score coral
		stateTriggers.get(SuperState.PRE_L1)
				.and(stowTrigger)
				.onFalse(this.forceState(SuperState.READY_SCORE_CORAL));

		stateTriggers.get(SuperState.PRE_L2)
				.and(stowTrigger)
				.onFalse(this.forceState(SuperState.READY_SCORE_CORAL));

		stateTriggers.get(SuperState.PRE_L3)
				.and(stowTrigger)
				.onFalse(this.forceState(SuperState.READY_SCORE_CORAL));

		stateTriggers.get(SuperState.PRE_L4)
				.and(stowTrigger)
				.onFalse(this.forceState(SuperState.READY_SCORE_CORAL));

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
				// .and(() -> elevator.isElevatorAtTarget())
				// .and(() -> arm.isMotorAtTarget())
				.and(outtakeTrigger)
				.onTrue(this.forceState(SuperState.SPIT));

		stateTriggers.get(SuperState.SCORE_CORAL)
				.and(() -> levelTarget.get() == LevelTarget.L3)
				.whileTrue(elevator.goToPosition(() -> Elevator.SCORE_L3_ROTATIONS))
				.whileTrue(arm.rotateToPositionCommand(() -> Arm.SCORE_L3_DEGREES))
				// .and(() -> elevator.isElevatorAtTarget())
				// .and(() -> arm.isMotorAtTarget())
				.and(outtakeTrigger)
				.onTrue(elevator.goToPosition(() -> 13))
				.onTrue(this.forceState(SuperState.SPIT));

		stateTriggers.get(SuperState.SCORE_CORAL)
				.and(() -> levelTarget.get() == LevelTarget.L4)
				.whileTrue(elevator.goToPosition(() -> Elevator.SCORE_L4_ROTATIONS))
				.whileTrue(arm.rotateToPositionCommand(() -> Arm.SCORE_L4_DEGREES))
				.and(() -> elevator.isElevatorAtTarget())
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
				.onTrue(rollers.setRollerVoltage(-1.5))
				.and(stowTrigger)
				.onFalse(this.forceState(SuperState.STOW));

		// algae scoring
		stateTriggers.get(SuperState.READY_SCORE_ALGAE)
				.and(() -> algaeTarget.get() == AlgaeTarget.PROCESSOR)
				.and(preScoreTrigger)
				.onFalse(this.forceState(SuperState.PRE_PROCESSOR));

		// eject algae immediately instead of scoring
		stateTriggers.get(SuperState.READY_SCORE_ALGAE)
				.and(stowTrigger)
				.onFalse(rollers.setRollerVoltage(-6)
						.andThen(new WaitCommand(0.5), this.forceState(SuperState.STOW)));

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
				.onFalse(this.forceState(SuperState.SPIT_ALGAE));

		stateTriggers.get(SuperState.PRE_NET)
				.whileTrue(rollers.setRollerVoltage(12))
				.whileTrue(elevator.goToPosition(() -> Elevator.PRE_NET_ROTATIONS))
				.onTrue(arm.rotateToPositionCommand(() -> 35))
				.and(preScoreTrigger)
				.onFalse(
						this.forceState(SuperState.THROW));

		stateTriggers.get(SuperState.THROW)
				.onTrue(arm.rotateToPositionCommand(() -> 80))
				.and(() -> arm.getArmPosition() > 72)
				.onTrue(this.forceState(SuperState.SPIT_ALGAE));

		// stateTriggers.get(SuperState.PRE_NET)
		// .whileTrue(rollers.setRollerVoltage(12))
		// .whileTrue(elevator.goToPosition(() -> 43.5))
		// .whileTrue(arm.rotateToPositionCommand(() -> 80))
		// .and(scoreTrigger)
		// .onFalse(this.forceState(SuperState.SPIT_ALGAE));

		stateTriggers.get(SuperState.SPIT_ALGAE)
				.onTrue(rollers.setRollerVoltage(-9))
				.and(stowTrigger)
				.onTrue(this.forceState(SuperState.STOW).alongWith(rollers.setRollerVoltage(0)));


		stateTriggers.get(SuperState.INTAKE_RESET)
				.onTrue(intake.smartResetDeployMotor().andThen(forceState(SuperState.STOW)));
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
