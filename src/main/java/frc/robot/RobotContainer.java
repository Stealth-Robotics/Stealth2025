// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveLogger;
import frc.robot.subsystems.Transfer;
import frc.robot.subsystems.CommandSwerveDrivetrain.ReefSide;
import frc.robot.subsystems.Superstructure.SuperState;

@Logged
public class RobotContainer {

	CommandXboxController driverController = new CommandXboxController(0);
	CommandXboxController operatorController = new CommandXboxController(1);

	@NotLogged
	private final double MAX_VELO = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
			MAX_ANGULAR_VELO = RotationsPerSecond.of(1.5).in(RadiansPerSecond);

	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(MAX_VELO * 0.1).withRotationalDeadband(MAX_ANGULAR_VELO * 0.1)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private final SwerveRequest.FieldCentricFacingAngle driveAngle = new SwerveRequest.FieldCentricFacingAngle()
			.withDeadband(MAX_VELO * 0.1).withRotationalDeadband(MAX_ANGULAR_VELO * 0.1)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
			.withTargetDirection(new Rotation2d(Radians.of(90).in(Degrees)));

	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

	Superstructure superstructure;
	Rollers rollers;
	Elevator elevator;
	Arm arm;
	Transfer transfer;
	Leds leds;
	Climber climber;
	CommandSwerveDrivetrain dt = TunerConstants.createDrivetrain();
	SwerveLogger logger = new SwerveLogger(dt);
	AutoFactory autoFactory;

	LevelTarget target = LevelTarget.L4;
	AlgaeTarget algaeTarget = AlgaeTarget.PROCESSOR;

	private final AutoChooser autoChooser;

	Command goToL4;
	Command dunk;
	Command intake;
	Command eject;

	Command driveFieldCentric;
	Command drivePointingAtAngle;

	boolean driveAngled = false;

	public Command leftAuto;
	Trigger subsystemsAtSetpoints;

	@Logged
	SwerveSample[] trajSamples;

	public RobotContainer() {
		// configure PID for heading controller
		driveAngle.HeadingController.setP(0.1);

		elevator = new Elevator();
		rollers = new Rollers(() -> superstructure.getState());
		arm = new Arm();
		transfer = new Transfer();
		leds = new Leds();
		climber = new Climber();

		subsystemsAtSetpoints = new Trigger(() -> elevator.isElevatorAtTarget())
				.and(() -> arm.isMotorAtTarget()).debounce(0.1);

		autoChooser = new AutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);
		autoFactory = dt.createAutoFactory();

		autoChooser.addRoutine("test auto", this::buildAuto);
		autoChooser.addRoutine("preload auto", this::preloadAuto);

		superstructure = new Superstructure(
				elevator,
				rollers,
				arm,
				transfer,
				leds,
				() -> target,
				() -> algaeTarget,
				driverController.leftBumper(),
				driverController.leftBumper(),
				driverController.rightBumper(),
				driverController.leftBumper(),
				driverController.rightTrigger(),
				driverController.rightBumper(),
				// TODO: BIND TO BUTTONS
				operatorController.leftBumper(),
				operatorController.rightBumper(),
				driverController.leftTrigger(),
				driverController.povLeft(),
				driverController.povRight(),
				(rumble) -> setRumble(rumble));

		climber.setDefaultCommand(climber.setDutyCycle(
				() -> operatorController.getRightTriggerAxis() - operatorController.getLeftTriggerAxis()));

		goToL4 = Commands.sequence(superstructure.forceState(SuperState.PRE_L4),
				new WaitUntilCommand(subsystemsAtSetpoints));
		dunk = Commands.sequence(superstructure.forceState(SuperState.SCORE_CORAL),
				new WaitUntilCommand(() -> elevator.isElevatorAtTarget()));
		eject = Commands.sequence(superstructure.forceState(SuperState.SPIT), new WaitCommand(0.5));
		intake = Commands.sequence(superstructure.forceState(SuperState.INTAKE_HP));

		autoChooser.addCmd("auto", this::buildLeftAuto);

		// NamedCommands.registerCommand("Go to scoring", goToL4);
		// NamedCommands.registerCommand("Dunk", dunk);
		// NamedCommands.registerCommand("Eject", eject);
		// NamedCommands.registerCommand("Intake", intake);

		rollers.configureStateSupplierTrigger();

		driveFieldCentric = dt.applyRequest(
				() -> drive.withVelocityY(-driverController.getLeftX() * MAX_VELO)
						.withVelocityX(-driverController.getLeftY() * MAX_VELO)
						.withRotationalRate(-driverController.getRightX() * MAX_ANGULAR_VELO))
				.alongWith(Commands.runOnce(() -> driveAngled = false));

		drivePointingAtAngle = dt.applyRequest(
				() -> driveAngle.withVelocityY(-driverController.getLeftX() * MAX_VELO)
						.withVelocityX(-driverController.getLeftY() * MAX_VELO))
				.alongWith(Commands.runOnce(() -> driveAngled = true));

		dt.setDefaultCommand(driveFieldCentric);

	}

	public enum LevelTarget {
		L1,
		L2,
		L3,
		L4
	}

	public enum AlgaeTarget {
		PROCESSOR,
		NET
	}

	public void configureIdleAnim() {
		CommandScheduler.getInstance().schedule(leds.rainbowAnim());
	}

	public void configureBindings() {
		// this gets run in teleopInit, so it should stop subsystems from moving on
		// enable
		CommandScheduler.getInstance().schedule(arm.neutral());
		CommandScheduler.getInstance().schedule(elevator.stopElevator());
		CommandScheduler.getInstance().schedule(superstructure.forceState(SuperState.IDLE));
		CommandScheduler.getInstance().schedule(Commands.runOnce(() -> leds.setLevel(target)));

		operatorController.a().onTrue(Commands.runOnce(() -> target = LevelTarget.L1)
				.alongWith(Commands.runOnce(() -> leds.setLevel(target))));
		operatorController.b().onTrue(Commands.runOnce(() -> target = LevelTarget.L2)
				.alongWith(Commands.runOnce(() -> leds.setLevel(target))));
		operatorController.x().onTrue(Commands.runOnce(() -> target = LevelTarget.L3)
				.alongWith(Commands.runOnce(() -> leds.setLevel(target))));
		operatorController.y().onTrue(Commands.runOnce(() -> target = LevelTarget.L4)
				.alongWith(Commands.runOnce(() -> leds.setLevel(target))));

		operatorController.povDown().onTrue(Commands.runOnce(() -> algaeTarget = AlgaeTarget.PROCESSOR));
		operatorController.povUp().onTrue(Commands.runOnce(() -> algaeTarget = AlgaeTarget.NET));

		driverController.povDown().onTrue(Commands.runOnce(() -> dt.seedFieldCentric()));

		driverController.y().onTrue(Commands.runOnce(() -> arm.incrementArmStow(1)));
		driverController.a().onTrue(Commands.runOnce(() -> arm.incrementArmStow(-1)));

		driverController.x().whileTrue(dt.goToPose(ReefSide.LEFT));
		driverController.b().whileTrue(dt.goToPose(ReefSide.RIGHT));

		// brake when we aren't driving
		new Trigger(() -> Math.abs(driverController.getLeftX()) < 0.1)
				.and(() -> Math.abs(driverController.getLeftY()) < 0.1)
				.and(() -> Math.abs(driverController.getRightX()) < 0.1)
				.whileTrue(dt.applyRequest(() -> brake))
				.onFalse(/*
							 * Commands.either(drivePointingAtAngle, driveFieldCentric, () -> driveAngled)
							 */ driveFieldCentric);

		// swap to driving at angle
		driverController.y().onTrue(drivePointingAtAngle);
		// if we try to rotate the bot, go back to normal driving
		new Trigger(() -> Math.abs(driverController.getRightTriggerAxis()) > 0.05).onTrue(driveFieldCentric);

	}

	public Command getAutonomousCommand() {
		if (Robot.isSimulation()) {
			return preloadAuto().cmd();
		}
		return autoChooser.selectedCommand();

	}

	// log if robot is enabled
	public boolean isRobotEnabled() {
		return DriverStation.isEnabled();
	}

	private void setRumble(double intensity) {
		if (!DriverStation.isAutonomous()) {
			driverController.getHID().setRumble(RumbleType.kBothRumble, intensity);
		}
	}

	public Command buildLeftAuto() {

		return Commands.sequence(
				autoFactory.resetOdometry("right start drive 1"),
				autoFactory.trajectoryCmd("right start drive 1"));

		// goToL4,
		// AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("score preload
		// i")),
		// dunk,
		// new WaitCommand(0.5),
		// eject,
		// Commands.parallel(
		// AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("preload to
		// HP")),
		// Commands.sequence(new WaitCommand(0.5), intake)),
		// new WaitCommand(1),

		// AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("hp to score
		// 1")),
		// goToL4,
		// AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("score K")),
		// new WaitCommand(0.5),
		// dunk,
		// new WaitCommand(0.5),
		// eject);

	}

	public AutoRoutine buildAuto() {
		AutoRoutine autoRoutine = autoFactory.newRoutine("autoroutine");
		AutoTrajectory path = autoRoutine.trajectory("mirrored_right start drive 1");
		AutoTrajectory path2 = autoRoutine.trajectory("mirrored_score preload i");
		AutoTrajectory path3 = autoRoutine.trajectory("mirrored_preload to HP");
		AutoTrajectory path4 = autoRoutine.trajectory("mirrored_hp to score 1");
		AutoTrajectory path5 = autoRoutine.trajectory("mirrored_score K");
		AutoTrajectory PRE_F = autoRoutine.trajectory("path", 0);
		AutoTrajectory SCORE_F = autoRoutine.trajectory("path", 1);
		AutoTrajectory F_SOURCE = autoRoutine.trajectory("path", 2);
		AutoTrajectory SCORE_D = autoRoutine.trajectory("path", 3);
		AutoTrajectory D_SOURCE = autoRoutine.trajectory("path", 4);
		AutoTrajectory SCORE_C = autoRoutine.trajectory("path", 5);
		// autoRoutine.active().onTrue(
		// path.resetOdometry().andThen(path.cmd(), /* dt.applyRequest(() -> brake), */
		// path2.cmd(), path3.cmd(),
		// path4.cmd(), path5.cmd()));
		autoRoutine.active().onTrue(
				PRE_F.resetOdometry().andThen(
						PRE_F.cmd(),

						SCORE_F.cmd()
								.alongWith(superstructure.forceState(SuperState.PRE_L4)
										.andThen(new WaitUntilCommand(subsystemsAtSetpoints))),
						dunk,
						eject));
		// F_SOURCE.cmd()
		// .alongWith(Commands.sequence(new WaitCommand(0.5),
		// superstructure.forceState(SuperState.INTAKE_HP))),
		// // */,
		// new WaitCommand(0.5),
		// SCORE_D.cmd()/* */,
		// dt.applyRequest(() -> brake).withTimeout(0.1),
		// (Commands.sequence(new WaitUntilCommand(
		// () -> superstructure.getState() == SuperState.READY_SCORE_CORAL),
		// superstructure.forceState(SuperState.PRE_L4)))));
		// D_SOURCE.cmd(),
		// new WaitCommand(0.5),
		// SCORE_C.cmd()));

		return autoRoutine;
	}

	public AutoRoutine preloadAuto() {
		AutoRoutine autoRoutine = autoFactory.newRoutine("auto");
		AutoTrajectory path = autoRoutine.trajectory("mirrored_preload", 0);
		AutoTrajectory path2 = autoRoutine.trajectory("mirrored_preload", 1);
		AutoTrajectory path3 = autoRoutine.trajectory("mirrored_preload", 2);

		autoRoutine.active().onTrue(
				path.resetOdometry().andThen(
						path.cmd().alongWith(superstructure.forceState(SuperState.PRE_L4)
								.andThen(new WaitUntilCommand(subsystemsAtSetpoints))),
						dunk,
						new WaitCommand(0.5),
						eject,
						path2.cmd(),
						superstructure.forceState(SuperState.STOW),
						new WaitUntilCommand(subsystemsAtSetpoints),
						path3.cmd()));

		return autoRoutine;
	}

}
