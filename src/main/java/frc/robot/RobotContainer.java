// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.lang.management.CompilationMXBean;

import org.opencv.features2d.FlannBasedMatcher;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
import frc.robot.subsystems.Superstructure.SuperState;

@Logged
public class RobotContainer {
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

	CommandXboxController driverController = new CommandXboxController(0);
	CommandXboxController operatorController = new CommandXboxController(1);
	Superstructure superstructure;
	Rollers rollers;
	Elevator elevator;
	Arm arm;
	Transfer transfer;
	Leds leds;
	Climber climber;
	CommandSwerveDrivetrain dt = TunerConstants.createDrivetrain();
	SwerveLogger logger = new SwerveLogger(dt);

	LevelTarget target = LevelTarget.L4;
	AlgaeTarget algaeTarget = AlgaeTarget.NET;

	// private final SendableChooser<Command> autoChooser;

	Command goToL4;
	Command dunk;
	Command intake;
	Command eject;

	Command driveFieldCentric;
	Command drivePointingAtAngle;

	boolean driveAngled = false;

	public Command leftAuto;

	public RobotContainer() {
		// configure PID for heading controller
		driveAngle.HeadingController.setP(0.1);

		elevator = new Elevator();
		rollers = new Rollers(() -> superstructure.getState());
		arm = new Arm();
		transfer = new Transfer();
		leds = new Leds();
		climber = new Climber();

		Trigger subsystemsAtSetpoints = new Trigger(() -> elevator.isElevatorAtTarget())
				.and(() -> arm.isMotorAtTarget()).debounce(0.1);

		// autoChooser = AutoBuilder.buildAutoChooser();
		// SmartDashboard.putData("Auto Chooser", autoChooser);

		superstructure = new Superstructure(
				elevator,
				rollers,
				arm,
				transfer,
				leds,
				// TODO: DECIDE WHETHER WE USE TOUCHSCREEN OR CONTROLLER
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
				new Trigger(() -> false),
				driverController.povLeft(),
				driverController.povRight(),
				(rumble) -> setRumble(rumble));

		climber.setDefaultCommand(climber.setDutyCycle(
				() -> operatorController.getRightTriggerAxis() - operatorController.getLeftTriggerAxis()));

		goToL4 = Commands.sequence(superstructure.forceState(SuperState.PRE_L4),
				new WaitUntilCommand(subsystemsAtSetpoints));
		dunk = Commands.sequence(superstructure.forceState(SuperState.SCORE_CORAL),
				new WaitUntilCommand(subsystemsAtSetpoints));
		eject = Commands.sequence(superstructure.forceState(SuperState.SPIT), new WaitCommand(0.5));
		intake = Commands.sequence(superstructure.forceState(SuperState.INTAKE_HP));

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

		driverController.povDown().onTrue(Commands.runOnce(() -> dt.seedFieldCentric()));

		// brake when we aren't driving
		new Trigger(() -> Math.abs(driverController.getLeftX()) < 0.1)
				.and(() -> Math.abs(driverController.getLeftY()) < 0.1)
				.and(() -> Math.abs(driverController.getRightX()) < 0.1)
				.whileTrue(dt.applyRequest(() -> brake))
				.onFalse(Commands.either(drivePointingAtAngle, driveFieldCentric, () -> driveAngled));

		// swap to driving at angle
		driverController.y().onTrue(drivePointingAtAngle);
		// if we try to rotate the bot, go back to normal driving
		new Trigger(() -> Math.abs(driverController.getRightTriggerAxis()) > 0.05).onTrue(driveFieldCentric);

	}

	public Command getAutonomousCommand() {

		return leftAuto;
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

	public void buildAutos() {
		try {
			leftAuto = Commands.sequence(

					AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("right start drive 1")),
					goToL4,
					AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("score preload i")),
					dunk,
					new WaitCommand(0.5),
					eject,
					Commands.parallel(
							AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("preload to HP")),
							Commands.sequence(new WaitCommand(0.5), intake)),
					new WaitCommand(1),

					AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("hp to score 1")),
					goToL4,
					AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("score K")),
					new WaitCommand(0.5),
					dunk,
					new WaitCommand(0.5),
					eject);
		} catch (Exception e) {
		}
	}

}
