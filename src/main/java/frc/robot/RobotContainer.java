// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.Transfer;

@Logged
public class RobotContainer {
	private final double MAX_VELO = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
	private final double MAX_ANGULAR_VELO = RotationsPerSecond.of(1.5).in(RadiansPerSecond);

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
	// Rollers rollers;
	Elevator elevator;
	Arm arm;
	Transfer transfer;
	CommandSwerveDrivetrain dt;

	LevelTarget target = LevelTarget.L4;
	AlgaeTarget algaeTarget = AlgaeTarget.PROCESSOR;

	// private final SendableChooser<Command> autoChooser;

	Command goToL4;
	Command dunk;
	Command intake;
	Command eject;

	Command driveFacingSetAngle;
	Command driveFieldCentric;

	private boolean driveFacingAngle = false;

	public Command leftAuto;

	public RobotContainer() {
		// configure PID for heading controller
		driveAngle.HeadingController.setP(0.1);

		elevator = new Elevator();
		// rollers = new Rollers(() -> superstructure.getState());
		arm = new Arm();
		transfer = new Transfer();
		dt = TunerConstants.createDrivetrain();

		Trigger subsystemsAtSetpoints = new Trigger(() -> elevator.isElevatorAtTarget())
				.and(() -> arm.isMotorAtTarget()).debounce(0.1);

		// autoChooser = AutoBuilder.buildAutoChooser();
		// SmartDashboard.putData("Auto Chooser", autoChooser);

		// superstructure = new Superstructure(
		// elevator,
		// rollers,
		// arm,
		// // TODO: DECIDE WHETHER WE USE TOUCHSCREEN OR CONTROLLER
		// () -> target,
		// () -> algaeTarget,
		// driverController.leftBumper(),
		// driverController.leftBumper(),
		// driverController.rightBumper(),
		// driverController.leftBumper(),
		// driverController.rightTrigger(),
		// driverController.rightBumper(),
		// // TODO: BIND TO BUTTONS
		// operatorController.leftBumper(),
		// operatorController.rightBumper(),
		// operatorController.leftTrigger());

		// goToL4 = Commands.sequence(superstructure.forceState(SuperState.PRE_L4),
		// new WaitUntilCommand(subsystemsAtSetpoints));
		// dunk = Commands.sequence(superstructure.forceState(SuperState.SCORE_CORAL),
		// new WaitUntilCommand(subsystemsAtSetpoints));
		// eject = Commands.sequence(superstructure.forceState(SuperState.SPIT), new
		// WaitCommand(0.5));
		// intake = Commands.sequence(superstructure.forceState(SuperState.INTAKE_HP));

		// NamedCommands.registerCommand("Go to scoring", goToL4);
		// NamedCommands.registerCommand("Dunk", dunk);
		// NamedCommands.registerCommand("Eject", eject);
		// NamedCommands.registerCommand("Intake", intake);

		// rollers.configureStateSupplierTrigger();

		driveFacingSetAngle = Commands.runOnce(() -> dt.setDefaultCommand(dt.applyRequest(
				() -> driveAngle
						.withVelocityX(-driverController.getLeftTriggerAxis()
								* MAX_VELO)
						.withVelocityX(-driverController.getLeftX() * MAX_VELO))),

				dt);

		driveFieldCentric = Commands.runOnce(() -> dt.setDefaultCommand(dt.applyRequest(
				() -> drive.withVelocityX(-driverController.getLeftTriggerAxis() * MAX_VELO)
						.withVelocityX(-driverController.getLeftX() * MAX_VELO)
						.withRotationalRate(-driverController.getRightX() * MAX_ANGULAR_VELO))),
				dt);

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

	public void configureBindings() {

		operatorController.a().onTrue(Commands.runOnce(() -> target = LevelTarget.L1));
		operatorController.b().onTrue(Commands.runOnce(() -> target = LevelTarget.L2));
		operatorController.x().onTrue(Commands.runOnce(() -> target = LevelTarget.L3));
		operatorController.y().onTrue(Commands.runOnce(() -> target = LevelTarget.L4));

		driverController.a().onTrue(elevator.goToPosition(() -> 40));
		driverController.x().onTrue(elevator.goToPosition(() -> 0));

		// toggle driving to face angle or not
		driverController.y().onTrue(Commands.runOnce(() -> driveFacingAngle = !driveFacingAngle));
		new Trigger(() -> (Math.abs(driverController.getRightX()) > 0.05))
				.onTrue(Commands.runOnce(() -> driveFacingAngle = false));

		new Trigger(() -> driveFacingAngle).onTrue(driveFacingSetAngle).onFalse(driveFieldCentric);
		driverController.povDown().onTrue(Commands.runOnce(() -> dt.seedFieldCentric()));

		// brake when we aren't driving
		new Trigger(() -> Math.abs(driverController.getLeftX()) < 0.1)
				.and(() -> Math.abs(driverController.getLeftY()) < 0.1)
				.and(() -> Math.abs(driverController.getRightX()) < 0.1)
				.whileTrue(dt.applyRequest(() -> brake));

		transfer.setDefaultCommand(transfer
				.setDutyCycle(() -> (driverController.getLeftTriggerAxis() - driverController.getRightTriggerAxis())));

	}

	public Command getAutonomousCommand() {

		return Commands.none();
	}

	// public void buildAutos() {
	// try {
	// leftAuto = Commands.sequence(
	// Commands.parallel(
	// AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("right start
	// score preload")),
	// goToL4),
	// new WaitCommand(0.5),
	// dunk,
	// new WaitCommand(0.5),
	// eject,
	// Commands.parallel(
	// AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("preload to
	// HP")),
	// Commands.sequence(new WaitCommand(0.5), intake)),
	// new WaitCommand(1),
	// Commands.parallel(
	// AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("hp to score
	// 1")),
	// goToL4),
	// new WaitCommand(0.5),
	// dunk,
	// new WaitCommand(0.5),
	// eject);
	// } catch (Exception e) {
	// }
	// }

}
