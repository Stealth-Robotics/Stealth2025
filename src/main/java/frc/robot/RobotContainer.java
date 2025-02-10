// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperState;

@Logged
public class RobotContainer {
	CommandXboxController driverController = new CommandXboxController(0);
	CommandXboxController operatorController = new CommandXboxController(1);
	Superstructure superstructure;
	Rollers rollers;
	Elevator elevator;
	Arm arm;
	CommandSwerveDrivetrain dt;
	// Dashboard dashboard;
	LevelTarget target = LevelTarget.L4;
	AlgaeTarget algaeTarget = AlgaeTarget.PROCESSOR;

	private final SendableChooser<Command> autoChooser;

	Command goToL4;
	Command dunk;
	Command intake;
	Command eject;

	public RobotContainer() {

		elevator = new Elevator();
		rollers = new Rollers(() -> superstructure.getState());
		arm = new Arm(elevator);
		dt = TunerConstants.createDrivetrain();
		// dashboard = new Dashboard();

		Trigger subsystemsAtSetpoints = new Trigger(() -> elevator.isElevatorAtTarget())
				.and(() -> arm.isMotorAtTarget()).debounce(0.1);
		NamedCommands.registerCommand("Go to scoring", goToL4
				.andThen(new WaitUntilCommand(subsystemsAtSetpoints)));
		NamedCommands.registerCommand("Dunk state",
				dunk.andThen(new WaitUntilCommand(subsystemsAtSetpoints)));
		NamedCommands.registerCommand("Intake state",
				intake.andThen(new WaitUntilCommand(subsystemsAtSetpoints)));
		NamedCommands.registerCommand("Spit state",
				eject.andThen(new WaitUntilCommand(subsystemsAtSetpoints)));

		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);

		superstructure = new Superstructure(
				elevator,
				rollers,
				arm,
				// TODO: DECIDE WHETHER WE USE TOUCHSCREEN OR CONTROLLER
				() -> target,
				() -> algaeTarget,
				operatorController.leftBumper(),
				operatorController.leftBumper(),
				operatorController.rightBumper(),
				operatorController.leftBumper(),
				operatorController.rightTrigger(),
				operatorController.rightBumper(),
				// TODO: BIND TO BUTTONS
				new Trigger(() -> false),
				new Trigger(() -> false),
				new Trigger(() -> false));
		goToL4 = Commands.sequence(superstructure.forceState(SuperState.PRE_L4),
				new WaitUntilCommand(subsystemsAtSetpoints));
		dunk = Commands.sequence(superstructure.forceState(SuperState.SCORE_CORAL),
				new WaitUntilCommand(subsystemsAtSetpoints));
		eject = Commands.sequence(superstructure.forceState(SuperState.SPIT), new WaitCommand(0.5));
		intake = Commands.sequence(superstructure.forceState(SuperState.INTAKE_HP));

		rollers.configureStateSupplierTrigger();

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

		driverController.a().onTrue(elevator.goToPosition(() -> 500));
		driverController.x().onTrue(elevator.goToPosition(() -> 0));

	}

	public Command getAutonomousCommand() {

		return Commands.sequence(
				superstructure.forceState(SuperState.READY_SCORE_CORAL),
				goToL4,
				new WaitCommand(0.5),
				dunk,
				new WaitCommand(0.5),
				eject,
				new WaitCommand(0.5),
				intake);
	}
}
