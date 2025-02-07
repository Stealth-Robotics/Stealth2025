// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.ModuleLayer.Controller;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import choreo.auto.AutoFactory;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.Superstructure;

@Logged
public class RobotContainer {
	CommandXboxController driverController = new CommandXboxController(0);
	CommandXboxController operatorController = new CommandXboxController(1);
	Superstructure superstructure;
	Rollers rollers;
	Elevator elevator;
	Arm arm;
	Dashboard dashboard;
	LevelTarget target = LevelTarget.L1;
	AlgaeTarget algaeTarget = AlgaeTarget.PROCESSOR;
	AutoTriggers autoTriggers = new AutoTriggers();

	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {
		NamedCommands.registerCommand("Go to scoring", autoTriggers.preScore());
		NamedCommands.registerCommand("Dunk", autoTriggers.score());
		NamedCommands.registerCommand("Intake", autoTriggers.intake());
		NamedCommands.registerCommand("Spit", autoTriggers.outtake());
		NamedCommands.registerCommand("Stow", autoTriggers.stow());

		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);

		elevator = new Elevator();
		rollers = new Rollers(() -> superstructure.getState());
		arm = new Arm();
		dashboard = new Dashboard();
		superstructure = new Superstructure(
				elevator,
				rollers,
				arm,
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
				operatorController.leftTrigger());

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

		superstructure.configureTriggers(driverController.leftBumper(),
				driverController.leftBumper(),
				driverController.rightBumper(),
				driverController.leftBumper(),
				driverController.rightTrigger(),
				driverController.rightBumper(),
				// TODO: BIND TO BUTTONS
				operatorController.leftBumper(),
				operatorController.rightBumper(),
				operatorController.leftTrigger());
	}

	public void configureAutoBindings() {
		superstructure.configureTriggers(autoTriggers.preScoreTrigger(),
				autoTriggers.scoreTrigger(),
				autoTriggers.intakeTrigger(),
				autoTriggers.outtakeTrigger(),
				new Trigger(() -> false),
				autoTriggers.stowTrigger(),
				new Trigger(() -> false),
				new Trigger(() -> false),
				new Trigger(() -> false));
	}

	public Command getAutonomousCommand() {

		return autoChooser.getSelected();
	}
}
