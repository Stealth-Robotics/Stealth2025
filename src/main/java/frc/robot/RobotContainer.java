// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.ModuleLayer.Controller;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
	// Dashboard dashboard;

	LevelTarget target = LevelTarget.L1;
	AlgaeTarget algaeTarget = AlgaeTarget.PROCESSOR;

	public RobotContainer() {

		elevator = new Elevator();
		rollers = new Rollers(() -> superstructure.getState());
		dashboard = new Dashboard();
		superstructure = new Superstructure(
				elevator,
				rollers,
				// TODO: DECIDE WHETHER WE USE TOUCHSCREEN OR CONTROLLER
				() -> dashboard.getTargetScoringLevel(),
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
		configureBindings();

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

	private void configureBindings() {
		operatorController.cross().onTrue(Commands.runOnce(() -> target = LevelTarget.L1));
		operatorController.circle().onTrue(Commands.runOnce(() -> target = LevelTarget.L2));
		operatorController.square().onTrue(Commands.runOnce(() -> target = LevelTarget.L3));
		operatorController.triangle().onTrue(Commands.runOnce(() -> target = LevelTarget.L4));
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
