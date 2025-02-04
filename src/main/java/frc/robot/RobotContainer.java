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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.Superstructure;

@Logged
public class RobotContainer {
	CommandPS5Controller operatorController = new CommandPS5Controller(1);
	Superstructure superstructure;
	Rollers rollers;
	Elevator elevator;
	// Dashboard dashboard;

	LevelTarget target = LevelTarget.L1;

	public RobotContainer() {

		elevator = new Elevator(operatorController.povDown());
		rollers = new Rollers(operatorController.square(), () -> superstructure.getState());
		// dashboard = new Dashboard();
		superstructure = new Superstructure(
				elevator,
				rollers,
				// TODO: DECIDE WHETHER WE USE TOUCHSCREEN OR CONTROLLER
				() -> target,
				operatorController.L1(),
				operatorController.L1(),
				operatorController.R1(), operatorController.povUp(),
				operatorController.povLeft(),
				// TODO: BIND TO BUTTONS
				new Trigger(() -> false),
				new Trigger(() -> false),
				new Trigger(() -> false));

		rollers.configureStateSupplierTrigger();
		configureBindings();

	}

	public enum LevelTarget {
		L1,
		L2,
		L3,
		L4
	}

	private void configureBindings() {
		operatorController.cross().onTrue(Commands.runOnce(() -> target = LevelTarget.L1));
		operatorController.circle().onTrue(Commands.runOnce(() -> target = LevelTarget.L2));
		// operatorController.square().onTrue(Commands.runOnce(() -> target =
		// LevelTarget.L3));
		operatorController.triangle().onTrue(Commands.runOnce(() -> target = LevelTarget.L4));

		operatorController.R1().onTrue(Commands.runOnce(() -> superstructure.printState()));

		operatorController.povDown().onTrue(Commands.runOnce(() -> elevator.togglePosition()));
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}
}
