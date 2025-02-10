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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Rollers;
import frc.robot.subsystems.Superstructure;

@Logged
public class RobotContainer {
	CommandXboxController driverController = new CommandXboxController(0);
	CommandPS5Controller operatorController = new CommandPS5Controller(1);
	Superstructure superstructure;
	Rollers rollers;
	Elevator elevator;
	Arm arm;
	CommandSwerveDrivetrain dt;
	// Dashboard dashboard;
	LevelTarget target = LevelTarget.L4;
	AlgaeTarget algaeTarget = AlgaeTarget.PROCESSOR;
	AutoTriggers autoTriggers = new AutoTriggers();

	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {

		elevator = new Elevator();
		rollers = new Rollers(() -> superstructure.getState());
		arm = new Arm();
		dt = TunerConstants.createDrivetrain();
		// dashboard = new Dashboard();

		Trigger subsystemsAtSetpoints = new Trigger(() -> elevator.isElevatorAtTarget())
				.and(() -> arm.isMotorAtTarget()).debounce(0.1);
		NamedCommands.registerCommand("Go to scoring", autoTriggers.preScore()
				.andThen(new WaitUntilCommand(subsystemsAtSetpoints)));
		NamedCommands.registerCommand("Dunk",
				autoTriggers.score().andThen(new WaitUntilCommand(subsystemsAtSetpoints)));
		NamedCommands.registerCommand("Intake",
				autoTriggers.intake().andThen(new WaitUntilCommand(subsystemsAtSetpoints)));
		NamedCommands.registerCommand("Spit",
				autoTriggers.outtake().andThen(new WaitUntilCommand(subsystemsAtSetpoints)));
		NamedCommands.registerCommand("Stow", autoTriggers.stow());

		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);

		superstructure = new Superstructure(
				elevator,
				rollers,
				arm,
				// TODO: DECIDE WHETHER WE USE TOUCHSCREEN OR CONTROLLER
				() -> target,
				() -> algaeTarget,
				operatorController.L1(),
				operatorController.L1(),
				operatorController.R1(),
				operatorController.L1(),
				operatorController.R2(),
				operatorController.R1(),
				// TODO: BIND TO BUTTONS
				new Trigger(() -> false),
				new Trigger(() -> false),
				new Trigger(() -> false));

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

		return AutoBuilder.buildAuto("Test Auto");
	}
}
