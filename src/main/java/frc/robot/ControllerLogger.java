package frc.robot;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

@CustomLoggerFor(CommandXboxController.class)
public class ControllerLogger extends ClassSpecificLogger<CommandXboxController> {

    public ControllerLogger() {
        super(CommandXboxController.class);
    }

    @Override
    public void update(EpilogueBackend backend, CommandXboxController controller) {
        backend.log("A", controller.a().getAsBoolean());
        backend.log("B", controller.b().getAsBoolean());
        backend.log("X", controller.x().getAsBoolean());
        backend.log("Y", controller.y().getAsBoolean());
        backend.log("Right bumper", controller.rightBumper().getAsBoolean());
        backend.log("Left bumper", controller.leftBumper().getAsBoolean());

        backend.log("Left X", controller.getLeftX());
        backend.log("Left Y", controller.getLeftY());
        backend.log("Right X", controller.getRightX());

        backend.log("DPAD Up", controller.povUp().getAsBoolean());
        backend.log("DPAD Down", controller.povDown().getAsBoolean());
        backend.log("DPAD Right", controller.povRight().getAsBoolean());
        backend.log("DPAD Left", controller.povLeft().getAsBoolean());

        backend.log("Left trigger", controller.getLeftTriggerAxis());
        backend.log("Right trigger", controller.getRightTriggerAxis());

    }
}
