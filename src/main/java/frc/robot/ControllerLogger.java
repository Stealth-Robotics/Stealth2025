package frc.robot;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

@CustomLoggerFor(CommandPS5Controller.class)
public class ControllerLogger extends ClassSpecificLogger<CommandPS5Controller> {

    public ControllerLogger() {
        super(CommandPS5Controller.class);
    }

    @Override
    public void update(EpilogueBackend backend, CommandPS5Controller controller) {
        backend.log("Cross", controller.cross().getAsBoolean());
        backend.log("Square", controller.square().getAsBoolean());
        backend.log("Circle", controller.circle().getAsBoolean());
        backend.log("Triangle", controller.triangle().getAsBoolean());
        backend.log("Left Y", controller.getLeftY());

    }
}
