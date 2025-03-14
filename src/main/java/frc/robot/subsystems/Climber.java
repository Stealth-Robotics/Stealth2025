package frc.robot.subsystems;

import java.util.DoubleSummaryStatistics;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Climber extends SubsystemBase {
    private final TalonFX motor;
    private final TalonFXConfiguration config = new TalonFXConfiguration();

    public Climber() {
        motor = new TalonFX(61);
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor.getConfigurator().apply(config);
    }

    public Command setDutyCycle(DoubleSupplier dutyCycle) {
        return this.run(() -> motor.set(dutyCycle.getAsDouble()));
    }

}
