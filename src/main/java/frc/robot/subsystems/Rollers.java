package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

@Logged

public class Rollers extends SubsystemBase {

    private final TalonFX motor = new TalonFX(0); // todo: change the can id
    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private boolean hasGamepiece = false;

    private final Debouncer gamepeiceDetectionCurrentDebouncer = new Debouncer(1, DebounceType.kRising);
    Trigger trigger;

    public Rollers(Trigger trigger) {
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // todo check
        config.CurrentLimits.SupplyCurrentLimit = 30;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        motor.getConfigurator().apply(config);
        this.trigger = trigger;
    }

    public Command setRollerVoltage(DoubleSupplier voltage) {
        // we no longer have a gamepiece if the rollers have been reversed
        if (voltage.getAsDouble() < 0) {
            hasGamepiece = false;
        }
        return this.runOnce(() -> motor.setControl(new VoltageOut(voltage.getAsDouble())));
    }

    public boolean getHasGamepiece() {
        if (gamepeiceDetectionCurrentDebouncer.calculate(trigger.getAsBoolean())) {
            hasGamepiece = true;
        }
        return hasGamepiece;
    }

}
