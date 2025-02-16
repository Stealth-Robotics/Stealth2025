package frc.robot.subsystems;

import java.util.DoubleSummaryStatistics;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.Superstructure.SuperState;

@Logged

public class Rollers extends SubsystemBase {

    private final TalonFX motor = new TalonFX(28); // todo: change the can id
    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private boolean hasGamepiece = false;

    // debouncer for determining whether we have a gamepiece, will only return true
    // when value is true for debounce time
    /*
     * todo tune debounce time, ideally will be as low as possible without false
     * positives
     */
    private final Debouncer gamepeiceDetectionCurrentDebouncer = new Debouncer(0.1, DebounceType.kRising);
    @NotLogged
    Trigger trigger;

    Trigger stateSupplierTrigger;

    private final Supplier<SuperState> stateSupplier;

    private final VoltageOut voltageOut;

    public Rollers(Trigger trigger, Supplier<SuperState> stateSupplier) {
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // todo check
        config.CurrentLimits.SupplyCurrentLimit = 30; // limit current so we dont draw too much when stalling rollers to
                                                      // keep gamepeice
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        motor.getConfigurator().apply(config);
        this.trigger = trigger;
        voltageOut = new VoltageOut(0);

        this.stateSupplier = stateSupplier;

    }

    public Rollers(Supplier<SuperState> stateSupplier) {
        this(null, stateSupplier);

    }

    public void configureStateSupplierTrigger() {
        stateSupplierTrigger = new Trigger(() -> stateSupplier.get() == SuperState.INTAKE_HP)
                .onTrue(Commands.runOnce(() -> this.hasGamepiece = false));
    }

    public Command setRollerVoltage(DoubleSupplier voltage) {
        // we no longer have a gamepiece if the rollers have been reversed
        return this.runOnce(() -> {
            voltageOut.Output = voltage.getAsDouble();
            if (voltage.getAsDouble() < 0) {
                hasGamepiece = false;
            }
            motor.setControl(voltageOut);
        });
    }

    public Command setRollerVoltage(double voltage) {
        // we no longer have a gamepiece if the rollers have been reversed
        return this.runOnce(() -> {
            voltageOut.Output = voltage;
            if (voltage < 0) {
                hasGamepiece = false;
            }
            motor.setControl(voltageOut);
        });
    }

    public boolean getHasGamepiece() {
        if (Robot.isSimulation()) {
            return true;
        }

        // if we draw enough current for long enough, we have a gamepiece.
        if (gamepeiceDetectionCurrentDebouncer.calculate(motor.getSupplyCurrent().getValueAsDouble() > 10)) {
            hasGamepiece = true;
        }
        return hasGamepiece;
    }

}
