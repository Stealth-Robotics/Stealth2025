package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX> {

    public TalonFXLogger() {
        super(TalonFX.class);
    }

    @Override
    public void update(EpilogueBackend backend, TalonFX talonFX) {
        // might want to add more stuff, but currently logging temp, current and voltage
        // position/velocity logging will only be opt-in for subsystems where we care
        // about that, like arm or elevator
        backend.log("Supply Current", talonFX.getSupplyCurrent().getValueAsDouble());
        backend.log("Temp", talonFX.getDeviceTemp().getValueAsDouble());
        backend.log("Voltage", talonFX.getMotorVoltage().getValueAsDouble());
        backend.log("Stator current", talonFX.getStatorCurrent().getValueAsDouble());

    }
}
