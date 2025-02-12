package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;

@Logged
public class Transfer extends SubsystemBase {

    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final VoltageOut voltageOut;

    public Transfer() {
        leftMotor = new TalonFX(25);
        rightMotor = new TalonFX(26);
        voltageOut = new VoltageOut(0);
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true));
    }

    public Command setDutyCycle(DoubleSupplier dutycycle) {
        return this.run(() -> leftMotor.set(dutycycle.getAsDouble()));
    }

    public Command setVoltage(DoubleSupplier voltage) {
        return this.runOnce(() -> {
            voltageOut.Output = voltage.getAsDouble();
            leftMotor.setControl(voltageOut);
        });
    }

}
