package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Intake extends SubsystemBase {

    private final TalonFX deployMotor;
    private final TalonFX intakeMotor;

    private final double kP = 0.0,
            kI = 0.0,
            kD = 0.0,
            kS = 0.0,
            kV = 0.0,
            kG = 0.0,
            MOTION_MAGIC_ACCELERATION = (Degrees.of(90).in(Rotations) / 0.2),
            MOTION_MAGIC_CRUISE_VELOCITY = (Degrees.of(90).in(Rotations) / 0.4),
            TOLERANCE = Degrees.of(0.5).in(Rotations);

    private final TalonFXConfiguration deployMotorConfiguration;
    private final TalonFXConfiguration intakeMotorConfiguration;

    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    private double targetPosition = 0.0;

    public Intake() {
        deployMotor = new TalonFX(0);
        intakeMotor = new TalonFX(1);

        deployMotorConfiguration = new TalonFXConfiguration();
        intakeMotorConfiguration = new TalonFXConfiguration();

        deployMotorConfiguration.Slot0.kP = kP;
        deployMotorConfiguration.Slot0.kI = kI;
        deployMotorConfiguration.Slot0.kD = kD;

        deployMotorConfiguration.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        deployMotorConfiguration.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;

        deployMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        deployMotorConfiguration.Feedback.SensorToMechanismRatio = (64.0 / 10.0) * (60.0 / 20.0) * (50.0 / 14.0);

        intakeMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }

    public Command rotateToPositionCommand(Angle angle) {
        motionMagicVoltage.Position = angle.in(Rotations);
        targetPosition = angle.in(Degrees);
        return this.runOnce(() -> deployMotor.setControl(motionMagicVoltage));
    }

    public boolean atPosition(Angle angle) {
        return MathUtil.isNear(deployMotor.getPosition().getValueAsDouble(), targetPosition, TOLERANCE);
    }

    public Command setIntakeVoltage(DoubleSupplier voltage) {
        return this.run(() -> intakeMotor.setVoltage(voltage.getAsDouble()));
    }

}
