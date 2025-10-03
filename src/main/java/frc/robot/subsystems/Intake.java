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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

@Logged
public class Intake extends SubsystemBase {

    private final TalonFX deployMotor;
    private final TalonFX intakeMotor;

    private final double kP = 500.0,
            kI = 0.0,
            kD = 0.0,
            kS = 0.0,
            kV = 0.0,
            kG = 0.0,
            MOTION_MAGIC_ACCELERATION = 8,
            MOTION_MAGIC_CRUISE_VELOCITY = 1.1,
            TOLERANCE = 2;

    private final TalonFXConfiguration deployMotorConfiguration;
    private final TalonFXConfiguration intakeMotorConfiguration;

    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    private final double STALL_CURRENT_THRESHOLD = 20.0;

    private Angle deployOffset = Degrees.of(0);
    private double targetPosition = 0.0;

    public static final Angle DEPLOYED_ANGLE = Degrees.of(0),
            STOWED_ANGLE = Degrees.of(134);

    public Intake() {
        deployMotor = new TalonFX(10);
        intakeMotor = new TalonFX(9);

        deployMotorConfiguration = new TalonFXConfiguration();
        intakeMotorConfiguration = new TalonFXConfiguration();

        deployMotorConfiguration.Slot0.kP = kP;
        deployMotorConfiguration.Slot0.kI = kI;
        deployMotorConfiguration.Slot0.kD = kD;

        deployMotorConfiguration.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        deployMotorConfiguration.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;

        deployMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        deployMotorConfiguration.Feedback.SensorToMechanismRatio = (64.0 / 10.0) * (60.0 / 20.0) * (50.0 / 16.0);

        intakeMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        intakeMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 30.0;
        intakeMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

        intakeMotor.getConfigurator().apply(intakeMotorConfiguration);
        deployMotor.getConfigurator().apply(deployMotorConfiguration);

        deployMotor.setPosition(STOWED_ANGLE);
    }

    public Command rotateToPositionCommand(Angle angle) {
        return this.runOnce(() -> {
            motionMagicVoltage.Position = angle.in(Rotations);
            targetPosition = angle.in(Degrees) - deployOffset.in(Degrees);
            deployMotor.setControl(motionMagicVoltage);
        });
    }

    public boolean atPosition() {
        return MathUtil.isNear(deployMotor.getPosition().getValueAsDouble(), targetPosition, TOLERANCE);
    }

    public double getPositionDegrees() {
        return Units.rotationsToDegrees(deployMotor.getPosition().getValueAsDouble());
    }

    public Command smartResetDeployMotor() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> deployMotor.set(-0.1)),
            new WaitCommand(0.5),
            new WaitUntilCommand(() -> deployMotor.getStatorCurrent().getValueAsDouble() > STALL_CURRENT_THRESHOLD),
            new InstantCommand(() -> deployMotor.set(0.0)),
            new InstantCommand(() -> deployOffset = deployMotor.getPosition().getValue()),
            rotateToPositionCommand(Intake.STOWED_ANGLE)
        );
    }

    public Command setIntakeVoltage(DoubleSupplier voltage) {
        return Commands.run(() -> intakeMotor.setVoltage(voltage.getAsDouble()));
    }

    public Command setIntakeVoltage(double voltage) {
        return Commands.runOnce(() -> intakeMotor.setVoltage(voltage));
    }
}
