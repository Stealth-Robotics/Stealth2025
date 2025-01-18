package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static java.lang.Math.abs;

import java.util.function.DoubleSupplier;

@Logged
public class Elevator extends SubsystemBase {
    
    private final TalonFX motor1;
    private final TalonFX motor2;
    
    private final double kP = 0.0;
    private final double kI = 0.0;
    private final double kD = 0.0;    

    private final double kS = 0.0;
    private final double kV = 0.0;
    private final double kG = 0.0;

    private final double MOTION_MAGIC_JERK = 7;
    private final double MOTION_MAGIC_ACCELERATION = 2;
    private final double MOTION_MAGIC_CRUISE_VELOCITY = 0.5;

    private final double TOLERANCE = 0.0; //todo tune

    private final double MAX_EXTENSION_IN_INCHES = 0.0;
    private final double MAX_EXTENSION_IN_ROTATIONS = 0.0;
    private final double ROTATIONS_PER_INCH = MAX_EXTENSION_IN_ROTATIONS / MAX_EXTENSION_IN_INCHES;

    private final TalonFXConfiguration config = new TalonFXConfiguration();
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    public Elevator() {
        motor1 = new TalonFX(0);
        motor2 = new TalonFX(0);
        applyConfigs();
    }

    private void applyConfigs() {

        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;

        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kG = kG;

        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //todo change

        config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicJerk = MOTION_MAGIC_JERK;

        motor1.getConfigurator().apply(config);
        motor2.getConfigurator().apply(config);

        motor2.setControl(new Follower(motor1.getDeviceID(), false));
    }

    public Command goToPositionInInches(DoubleSupplier inches) {
        return goToPosition(() -> inches.getAsDouble() * ROTATIONS_PER_INCH);
    }

    private Command goToPosition(DoubleSupplier rot) {
        return this.runOnce(() -> setElevatorTargetPosition(rot.getAsDouble()))
            .andThen(new WaitUntilCommand(() -> isElevatorAtTarget()));
    }

    private boolean isElevatorAtTarget() {
        return abs(motor1.getPosition().getValueAsDouble() - motionMagicVoltage.Position) <= TOLERANCE;
    }

    private void setElevatorTargetPosition(double pos) {
        motionMagicVoltage.Position = pos;
        motor1.setControl(motionMagicVoltage);
    }

    @Override
    public void periodic() {

    }
}
