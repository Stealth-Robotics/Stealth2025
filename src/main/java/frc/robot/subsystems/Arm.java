package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Arm extends SubsystemBase{
    
    private final double ARM_GEAR_RATIO = 1;
    private final TalonFX armMotor;
    
    private final double ZeroOffset = 0;

    private final double kP = 0;
    private final double kI = 0;
    private final double kD = 0;
    // Tolerance in degrees
    private final double kTolerance = 0;

    private final TalonFXConfiguration armMotorConfiguration;

    private final double MOTION_MAGIC_JERK = 0;
    private final double MOTION_MAGIC_ACCELERATION = 0;
    private final double MOTION_MAGIC_CRUISE_VELOCITY = 0;

    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    public Arm(){
        //TODO: Find CAN IDs
        armMotor = new TalonFX(0);
        armMotorConfiguration = new TalonFXConfiguration();
        applyConfigs();
        resetEncoder();
    }
    private void applyConfigs(){
        armMotorConfiguration.Slot0.kP = kP;
        armMotorConfiguration.Slot0.kI = kI;
        armMotorConfiguration.Slot0.kD = kD;

        armMotorConfiguration.MotionMagic.MotionMagicJerk = MOTION_MAGIC_JERK;
        armMotorConfiguration.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        armMotorConfiguration.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;

        armMotorConfiguration.Feedback.SensorToMechanismRatio = ARM_GEAR_RATIO;

        armMotor.getConfigurator().apply(armMotorConfiguration);
    }
    private void resetEncoder(){
        armMotor.setPosition(ZeroOffset);
    }
    private double getMotorPosition(){
        return armMotor.getPosition().getValueAsDouble();
    }
    private double getTargetPosition(){
        return motionMagicVoltage.Position;
    }
    private boolean isMotorAtTarget(double tolerance){
        return Math.abs(getMotorPosition()-getTargetPosition()) <= tolerance;
    }
    private void setTargetPosition(double degrees){
        motionMagicVoltage.Position = degrees;
        armMotor.setControl(motionMagicVoltage);
    }
    public Command rotateToPositionCommand(DoubleSupplier degrees, double tolerance){
        return this.runOnce(()->setTargetPosition(degrees.getAsDouble()))
        .andThen(new WaitUntilCommand(()->this.isMotorAtTarget(tolerance)));
    }
    public Command rotateWhileDrivingCommand(DoubleSupplier degrees){
        return this.runOnce(()->setTargetPosition(degrees.getAsDouble()));
    }

    
}
