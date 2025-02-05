package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

@Logged
public class Arm extends SubsystemBase{
    
    private final double ARM_GEAR_RATIO = 1;
    private final TalonFX armMotor;
    
    private final double ZERO_OFFSET = 0;
    private final double DEGREES_TO_TICKS = 360;

    private double armTargetPosition = 0; //Target position as a variable for logging purposes

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
        armMotor.setPosition(ZERO_OFFSET);
    }
    private double getMotorPosition(){
        return armMotor.getPosition().getValueAsDouble();
    }
    private double getTargetPosition(){
        return motionMagicVoltage.Position;
    }
    private boolean isMotorAtTarget(){
        return Math.abs(getMotorPosition()-getTargetPosition()) <= kTolerance;
    }
    private void setTargetPosition(double degrees){
        motionMagicVoltage.Position = degrees;
        armTargetPosition = degrees;
        armMotor.setControl(motionMagicVoltage);
    }
    public Command rotateToPositionCommand(DoubleSupplier degrees){
        return this.runOnce(()->setTargetPosition(degrees.getAsDouble()*DEGREES_TO_TICKS))
        .andThen(new WaitUntilCommand(()->this.isMotorAtTarget()));
    }

    
}
