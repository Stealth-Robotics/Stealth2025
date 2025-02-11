package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Arm extends SubsystemBase {

    private final TalonFX armMotor;
    private final CANcoder canCoder;

    // TODO: Find CANcoder CAN ID
    @NotLogged
    private final int CANCODER_CAN_ID = 0;

    @NotLogged
    private final double kP = 0,
            kI = 0,
            kD = 0,
            kTolerance = 0.5, // Tolerance in degrees
            MOTION_MAGIC_ACCELERATION = 0,
            MOTION_MAGIC_CRUISE_VELOCITY = 0,
            DEGREES_TO_TICKS = 1 / 360.0,
            ZERO_OFFSET = 0;

    private final TalonFXConfiguration armMotorConfiguration;
    private final CANcoderConfiguration canCoderConfiguration;

    @SuppressWarnings("unused")
    private double armTargetPosition = 0; // Target position as a variable for logging purposes

    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    @NotLogged
    public static final double INTAKE_HP_DEGREES = 180, // todo tune
            PRE_L1_DEGREES = 165, // todo tune
            PRE_L2_DEGREES = 15, // todo tune
            PRE_L3_DEGREES = 15, // todo tune
            PRE_L4_DEGREES = 70, // todo tune
            SCORE_L1_DEGREES = 165, // todo tune
            SCORE_L2_DEGREES = 20, // todo tune
            SCORE_L3_DEGREES = 20, // todo tune
            SCORE_L4_DEGREES = 90, // todo tune
            REMOVE_ALGAE_HIGH_DEGREES = 0.0, // todo tune
            REMOVE_ALGAE_LOW_DEGREES = 0.0, // todo tune
            PRE_PROCESSOR_DEGREES = 0.0, // todo tune
            PRE_NET_DEGREES = 0.0, // todo tune
            STOWED_DEGREES = 5; // todo tune

    public Arm() {
        // TODO: Find CAN IDs
        armMotor = new TalonFX(99);
        canCoder = new CANcoder(CANCODER_CAN_ID);
        armMotorConfiguration = new TalonFXConfiguration();
        canCoderConfiguration = new CANcoderConfiguration();
        applyConfigs();
    }

    private void applyConfigs() {
        armMotorConfiguration.Slot0.kP = kP;
        armMotorConfiguration.Slot0.kI = kI;
        armMotorConfiguration.Slot0.kD = kD;

        armMotorConfiguration.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        armMotorConfiguration.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;

        canCoderConfiguration.MagnetSensor.MagnetOffset = ZERO_OFFSET;

        armMotorConfiguration.Feedback.FeedbackRemoteSensorID = CANCODER_CAN_ID;
        armMotorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        armMotorConfiguration.Feedback.SensorToMechanismRatio = 1;
        armMotorConfiguration.Feedback.FeedbackRotorOffset = ZERO_OFFSET;

        armMotor.getConfigurator().apply(armMotorConfiguration);
        canCoder.getConfigurator().apply(canCoderConfiguration);
    }

    // made public so this is logged
    public double getArmPosition() {
        return canCoder.getPosition().getValueAsDouble();
    }

    private double getTargetPosition() {
        return motionMagicVoltage.Position;
    }

    public boolean isMotorAtTarget() {
        return Math.abs(getArmPosition() - getTargetPosition()) <= kTolerance;
    }

    private void setTargetPosition(double degrees) {
        motionMagicVoltage.Position = degrees;
        armTargetPosition = degrees;
        armMotor.setControl(motionMagicVoltage);
    }

    public Command rotateToPositionCommand(DoubleSupplier degrees) {
        return this.runOnce(() -> setTargetPosition(degrees.getAsDouble() * DEGREES_TO_TICKS));
        // No WaitUntil because its handled in the SuperStructure
    }

}
