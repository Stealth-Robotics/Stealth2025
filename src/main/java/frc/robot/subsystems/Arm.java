package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Arm extends SubsystemBase {

    private final TalonFX armMotor;
    private final CANcoder canCoder;

    // TODO: Find CANcoder CAN ID
    @NotLogged
    private final int CANCODER_CAN_ID = 36;

    private final NeutralOut neutralOut;
    @NotLogged
    private final double kP = 100,
            kI = 0,
            kD = 0,
            kTolerance = 1, // Tolerance in degrees
            MOTION_MAGIC_ACCELERATION = 8,
            MOTION_MAGIC_CRUISE_VELOCITY = 4,
            DEGREES_TO_ROTATIONS = 1 / 360.0,
            ZERO_OFFSET = 0.12;

    private final TalonFXConfiguration armMotorConfiguration;
    private final CANcoderConfiguration canCoderConfiguration;

    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    @NotLogged
    public static final double INTAKE_HP_DEGREES = 0, // todo tune
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
        armMotor = new TalonFX(36);
        canCoder = new CANcoder(CANCODER_CAN_ID);
        armMotorConfiguration = new TalonFXConfiguration();
        canCoderConfiguration = new CANcoderConfiguration();
        neutralOut = new NeutralOut();
        applyConfigs();
    }

    private void applyConfigs() {
        armMotorConfiguration.Slot0.kP = kP;
        armMotorConfiguration.Slot0.kI = kI;
        armMotorConfiguration.Slot0.kD = kD;

        armMotorConfiguration.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        armMotorConfiguration.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;
        armMotorConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        canCoderConfiguration.MagnetSensor.MagnetOffset = ZERO_OFFSET;
        canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        armMotorConfiguration.Feedback.FeedbackRemoteSensorID = CANCODER_CAN_ID;
        armMotorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        armMotorConfiguration.Feedback.SensorToMechanismRatio = 1;
        armMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        armMotor.getConfigurator().apply(armMotorConfiguration);
        canCoder.getConfigurator().apply(canCoderConfiguration);
    }

    // made public so this is logged
    public double getArmPosition() {
        return Units.rotationsToDegrees(canCoder.getPosition().getValueAsDouble());
    }

    public double getTargetPosition() {
        return Units.rotationsToDegrees(motionMagicVoltage.Position);
    }

    public boolean isMotorAtTarget() {
        return Math.abs(getArmPosition() - getTargetPosition()) <= kTolerance;
    }

    private void setTargetPosition(double rotations) {
        motionMagicVoltage.Position = rotations;
        armMotor.setControl(motionMagicVoltage);
    }

    public Command rotateToPositionCommand(DoubleSupplier degrees) {
        return this.runOnce(() -> setTargetPosition(degrees.getAsDouble() * DEGREES_TO_ROTATIONS));
        // No WaitUntil because its handled in the SuperStructure
    }

    public Command neutral() {
        return this.runOnce(() -> armMotor.setControl(neutralOut));
    }

}
