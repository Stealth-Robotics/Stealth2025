package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

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
            kG = 0.3,
            kTolerance = 2, // Tolerance in degrees
            MOTION_MAGIC_ACCELERATION = 8,
            MOTION_MAGIC_CRUISE_VELOCITY = 4,
            DEGREES_TO_ROTATIONS = 1 / 360.0,
            ZERO_OFFSET = 0.0249;

    private final TalonFXConfiguration armMotorConfiguration;
    private final CANcoderConfiguration canCoderConfiguration;

    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    @NotLogged
    public static final double PRE_L1_DEGREES = 50,
            PRE_L2_DEGREES = 50,
            PRE_L3_DEGREES = 50,
            PRE_L4_DEGREES = 58,
            SCORE_L1_DEGREES = 50, 
            SCORE_L2_DEGREES = 30, 
            SCORE_L3_DEGREES = 45, 
            SCORE_L4_DEGREES = 45, 
            REMOVE_ALGAE_HIGH_DEGREES = 0.0, 
            REMOVE_ALGAE_LOW_DEGREES = 0.0, 
            PRE_PROCESSOR_DEGREES = -5.0, 
            PRE_NET_DEGREES = 80, 
            READY_SCORE_ALGAE = 70.0,
            STOWED_DEGREES = 80; 

    public static double INTAKE_HP_DEGREES = -92.5; 

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
        armMotorConfiguration.Slot0.kG = kG;

        armMotorConfiguration.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        armMotorConfiguration.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;
        armMotorConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        // ! Stop arm from falling down in auto
        armMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        canCoderConfiguration.MagnetSensor.MagnetOffset = ZERO_OFFSET;

        canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        // canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.75;

        // armMotorConfiguration.Feedback.FeedbackRemoteSensorID = CANCODER_CAN_ID;
        // armMotorConfiguration.Feedback.FeedbackSensorSource =
        // FeedbackSensorSourceValue.RotorSensor;
        armMotorConfiguration.Feedback.SensorToMechanismRatio = (80.0 / 10.0) * (50.0 / 16.0) * (36.0 / 18.0);

        armMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // // turn down status signals
        // armMotor.getDeviceTemp().setUpdateFrequency(1);
        // armMotor.getSupplyCurrent().setUpdateFrequency(1);
        // armMotor.getMotorVoltage().setUpdateFrequency(1);
        // armMotor.getPosition().setUpdateFrequency(200);
        // // turn off everythign we dont need
        // armMotor.optimizeBusUtilization();

        armMotor.getConfigurator().apply(armMotorConfiguration);
        canCoder.getConfigurator().apply(canCoderConfiguration);
        armMotor.setPosition(Units.degreesToRotations(-92.0));
    }

    public double getArmPosition() {
        return armMotor.getPosition().getValueAsDouble() * 360.0;
    }

    public double getTargetPosition() {
        return Units.rotationsToDegrees(motionMagicVoltage.Position);
    }

    public boolean isMotorAtTarget() {
        if (Robot.isSimulation()) {
            return true;
        }
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

    public void incrementArmStow(double incr) {
        INTAKE_HP_DEGREES += incr;
    }

}
