package frc.robot.subsystems;

import java.awt.Color;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Robot;

@Logged
public class Arm extends SubsystemBase {

    private final TalonFX armMotor;
    private final CANcoder canCoder;

    // TODO: Find CANcoder CAN ID
    private final int CANCODER_CAN_ID = 0;

    private final double ARM_GEAR_RATIO = 1;
    private final double DEGREES_TO_TICKS = 1 / 360.0;
    private final double ZERO_OFFSET = 0;

    private final double kP = 0;
    private final double kI = 0;
    private final double kD = 0;
    // Tolerance in degrees
    private final double kTolerance = 0.5;

    private final TalonFXConfiguration armMotorConfiguration;
    private final CANcoderConfiguration canCoderConfiguration;

    @SuppressWarnings("unused")
    private double armTargetPosition = 0; // Target position as a variable for logging purposes

    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    private final ProfiledPIDController armPIDController = new ProfiledPIDController(20, 0, 1,
            new Constraints(5, 10));

    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            60,
            1.5,
            Units.inchesToMeters(8),
            0,
            Units.degreesToRadians(180),
            false,
            Units.degreesToRadians(0));

    private final MechanismLigament2d arm = new MechanismLigament2d("Arm", Units.inchesToMeters(6), 90, 3.0,
            new Color8Bit(0, 255, 0));

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
        armMotor = new TalonFX(0);
        canCoder = new CANcoder(CANCODER_CAN_ID);
        armMotorConfiguration = new TalonFXConfiguration();
        canCoderConfiguration = new CANcoderConfiguration();
        applyConfigs();
    }

    public Arm(Elevator elevator) {
        this();
        elevator.getElevatorCarriage().append(arm);

    }

    private void applyConfigs() {
        armMotorConfiguration.Slot0.kP = kP;
        armMotorConfiguration.Slot0.kI = kI;
        armMotorConfiguration.Slot0.kD = kD;

        canCoderConfiguration.MagnetSensor.MagnetOffset = ZERO_OFFSET;

        armMotorConfiguration.Feedback.FeedbackRemoteSensorID = CANCODER_CAN_ID;
        armMotorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        armMotorConfiguration.Feedback.SensorToMechanismRatio = 1;
        armMotorConfiguration.Feedback.FeedbackRotorOffset = ZERO_OFFSET;

        armMotor.getConfigurator().apply(armMotorConfiguration);
        canCoder.getConfigurator().apply(canCoderConfiguration);
    }

    private double getTargetPosition() {
        return motionMagicVoltage.Position;
    }

    public double getMotorPosition() {
        if (Robot.isSimulation()) {
            return Units.radiansToDegrees(armSim.getAngleRads());
        }
        return armMotor.getPosition().getValueAsDouble();
    }

    public boolean isMotorAtTarget() {
        if (Robot.isSimulation()) {
            return MathUtil.isNear(armTargetPosition, Units.radiansToDegrees(armSim.getAngleRads()), kTolerance);
        }
        return Math.abs(getMotorPosition() - getTargetPosition()) <= kTolerance;
    }

    private void setTargetPosition(double degrees) {
        motionMagicVoltage.Position = degrees;
        armTargetPosition = degrees;
        armMotor.setControl(motionMagicVoltage);
    }

    public Command rotateToPositionCommand(DoubleSupplier degrees) {
        return this.run(() -> {
            armTargetPosition = degrees.getAsDouble();
            armSim.setInputVoltage(
                    armPIDController.calculate(armSim.getAngleRads(), Units.degreesToRadians(degrees.getAsDouble())));

        });
        // No WaitUntil because its handled in the SuperStructure
    }

    @Override
    public void periodic() {
        if (Robot.isSimulation()) {

            armSim.update(0.02);
            arm.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
        }
    }

}
