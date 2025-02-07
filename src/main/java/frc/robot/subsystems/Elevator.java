package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

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

    @NotLogged
    public static final double INTAKE_HP_INCHES = 15.0, // todo tune
            PRE_L1_INCHES = 10, // todo tune
            PRE_L2_INCHES = 20, // todo tune
            PRE_L3_INCHES = 30, // todo tune
            PRE_L4_INCHES = 40, // todo tune
            SCORE_L1_INCHES = 5, // todo tune
            SCORE_L2_INCHES = 15, // todo tune
            SCORE_L3_INCHES = 25, // todo tune
            SCORE_L4_INCHES = 40, // todo tune
            REMOVE_ALGAE_HIGH_INCHES = 0.0, // todo tune
            REMOVE_ALGAE_LOW_INCHES = 0.0, // todo tune
            PRE_PROCESSOR_INCHES = 0.0, // todo tune
            PRE_NET_INCHES = 0.0, // todo tune
            STOWED_INCHES = 10; // todo tune

    // TODO tune
    private final double MOTION_MAGIC_JERK = 7;
    private final double MOTION_MAGIC_ACCELERATION = 2;
    private final double MOTION_MAGIC_CRUISE_VELOCITY = 0.5;

    private final double TOLERANCE = 2.0; // todo tune

    @NotLogged
    private final double MAX_EXTENSION_IN_INCHES = 0.0,
            MAX_EXTENSION_IN_ROTATIONS = 0.0,
            ROTATIONS_PER_INCH = MAX_EXTENSION_IN_ROTATIONS / MAX_EXTENSION_IN_INCHES;

    boolean atPosition;

    // keep track of target position for logging purposes, this value gets updated
    // in setposition command
    private double elevatorTargetPositionInches;

    private final Mechanism2d mechanism = new Mechanism2d(3, Units.feetToMeters(6));
    private final MechanismRoot2d root = mechanism.getRoot("Elevator", 1, 0.3);
    private final MechanismLigament2d carriage = new MechanismLigament2d("Carriage", 0, 90);

    private final ElevatorSim sim = new ElevatorSim(
            DCMotor.getKrakenX60(2),
            9,
            Units.lbsToKilograms(15),
            Units.inchesToMeters(2.25),
            0.0,
            Units.inchesToMeters(60),
            true,
            0.0);
    private final ProfiledPIDController controller = new ProfiledPIDController(30, 0, 0,
            new Constraints(5, 10));

    private final TalonFXConfiguration config = new TalonFXConfiguration();
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    public Elevator() {
        motor1 = new TalonFX(0);
        motor2 = new TalonFX(0);
        elevatorTargetPositionInches = motor1.getPosition().getValueAsDouble();
        applyConfigs();
        SmartDashboard.putData("elevator", mechanism);
        root.append(carriage);
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
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // todo change

        config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;
        config.MotionMagic.MotionMagicJerk = MOTION_MAGIC_JERK;

        motor1.getConfigurator().apply(config);
        motor2.getConfigurator().apply(config);

        motor2.setControl(new Follower(motor1.getDeviceID(), false));
    }

    public Command goToPositionInInches(DoubleSupplier inches) {

        return this.run(() -> {
            elevatorTargetPositionInches = inches.getAsDouble();
            double voltage = controller.calculate(sim.getPositionMeters(), Units.inchesToMeters(inches.getAsDouble()));
            sim.setInputVoltage(voltage);
        });

        // return goToPosition(() -> (inches.getAsDouble() * ROTATIONS_PER_INCH))
        // .alongWith(Commands.runOnce(() -> elevatorTargetPositionInches =
        // inches.getAsDouble()));
    }

    private Command goToPosition(DoubleSupplier rot) {
        // removed wait until for use with superstructure logic stuff
        return this.runOnce(() -> setElevatorTargetPositionInches(rot.getAsDouble()));
    }

    public boolean isElevatorAtTarget() {
        if (Robot.isSimulation()) {
            return MathUtil.isNear(Units.metersToInches(sim.getPositionMeters()), elevatorTargetPositionInches,
                    TOLERANCE);
        }
        return MathUtil.isNear(motionMagicVoltage.Position, motor1.getPosition().getValueAsDouble(), TOLERANCE);
    }

    public Command stopElevator() {
        return this.runOnce(() -> motor1.setControl(new NeutralOut()));
    }

    private void setElevatorTargetPositionInches(double pos) {
        motionMagicVoltage.Position = pos;
        motor1.setControl(motionMagicVoltage);
    }

    public void togglePosition() {
        System.out.println("hello from elevator");
        atPosition = !atPosition;
    }

    public void setLength(double length) {
        carriage.setLength(length);
    }

    public MechanismLigament2d getElevatorCarriage() {
        return carriage;
    }

    @Override
    public void periodic() {
        sim.update(0.02);
        setLength(sim.getPositionMeters());
    }
}
