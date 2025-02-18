package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;

import java.util.function.DoubleSupplier;

@Logged
public class Elevator extends SubsystemBase {

    private final TalonFX motor1;
    private final TalonFX motor2;

    @NotLogged
    private final double kP = 10,
            kI = 0.0,
            kD = 0.0,

            kS = 0.0,
            kV = 0.0,
            kG = 0.0,
            // TODO tune
            MOTION_MAGIC_ACCELERATION = 150,
            MOTION_MAGIC_CRUISE_VELOCITY = 75;

    private final double TOLERANCE = 0.05; // todo tune

    @NotLogged
    public static final double INTAKE_HP_INCHES = 24, // todo tune
            PRE_L1_INCHES = 20, // todo tune
            PRE_L2_INCHES = 25, // todo tune
            PRE_L3_INCHES = 30, // todo tune
            PRE_L4_INCHES = 60, // todo tune
            SCORE_L1_INCHES = 15, // todo tune
            SCORE_L2_INCHES = 20.0, // todo tune
            SCORE_L3_INCHES = 25.0, // todo tune
            SCORE_L4_INCHES = 50.0, // todo tune
            REMOVE_ALGAE_HIGH_INCHES = 0.0, // todo tune
            REMOVE_ALGAE_LOW_INCHES = 0.0, // todo tune
            PRE_PROCESSOR_INCHES = 0.0, // todo tune
            PRE_NET_INCHES = 0.0, // todo tune
            GRAB_CORAL_INCHES = 18.5, // todo tune
            STOWED_INCHES = 24.0; // todo tune

    @NotLogged
    public static final double INTAKE_HP_ROTATIONS = 17.4, // todo tune
            PRE_L1_ROTATIONS = 20, // todo tune
            PRE_L2_ROTATIONS = 7.25, // todo tune
            PRE_L3_ROTATIONS = 24.0, // todo tune
            PRE_L4_ROTATIONS = 43.5, // todo tune
            SCORE_L1_ROTATIONS = 15, // todo tune
            SCORE_L2_ROTATIONS = 4.5, // todo tune
            SCORE_L3_ROTATIONS = 22.0, // todo tune
            SCORE_L4_ROTATIONS = 43.5, // todo tune
            REMOVE_ALGAE_HIGH_ROTATIONS = 22.0, // todo tune
            REMOVE_ALGAE_LOW_ROTATIONS = 35, // todo tune
            PRE_PROCESSOR_ROTATIONS = 40.0, // todo tune
            PRE_NET_ROTATIONS = 40.0, // todo tune
            GRAB_CORAL_ROTATIONS = 10, // todo tune
            STOWED_ROTATIONS = 12; // todo tune

    @NotLogged
    private final double MAX_EXTENSION_IN_INCHES = 60.0,
            MAX_EXTENSION_IN_ROTATIONS = 43.0,
            STARTING_POSITION_INCHES = 15.0,
            ROTATIONS_PER_INCH = MAX_EXTENSION_IN_ROTATIONS / MAX_EXTENSION_IN_INCHES;

    boolean atPosition;
    private boolean isHomed;

    Trigger currentOverZeroThreshold;

    // keep track of target position for logging purposes, this value gets updated
    // in setposition command
    @Logged(name = "Elevator Target")
    private double elevatorTargetPosition;

    private final TalonFXConfiguration config = new TalonFXConfiguration();
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(
            0);

    public Elevator() {
        motor1 = new TalonFX(2);
        motor2 = new TalonFX(3);

        elevatorTargetPosition = 0;
        motor1.setPosition(0);
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

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // todo change

        config.MotionMagic.MotionMagicAcceleration = MOTION_MAGIC_ACCELERATION;
        config.MotionMagic.MotionMagicCruiseVelocity = MOTION_MAGIC_CRUISE_VELOCITY;

        // turn down status signals
        motor1.getDeviceTemp().setUpdateFrequency(1);
        motor1.getSupplyCurrent().setUpdateFrequency(50);

        // IMPORTANT: MUST BE ENABLED FOR FOLLOWER
        motor1.getMotorVoltage().setUpdateFrequency(50);
        motor1.getDutyCycle().setUpdateFrequency(50);
        motor1.getTorqueCurrent().setUpdateFrequency(50);

        motor2.getDeviceTemp().setUpdateFrequency(1);
        motor2.getSupplyCurrent().setUpdateFrequency(1);
        motor2.getMotorVoltage().setUpdateFrequency(1);
        motor1.getPosition().setUpdateFrequency(200);
        // turn off everythign we dont need
        ParentDevice.optimizeBusUtilizationForAll(motor1, motor2);

        motor1.getConfigurator().apply(config);
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motor2.getConfigurator().apply(config);

        motor2.setControl(new Follower(motor1.getDeviceID(), true));

        currentOverZeroThreshold = new Trigger(() -> (motor1.getSupplyCurrent().getValueAsDouble() > 10)).debounce(0.5);
    }

    public Command goToPositionInInches(DoubleSupplier inches) {

        return goToPosition(() -> (inches.getAsDouble() * ROTATIONS_PER_INCH))
                .alongWith(Commands.runOnce(() -> elevatorTargetPosition = inches.getAsDouble()));
    }

    public Command goToPosition(DoubleSupplier rot) {
        // removed wait until for use with superstructure logic stuff
        return this.runOnce(() -> setElevatorTargetPosition(rot.getAsDouble())).alongWith(
                Commands.runOnce(() -> elevatorTargetPosition = rot.getAsDouble()));
    }

    // public so it's auto-logged
    public double getElevatorMotorPosition() {
        return motor1.getPosition().getValueAsDouble();
    }

    public double getElevatorMotorPositionInches() {
        return motor1.getPosition().getValueAsDouble() / (12 / 4.25 * Math.PI);
    }

    public boolean isElevatorAtTarget() {
        if (Robot.isSimulation()) {
            return true;
        }
        return MathUtil.isNear(motionMagicVoltage.Position, motor1.getPosition().getValueAsDouble(), TOLERANCE);
    }

    public Command stopElevator() {
        return this.runOnce(() -> motor1.setControl(new NeutralOut()));
    }

    private void setElevatorTargetPosition(double pos) {
        motionMagicVoltage.Position = pos;
        motor1.setControl(motionMagicVoltage);
    }

    public Command homeElevator() {
        return this.run(() -> {
            motor1.setVoltage(-1);
            isHomed = false;
        }).until(currentOverZeroThreshold)
                .andThen(
                        this.runOnce(() -> {
                            motor1.setControl(new NeutralOut());
                            motor1.setPosition(0);
                            isHomed = true;
                        }));
    }

    public boolean getIsHomed() {
        return isHomed;
    }

}
