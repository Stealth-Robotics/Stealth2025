package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class SwerveLogger extends SubsystemBase {
    @NotLogged
    SwerveDrivetrain<TalonFX, TalonFX, CANcoder> dt;

    double[] modulePositions = new double[4];

    TalonFX frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    TalonFX frontLeftSteer, frontRightSteer, backLeftSteer, backRightSteer;

    private Pose2d odoPose = new Pose2d();

    public SwerveLogger(SwerveDrivetrain<TalonFX, TalonFX, CANcoder> dt) {
        this.dt = dt;

        frontLeftDrive = dt.getModule(0).getDriveMotor();
        frontRightDrive = dt.getModule(1).getDriveMotor();
        backLeftDrive = dt.getModule(2).getDriveMotor();
        backRightDrive = dt.getModule(3).getDriveMotor();

        frontLeftSteer = dt.getModule(0).getSteerMotor();
        frontRightSteer = dt.getModule(1).getSteerMotor();
        backLeftSteer = dt.getModule(2).getSteerMotor();
        backRightSteer = dt.getModule(3).getSteerMotor();

    }

    private void updateModulePositions() {
        for (int i = 0; i < 4; i++) {
            modulePositions[i] = dt.getModule(i).getSteerMotor().getPosition().getValueAsDouble() * 360.0;
        }
    }

    private void updateOdometryPosition() {
        odoPose = dt.getState().Pose;
    }

    @Override
    public void periodic() {
        updateModulePositions();
        updateOdometryPosition();
    }

}