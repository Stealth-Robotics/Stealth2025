package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.Robot;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.RobotContainer.LevelTarget;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
@Logged
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during field-centric path following */
    private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();
    private final PIDController m_pathXController = new PIDController(20, 0, 0);
    private final PIDController m_pathYController = new PIDController(20, 0, 0);
    private final PIDController m_pathThetaController = new PIDController(7, 0, 0);

    private final SwerveRequest.ApplyFieldSpeeds applyRobotSpeeds = new SwerveRequest.ApplyFieldSpeeds();

    //TODO TUNE PID CONSTANTS
    private final ProfiledPIDController xController = new ProfiledPIDController(15, 3, 0, new Constraints(3, 3));
    private final ProfiledPIDController yController = new ProfiledPIDController(15, 3, 0, new Constraints(3, 3));
    private final ProfiledPIDController thetaController = new ProfiledPIDController(0.35, 0.06, 0.01, new Constraints(200, 300));

    private final PIDController reefController = new PIDController(0.4, 0.06, 0);
    Field2d field = new Field2d();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final Pose2d TAG_6 = new Pose2d(Inches.of(530.49), Inches.of(130.17), new Rotation2d(Degrees.of(300)));
    private final Pose2d TAG_7 = new Pose2d(Inches.of(546.87), Inches.of(158.5), new Rotation2d(Degrees.of(0)));
    private final Pose2d TAG_8 = new Pose2d(Inches.of(530.49), Inches.of(186.83), new Rotation2d(Degrees.of(60)));
    private final Pose2d TAG_9 = new Pose2d(Inches.of(497.77), Inches.of(186.83), new Rotation2d(Degrees.of(120)));
    private final Pose2d TAG_10 = new Pose2d(Inches.of(481.39), Inches.of(158.50), new Rotation2d(Degrees.of(180)));
    private final Pose2d TAG_11 = new Pose2d(Inches.of(497.77), Inches.of(130.17), new Rotation2d(Degrees.of(240)));

    private final Pose2d TAG_17 = new Pose2d(Inches.of(160.39), Inches.of(130.17), new Rotation2d(Degrees.of(240)));
    private final Pose2d TAG_18 = new Pose2d(Inches.of(144), Inches.of(158.5), new Rotation2d(Degrees.of(180)));
    private final Pose2d TAG_19 = new Pose2d(Inches.of(160.39), Inches.of(186.83), new Rotation2d(Degrees.of(120)));
    private final Pose2d TAG_20 = new Pose2d(Inches.of(193.1), Inches.of(186.83), new Rotation2d(Degrees.of(60)));
    private final Pose2d TAG_21 = new Pose2d(Inches.of(209.49), Inches.of(158.5), new Rotation2d(Degrees.of(0)));
    private final Pose2d TAG_22 = new Pose2d(Inches.of(193.1), Inches.of(130.17), new Rotation2d(Degrees.of(300)));

    private final Pose2d RED_REEF = new Pose2d(Inches.of(514.13), Inches.of(158.5), new Rotation2d());
    private final Pose2d BLUE_REEF = new Pose2d(Inches.of(176.7), Inches.of(158.5), new Rotation2d());

    private Pose2d targetReef;

    private Pose2d closestReefPole = new Pose2d();

    double MAX_VELO = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    double MAX_ANGULAR_VELO = RotationsPerSecond.of(1.5).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MAX_VELO * 0.1).withRotationalDeadband(MAX_ANGULAR_VELO * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final List<Pose2d> tagPoses = Arrays.asList(
            TAG_6,
            TAG_7,
            TAG_8,
            TAG_9,
            TAG_10,
            TAG_11,
            TAG_17,
            TAG_18,
            TAG_19,
            TAG_20,
            TAG_21,
            TAG_22);

    private List<Pose2d> reefPoleLocations = new ArrayList<>();

    private final Transform2d L4_APRILTAG_LEFT_TRANSFORM = new Transform2d(Units.inchesToMeters(25.5),
            Units.inchesToMeters(-5.0), new Rotation2d());

    private final Transform2d L4_APRILTAG_RIGHT_TRANSFORM = new Transform2d(Units.inchesToMeters(25.5),
            Units.inchesToMeters(7.5), new Rotation2d());

    private final Transform2d LOWER_APRILTAG_LEFT_TRANSFORM = new Transform2d(Units.inchesToMeters(22),
            Units.inchesToMeters(-5.0), new Rotation2d());
    private final Transform2d LOWER_APRILTAG_RIGHT_TRANSFORM = new Transform2d(Units.inchesToMeters(22),
            Units.inchesToMeters(7.5), new Rotation2d());

    Transform2d APRILTAG_LEFT_TRANSFORM = L4_APRILTAG_LEFT_TRANSFORM;
    Transform2d APRILTAG_RIGHT_TRANSFORM = L4_APRILTAG_RIGHT_TRANSFORM;

    Pose3d limelightEst;

    double measurementX, measurementY, measurementTheta;

    Pose2d targetPose2d;

    private boolean atPose;

    private final Map<Pose2d, Integer> poseTagMap = new HashMap<>();

    boolean isAligning = false;

    private int nearestTagId;

    private SwerveSetpoint previousSetpoint;
    // private SwerveSetpointGenerator setpointGenerator = new
    // SwerveSetpointGenerator(null, null)

    public enum ReefSide {
        LEFT,
        RIGHT
    }

    public void configField() {
        SmartDashboard.putData(field);
    }

    Rotation2d zeroRotation = new Rotation2d();

    // @Logged
    SwerveSample[] trajSamples;

    // @Logged
    Pose3d testPose3d = new Pose3d(
            new Pose2d(Distance.ofBaseUnits(6, Meter), Distance.ofBaseUnits(3, Meter), new Rotation2d()));

    Pose3d test2 = testPose3d.transformBy(new Transform3d(APRILTAG_LEFT_TRANSFORM));

    Pose2d testAtagPos = new Pose2d(Units.inchesToMeters(209.5), Units.inchesToMeters(158.5),
            new Rotation2d(Units.degreesToRadians(0)));

    Transform2d botToTag;

    Pose2d tagPoseRobotRelative = new Pose2d();
    boolean hastarget = false;

    LinearFilter movingaverageX = LinearFilter.movingAverage(15);
    LinearFilter movingaverageY = LinearFilter.movingAverage(15);
    LinearFilter movingaverageTheta = LinearFilter.movingAverage(15);

    private Pose2d targetRight = new Pose2d();
    private Pose2d targetLeft = new Pose2d();

    private Pose2d leftLimelightPoseEst;
    private Pose2d rightLimelightPoseEst;

    boolean updatePose;

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Creates a new auto factory for this drivetrain.
     *
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory() {
        return createAutoFactory((sample, isStart) -> {

        });
    }

    /**
     * Creates a new auto factory for this drivetrain with the given
     * trajectory logger.
     *
     * @param trajLogger Logger for the trajectory
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajLogger) {

        return new AutoFactory(
                () -> getState().Pose,
                this::resetPose,
                this::followPath,
                true,
                this,
                (traj, starting) -> {
                    System.out.println("Trajectory starting? " + starting);
                    if (starting) {
                        trajSamples = traj.samples().toArray(SwerveSample[]::new);
                    } else {
                        trajSamples = new SwerveSample[0];
                    }
                });
    }

    @Logged
    public Pose2d getPose() {
        // if (Robot.isSimulation()) {
        // return new Pose2d(5.5, 2.5, new Rotation2d());
        // }
        return getState().Pose;
    }

    public Command goToPose(ReefSide side) {

        return this.runOnce(() -> {
            thetaController.enableContinuousInput(-180, 180);
            thetaController.reset(getPose().getRotation().getDegrees());
            xController.reset(getPose().getX());
            yController.reset(getPose().getY());

            targetPose2d = getTargetPose(side);
            atPose = false;

            isAligning = true;

            ChassisSpeeds currentSpeeds = getState().Speeds;
            SwerveModuleState[] currentStates = getState().ModuleStates;
            previousSetpoint = new SwerveSetpoint(currentSpeeds, currentStates, DriveFeedforwards.zeros(4));

        }).andThen(this.run(
                () -> {

                    double xSpeed = xController.calculate(getPose().getX(), getTargetPose(side).getX());
                    double ySpeed = yController.calculate(getPose().getY(), getTargetPose(side).getY());
                    double angularSpeed = thetaController.calculate(getPose().getRotation().getDegrees(),
                            getTargetPose(side).getRotation().minus(new Rotation2d(Degrees.of(0))).getDegrees());

                    // previousSetpoint = setpointge
                    ChassisSpeeds speeds = new ChassisSpeeds(xSpeed,
                            ySpeed, angularSpeed);
                    setControl(applyRobotSpeeds.withSpeeds(speeds));
                }).until(
                        () -> (MathUtil.isNear(getPose().getX(), getTargetPose(side).getX(), Units.inchesToMeters(1))
                                && MathUtil.isNear(getPose().getY(), getTargetPose(side).getY(),
                                        Units.inchesToMeters(1)))
                                && MathUtil.isNear(
                                        getPose().getRotation().getDegrees(),
                                        getTargetPose(side).getRotation().getDegrees(), 0.5))
                .andThen(Commands.runOnce(() -> {
                    atPose = true;
                    isAligning = false;
                })));
    }

    public Command goToNearestReefPole() {

        return this.runOnce(() -> {
            isAligning = true;
        }).andThen(goToPose(() -> closestReefPole)).finallyDo(() -> {
            isAligning = false;
        });

    }

    public Command goToPose(Supplier<Pose2d> pose) {

        return this.runOnce(() -> {
            thetaController.enableContinuousInput(-180, 180);
            thetaController.reset(getPose().getRotation().getDegrees());
            xController.reset(getPose().getX());
            yController.reset(getPose().getY());

            targetPose2d = pose.get();

            atPose = false;

        }).andThen(this.run(
                () -> {
                    double xSpeed = xController.calculate(getPose().getX(), pose.get().getX());
                    double ySpeed = yController.calculate(getPose().getY(), pose.get().getY());
                    double angularSpeed = thetaController.calculate(getPose().getRotation().getDegrees(),
                            pose.get().getRotation().minus(new Rotation2d(Degrees.of(0))).getDegrees());
                    ChassisSpeeds speeds = new ChassisSpeeds(xSpeed,
                            ySpeed, angularSpeed);
                    setControl(applyRobotSpeeds.withSpeeds(speeds));
                }).until(
                        () -> (MathUtil.isNear(getPose().getX(), pose.get().getX(), Units.inchesToMeters(1))
                                && MathUtil.isNear(getPose().getY(), pose.get().getY(),
                                        Units.inchesToMeters(1)))
                                && MathUtil.isNear(
                                        getPose().getRotation().getDegrees(),
                                        pose.get().getRotation().getDegrees(), 0.5))
                .andThen(Commands.runOnce(() -> atPose = true)));
    }

    public void stopAligning() {
        isAligning = false;
    }

    public void configMap() {
        poseTagMap.put(TAG_6, 6);
        poseTagMap.put(TAG_7, 7);
        poseTagMap.put(TAG_8, 8);
        poseTagMap.put(TAG_9, 9);
        poseTagMap.put(TAG_10, 10);
        poseTagMap.put(TAG_11, 11);
        poseTagMap.put(TAG_17, 17);
        poseTagMap.put(TAG_18, 18);
        poseTagMap.put(TAG_19, 19);
        poseTagMap.put(TAG_20, 20);
        poseTagMap.put(TAG_21, 21);
        poseTagMap.put(TAG_22, 22);

        for (Pose2d pose : tagPoses) {
            reefPoleLocations.add(pose.transformBy(APRILTAG_LEFT_TRANSFORM));
            reefPoleLocations.add(pose.transformBy(APRILTAG_RIGHT_TRANSFORM));
        }

    }

    public Command goToPose(Pose2d pose) {

        return this.runOnce(() -> {
            thetaController.enableContinuousInput(-180, 180);
            thetaController.reset(getPose().getRotation().getDegrees());
            xController.reset(getPose().getX());
            yController.reset(getPose().getY());

            targetPose2d = pose;

            atPose = false;

        }).andThen(this.run(
                () -> {
                    double xSpeed = xController.calculate(getPose().getX(), pose.getX());
                    double ySpeed = yController.calculate(getPose().getY(), pose.getY());
                    double angularSpeed = thetaController.calculate(getPose().getRotation().getDegrees(),
                            pose.getRotation().minus(new Rotation2d(Degrees.of(0))).getDegrees());
                    ChassisSpeeds speeds = new ChassisSpeeds(xSpeed,
                            ySpeed, angularSpeed);
                    setControl(applyRobotSpeeds.withSpeeds(speeds));
                }).until(
                        () -> (MathUtil.isNear(getPose().getX(), pose.getX(), Units.inchesToMeters(1))
                                && MathUtil.isNear(getPose().getY(), pose.getY(),
                                        Units.inchesToMeters(1)))
                                && MathUtil.isNear(
                                        getPose().getRotation().getDegrees(),
                                        pose.getRotation().getDegrees(), 0.5))
                .andThen(Commands.runOnce(() -> atPose = true)));
    }

    public Command drivePointedAtReef(DoubleSupplier xDoubleSupplier, DoubleSupplier yDoubleSupplier) {
        return this.runOnce(() -> {
            reefController.reset();
            reefController.enableContinuousInput(-180, 180);

        }).andThen(this.run(() -> {
            Transform2d reefOffset = getPose().minus(targetReef);
            Angle targetAngle = Radians.of(Math.atan2(reefOffset.getY(), reefOffset.getX()));
            double angularSpeed = reefController.calculate(getPose().getRotation().getDegrees(),
                    targetAngle.in(Degrees));
            setControl(drive.withVelocityY(yDoubleSupplier.getAsDouble() * MAX_VELO)
                    .withVelocityX(xDoubleSupplier.getAsDouble() * MAX_VELO)
                    .withRotationalRate(angularSpeed));
        }));
    }

    public boolean nearReef() {
        return getPose().getTranslation().getDistance(targetReef.getTranslation()) < Meters.of(3.5).baseUnitMagnitude();
    }

    public boolean getAtPose() {
        return atPose;
    }

    public void setTransforms(Supplier<LevelTarget> target) {
        switch (target.get()) {
            // Further from reef for L4
            case L4 -> {
                APRILTAG_LEFT_TRANSFORM = L4_APRILTAG_LEFT_TRANSFORM;
                APRILTAG_RIGHT_TRANSFORM = L4_APRILTAG_RIGHT_TRANSFORM;
            }
            // For other levels, closer to reef
            default -> {
                APRILTAG_LEFT_TRANSFORM = LOWER_APRILTAG_LEFT_TRANSFORM;
                APRILTAG_RIGHT_TRANSFORM = LOWER_APRILTAG_RIGHT_TRANSFORM;
            }
        }
    }

    public Pose2d getTargetPose(ReefSide side) {
        return side == ReefSide.LEFT ? targetLeft : targetRight;
    }

    public Pose2d solveClosestTagPose() {
        return getPose().nearest(tagPoses);
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Follows the given field-centric path sample with PID.
     *
     * @param sample Sample along the path to follow
     */
    public void followPath(SwerveSample sample) {
        m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

        var pose = getState().Pose;

        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += m_pathXController.calculate(
                pose.getX(), sample.x);
        targetSpeeds.vyMetersPerSecond += m_pathYController.calculate(
                pose.getY(), sample.y);
        targetSpeeds.omegaRadiansPerSecond += m_pathThetaController.calculate(
                pose.getRotation().getRadians(), sample.heading);

        setControl(
                m_pathApplyFieldSpeeds.withSpeeds(targetSpeeds)
                        .withWheelForceFeedforwardsX(sample.moduleForcesX())
                        .withWheelForceFeedforwardsY(sample.moduleForcesY()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    public Command zeroGyro() {
        return this.runOnce(() -> this.resetRotation(zeroRotation));
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }

        // if (LimelightHelpers.getTV("limelight-left")) {

        // limelightEst = LimelightHelpers.getBotPose3d_TargetSpace("limelight-left");

        // measurementX = movingaverageX.calculate(limelightEst.getX());
        // measurementY = movingaverageY.calculate(limelightEst.getZ());
        // measurementTheta =
        // movingaverageTheta.calculate(limelightEst.getRotation().getX());

        // if (Math.abs(measurementY) > 0.25 &&
        // (Math.abs(getState().Speeds.vxMetersPerSecond) < 0.5
        // && Math.abs(getState().Speeds.vyMetersPerSecond) < 0.5)) {

        // if (!hastarget || MathUtil.isNear(this.getState().RawHeading.getDegrees(),
        // measurementTheta, 3)) {
        // hastarget = true;
        // botToTag = new Transform2d(measurementY, -measurementX,
        // new Rotation2d(measurementTheta).plus(new Rotation2d(Degrees.of(180))));

        // targetPose2d =
        // getPose().transformBy(botToTag).transformBy(atagToRightTransform2d);

        // tagPoseRobotRelative = getPose().transformBy(botToTag);
        // }

        // }

        // }

        targetRight = solveClosestTagPose().transformBy(APRILTAG_RIGHT_TRANSFORM);
        targetLeft = solveClosestTagPose().transformBy(APRILTAG_LEFT_TRANSFORM);

        if (!isAligning) {
            closestReefPole = getPose().nearest(reefPoleLocations);
        }

        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            targetReef = RED_REEF;
        } else {
            targetReef = BLUE_REEF;
        }

        // if(/*DriverStation.isAutonomous() && */DriverStation.isDisabled()){
        // if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red){
        // this.resetRotation(Rotation2d.k180deg);
        // }
        // else{
        // this.resetRotation(Rotation2d.kZero);
        // }
        // }

        double yaw = getPose().getRotation().getDegrees();
        LimelightHelpers.SetRobotOrientation("limelight-left", yaw, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate leftEst = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");

        LimelightHelpers.PoseEstimate leftMT1Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");

        nearestTagId = poseTagMap.getOrDefault(solveClosestTagPose(), 0);

        if (leftEst != null && leftEst.tagCount > 0) {
            // log where limelight thinks we are
            leftLimelightPoseEst = leftEst.pose;
            // only add vision measurement if we arent spinning quickly
            if (Math.abs(Units.radiansToDegrees(getState().Speeds.omegaRadiansPerSecond)) < 360) {
                // we dont care a ton about the tag id if we are not aligning

                if (getPose().getTranslation().getDistance(closestReefPole.getTranslation()) > 1.5) {
                    if (!isAligning) {
                        addVisionMeasurement(leftEst.pose, leftEst.timestampSeconds, VecBuilder.fill(.4, .4, 9999999));
                        return;
                    }

                    RawFiducial[] rawFiducials = leftEst.rawFiducials;
                    // if the pose estimate includes the tag we're aligning to
                    for (RawFiducial rawFiducial : rawFiducials) {

                        if (rawFiducial.id == nearestTagId) {
                            addVisionMeasurement(leftEst.pose, leftEst.timestampSeconds,
                                    VecBuilder.fill(.4, .4, 9999999));
                        }
                    }
                }

                else{
                    System.out.println("using mt1");
                    addVisionMeasurement(leftMT1Estimate.pose, leftMT1Estimate.timestampSeconds, VecBuilder.fill(.6, .6, Units.degreesToRadians(3)));
                }

            }
        }

        zeroRotation = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? Rotation2d.kZero
                : Rotation2d.k180deg;

        field.setRobotPose(getState().Pose);

    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement
     *                                 in the form [x, y, theta]ᵀ, with units in
     *                                 meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }
}