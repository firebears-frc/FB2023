package frc.robot.chassis;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Chassis extends SubsystemBase {
    public static class ChassisConstants {
        // Driving
        public static final int RIGHT_FRONT_PORT = 15;
        public static final int RIGHT_BACK_PORT = 17;
        public static final int LEFT_FRONT_PORT = 16;
        public static final int LEFT_BACK_PORT = 18;

        public static final double TRACK_WIDTH = 0.96679; // Meters
        public static final double MAX_VELOCITY = 5.0; // Meters per second
        public static final double SLOW_VELOCITY = 1.0; // Meters per second
        public static final double MAX_ANGULAR_VELOCITY = 8.0; // Radians per second
        public static final double SLOW_ANGULAR_VELOCITY = 2.0; // Radians per second
        public static final double S = 0.15473;
        public static final double V = 2.3007;
        public static final double A = 0.22029;

        // Trajectories
        public static final double MAX_ACCELERATION = 5.0; // Meters per second squared: TODO!
        public static final double MAX_VOLTAGE = 10.0;

        // Charge Station
        public static final double LEVEL_TOLERANCE = 2.0; // degrees
        public static final double ON_TOLERANCE = 10.0; // degrees
        public static final double PITCH_VELOCITY_MAX = 0.2; // degrees per cycle
    }

    // Driving
    private ChassisSide left, right;
    private SimpleMotorFeedforward feedforward;
    private DifferentialDriveKinematics kinematics;

    // Localization
    private AHRS navX;
    private DifferentialDrivePoseEstimator poseEstimator;
    private Field2d field;

    // Trajectories
    private DifferentialDriveVoltageConstraint constraint;
    private TrajectoryConfig config;
    private RamseteController controller;

    // Charge Station
    private double lastPitch = 0;
    private double pitchVelocity = 0;

    public Chassis() {
        feedforward = new SimpleMotorFeedforward(ChassisConstants.S, ChassisConstants.V, ChassisConstants.A);
        kinematics = new DifferentialDriveKinematics(ChassisConstants.TRACK_WIDTH);

        left = new ChassisSide(ChassisConstants.LEFT_FRONT_PORT, ChassisConstants.LEFT_BACK_PORT, feedforward);
        addChild("Left", left);
        right = new ChassisSide(ChassisConstants.RIGHT_FRONT_PORT, ChassisConstants.RIGHT_BACK_PORT, feedforward);
        addChild("Right", right);
        try {
            navX = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            DriverStation.reportError(ex.getMessage(), true);
        }

        poseEstimator = new DifferentialDrivePoseEstimator(kinematics,
                navX.getRotation2d(), 0, 0, new Pose2d());
        field = new Field2d();
        addChild("Field", field);

        constraint = new DifferentialDriveVoltageConstraint(feedforward, kinematics, ChassisConstants.MAX_VOLTAGE);
        config = new TrajectoryConfig(ChassisConstants.MAX_VELOCITY, ChassisConstants.MAX_ACCELERATION);
        config.setKinematics(kinematics);
        config.addConstraint(constraint);
        controller = new RamseteController(); // Uses default constants
    }

    @Override
    public void periodic() {
        double leftDistance = left.update();
        double rightDistance = right.update();

        poseEstimator.update(navX.getRotation2d(), leftDistance, rightDistance);
        field.setRobotPose(getPose());

        double currentPitch = getPitch();
        pitchVelocity = currentPitch - lastPitch;
        lastPitch = currentPitch;
    }

    /****************** DRIVING ******************/
    public void drive(ChassisSpeeds chassisSpeeds) {
        tankDrive(kinematics.toWheelSpeeds(chassisSpeeds));
    }

    public void tankDrive(DifferentialDriveWheelSpeeds wheelSpeeds) {
        tankDrive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        left.setSetpoint(leftSpeed);
        right.setSetpoint(rightSpeed);
    }

    public void setBrakeMode(boolean brakeMode) {
        left.setBrakeMode(brakeMode);
        right.setBrakeMode(brakeMode);
    }

    /****************** LOCALIZATION ******************/
    public void visionPose(Pose2d pose, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(pose, timestampSeconds);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /****************** TRAJECTORIES ******************/
    public Trajectory generateTrajectory(Pose2d start, Pose2d end) {
        return generateTrajectory(start, end, false);
    }

    public Trajectory generateTrajectory(Pose2d start, Pose2d end, boolean reversed) {
        return generateTrajectory(start, new ArrayList<>(), end, reversed);
    }

    public Trajectory generateTrajectory(Pose2d start, List<Translation2d> interior, Pose2d end, boolean reversed) {
        config.setReversed(reversed);
        return TrajectoryGenerator.generateTrajectory(start, interior, end, config);
    }

    public RamseteCommand generateRamseteCommand(Trajectory trajectory) {
        return new RamseteCommand(
                trajectory,
                this::getPose,
                controller,
                kinematics,
                this::tankDrive,
                this);
    }

    /****************** CHARGE STATION ******************/
    public double getPitch() {
        return navX.getPitch();
    }

    public boolean isLevel() {
        return Math.abs(getPitch()) < ChassisConstants.LEVEL_TOLERANCE;
    }

    public boolean isOnChargeStation() {
        return Math.abs(getPitch()) > ChassisConstants.ON_TOLERANCE;
    }

    private double getPitchVelocity() {
        return pitchVelocity;
    }

    public boolean isNotPitching() {
        return Math.abs(getPitchVelocity()) < ChassisConstants.PITCH_VELOCITY_MAX;
    }
}
