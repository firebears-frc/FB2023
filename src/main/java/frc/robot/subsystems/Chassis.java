package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.ChassisConstants;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
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
    private ChassisSide left, right;
    private AHRS navX;

    private SimpleMotorFeedforward feedforward;
    private DifferentialDriveKinematics kinematics;
    private DifferentialDriveOdometry odometry;
    private DifferentialDriveVoltageConstraint constraint;
    private TrajectoryConfig config;
    private RamseteController controller;
    private Field2d field;

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

        odometry = new DifferentialDriveOdometry(
                navX.getRotation2d(), 0.0, 0.0);
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

        odometry.update(navX.getRotation2d(), leftDistance, rightDistance);
        field.setRobotPose(odometry.getPoseMeters());
    }

    public void resetPose(Pose2d pose) {
        left.resetDistance();
        right.resetDistance();
        odometry.resetPosition(
                navX.getRotation2d(),
                0.0, 0.0,
                pose);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void arcadeDrive(double forward, double rotation) {
        drive(new ChassisSpeeds(
                forward * ChassisConstants.MAX_VELOCITY,
                0, // No sideways
                rotation * ChassisConstants.MAX_ANGULAR_VELOCITY));
    }

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

    public Trajectory generateTrajectory(Pose2d start, Pose2d end) {
        return generateTrajectory(start, new ArrayList<>(), end);
    }

    public Trajectory generateTrajectory(Pose2d start, List<Translation2d> interior, Pose2d end) {
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
}
