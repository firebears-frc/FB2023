package frc.robot.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class Trajectories {
    private static final class Constants {
        public static final double MAX_AUTO_VELOCITY = 4.5; // meters per second
        public static final double MAX_AUTO_ACCELERATION = 3.0; // meters per second squared
        public static final double MAX_AUTO_ANGULAR_VELOCITY = Math.PI; // radians per second
        public static final double MAX_AUTO_ANGULAR_ACCELERATION = Math.PI; // radians per second squared
        public static final double X_CONTROLLER_P = 1.0;
        public static final double Y_CONTROLLER_P = 1.0;
        public static final double R_CONTROLLER_P = 1.0;
    }

    private final SwerveDriveKinematics kinematics;
    private final Supplier<Pose2d> poseSupplier;
    private final Consumer<SwerveModuleState[]> stateConsumer;
    private final TrapezoidProfile.Constraints rotationConstraints;
    private final TrajectoryConfig config;
    private final Subsystem requirement;

    public Trajectories(SwerveDriveKinematics kinematics, Supplier<Pose2d> poseSupplier,
            Consumer<SwerveModuleState[]> stateConsumer, Subsystem requirement) {
        this.kinematics = kinematics;
        this.poseSupplier = poseSupplier;
        this.stateConsumer = stateConsumer;
        this.requirement = requirement;
        rotationConstraints = new TrapezoidProfile.Constraints(Constants.MAX_AUTO_ANGULAR_VELOCITY,
                Constants.MAX_AUTO_ANGULAR_ACCELERATION);
        config = new TrajectoryConfig(Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION);
        config.setKinematics(kinematics);
    }

    private Trajectory generateTrajectory(Pose2d start, Pose2d end, boolean reversed) {
        return generateTrajectory(start, new ArrayList<>(), end, reversed);
    }

    private Trajectory generateTrajectory(Pose2d start, List<Translation2d> interior, Pose2d end, boolean reversed) {
        config.setReversed(reversed);
        return TrajectoryGenerator.generateTrajectory(start, interior, end, config);
    }

    private SwerveControllerCommand generateSwerveControllerCommand(Trajectory trajectory) {
        return new SwerveControllerCommand(
                trajectory,
                poseSupplier,
                kinematics,
                new PIDController(Constants.X_CONTROLLER_P, 0.0, 0.0),
                new PIDController(Constants.Y_CONTROLLER_P, 0.0, 0.0),
                new ProfiledPIDController(Constants.R_CONTROLLER_P, 0.0, 0.0, rotationConstraints),
                stateConsumer,
                requirement);
    }

    public Command driveTrajectory(Pose2d start, Pose2d end, boolean reversed) {
        Trajectory trajectory = generateTrajectory(start, end, reversed);
        return generateSwerveControllerCommand(trajectory);
    }

    public Command driveTrajectory(Pose2d start, List<Translation2d> interior, Pose2d end, boolean reversed) {
        Trajectory trajectory = generateTrajectory(start, interior, end, reversed);
        return generateSwerveControllerCommand(trajectory);
    }
}
