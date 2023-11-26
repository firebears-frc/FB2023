package frc.robot.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class Drive extends SubsystemBase {
    private static final class Constants {
        public static final double MAX_AUTO_VELOCITY = 4.5; // meters per second
        public static final double MAX_AUTO_ACCELERATION = 3.0; // meters per second squared
        public static final double MAX_AUTO_ANGULAR_VELOCITY = Math.PI; // radians per second
        public static final double MAX_AUTO_ANGULAR_ACCELERATION = Math.PI; // radians per second squared
        public static final double X_CONTROLLER_P = 1.0;
        public static final double Y_CONTROLLER_P = 1.0;
        public static final double R_CONTROLLER_P = 1.0;
    }

    private final Chassis chassis;
    private final Localization localization;

    public Drive() {
        chassis = new Chassis();
        localization = new Localization(chassis.getKinematics(), chassis.getModulePositions());
    }

    @Override
    public void periodic() {
        localization.periodic(chassis.getModulePositions());
    }

    public Command zeroHeading() {
        return runOnce(() -> {
            Pose2d pose = localization.getPose();
            pose = new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(0.0));
            localization.setPose(pose, chassis.getModulePositions());
        });
    }

    public Command setPose(Pose2d pose) {
        return runOnce(() -> localization.setPose(pose, chassis.getModulePositions()));
    }

    public Command turtle() {
        return startEnd(chassis::setX, null);
    }

    public Command defaultCommand(Supplier<ChassisSpeeds> commandSupplier, boolean slowMode) {
        return new DefaultCommand(commandSupplier,
                speeds -> chassis.driveFieldRelative(speeds, localization.getRawYaw()), slowMode, this);
    }

    public Command driveTrajectory(Pose2d start, Pose2d end, boolean reversed) {
        return driveTrajectory(start, new ArrayList<>(), end, reversed);
    }

    public Command driveTrajectory(Pose2d start, List<Translation2d> interior, Pose2d end, boolean reversed) {
        TrajectoryConfig config = new TrajectoryConfig(Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION);
        config.setKinematics(chassis.getKinematics());

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, interior, end, config);

        return new SwerveControllerCommand(
                trajectory,
                localization::getPose,
                chassis.getKinematics(),
                new PIDController(Constants.X_CONTROLLER_P, 0.0, 0.0),
                new PIDController(Constants.Y_CONTROLLER_P, 0.0, 0.0),
                new ProfiledPIDController(Constants.R_CONTROLLER_P, 0.0, 0.0,
                        new TrapezoidProfile.Constraints(Constants.MAX_AUTO_ANGULAR_VELOCITY,
                                Constants.MAX_AUTO_ANGULAR_ACCELERATION)),
                chassis::swerveDrive,
                this);
    }
}
