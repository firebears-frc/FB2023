package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.chassis.Chassis;

public class AutoDriveCommand extends SequentialCommandGroup {
    public AutoDriveCommand(Chassis chassis, double distance) {
        Trajectory trajectory = chassis.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(1.0, 0, new Rotation2d()),
                distance < 0);
        addCommands(chassis.generateRamseteCommand(trajectory));
    }
}
