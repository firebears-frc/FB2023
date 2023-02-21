package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class AutoCommand extends CommandBase {
    private static Trajectory generateTrajectory() {
        Pose2d start = new Pose2d();
        Pose2d end = new Pose2d(0.0, 0.0, new Rotation2d());

        TrajectoryConfig config = new TrajectoryConfig(
                Chassis.Constants.MAX_VELOCITY,
                Chassis.Constants.MAX_ACCELERATION);

        return TrajectoryGenerator.generateTrajectory(
                start,
                new ArrayList<>(),
                end,
                config);
    }

    private final RamseteController controller;

    public AutoCommand() {
        controller = new RamseteController(2.0, 0.7);
    }
}
