package frc.robot;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;

public class RobotContainer {
    private static class Constants {
        public static int JOYSTICK_PORT = 0;
    }

    private final Chassis chassis;
    private final Joystick joystick;

    public RobotContainer() {
        chassis = new Chassis();
        joystick = new Joystick(Constants.JOYSTICK_PORT);

        configureButtonBindings();

        displayGitInfo();
    }

    private void configureButtonBindings() {
        chassis.setDefaultCommand(new RunCommand(() -> {
            double forward = joystick.getY() * -1.0;
            double rotation = joystick.getX() * -1.0;

            chassis.arcadeDrive(forward, rotation);
        }, chassis));
    }

    public Command getAutonomousCommand() {
        Trajectory trajectory = chassis.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d()),
            new Pose2d(1.0, 0, new Rotation2d())
        );
        return chassis.generateRamseteCommand(trajectory);
    }

    private String getFileContents(String filename) {
        // Create the file object
        File file = new File(Filesystem.getDeployDirectory(), filename);
        // Open the file stream
        try (FileInputStream inputStream = new FileInputStream(file)) {
            // Prepare the buffer
            byte[] data = new byte[(int) file.length()];
            // Read the data
            data = inputStream.readAllBytes();
            // Format into string and return
            return new String(data, "UTF-8");
        } catch (IOException e) {
            // Print exception and return
            e.printStackTrace();
            return "Unknown";
        }
    }

    private void displayGitInfo() {
        // Get the branch name and display on the dashboard
        String branchName = getFileContents("branch.txt");
        SmartDashboard.putString("Branch Name", branchName);

        // Get the commit hash and display on the dashboard
        String commitHash = getFileContents("commit.txt");
        SmartDashboard.putString("Commit Hash", commitHash.substring(0, 8));
    }
}
