package frc.robot;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;

public class RobotContainer {
    private static RobotContainer m_robotContainer = null;
    public final Chassis chassis;

    private RobotContainer() {
        chassis = new Chassis();

        configureButtonBindings();

        displayGitInfo();
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

    public static RobotContainer getInstance() {
        if (m_robotContainer == null) {
            m_robotContainer = new RobotContainer();
        }
        return m_robotContainer;
    }

    private void configureButtonBindings() {
    }
}
