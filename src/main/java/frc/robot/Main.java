package frc.robot;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
    private Main() {
    }

    public static void main(String... args) {
        displayGitInfo();
        RobotBase.startRobot(Robot::new);
    }

    private static String getFileContents(String filename) {
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

    private static void displayGitInfo() {
        final NetworkTable table = NetworkTableInstance.getDefault().getTable("Git Info");
        // Get the branch name and display on the dashboard
        String branchName = getFileContents("branch.txt");
        table.getEntry("Branch Name").setValue(branchName);

        // Get the commit hash and display on the dashboard
        String commitHash = getFileContents("commit.txt");
        table.getEntry("Commit Hash (Short)").setValue(commitHash.substring(0, 8));
        table.getEntry("Commit Hash (Full)").setValue(commitHash);
    }
}
