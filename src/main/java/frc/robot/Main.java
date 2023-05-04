package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
    private Main() {
    }

    public static void main(String... args) {
        displayGitInfo();
        RobotBase.startRobot(Robot::new);
    }

    private static void displayGitInfo() {
        final NetworkTable table = NetworkTableInstance.getDefault().getTable("Build Info");
        table.getEntry("Branch Name").setValue(BuildConstants.GIT_BRANCH);
        table.getEntry("Commit Hash (Short)").setValue(BuildConstants.GIT_SHA.substring(0, 8));
        table.getEntry("Commit Hash (Full)").setValue(BuildConstants.GIT_SHA);
        table.getEntry("Build Time").setValue(BuildConstants.BUILD_DATE);
    }
}
