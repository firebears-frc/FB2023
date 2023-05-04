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
        table.getEntry("Branch Name").setString(BuildConstants.GIT_BRANCH);
        table.getEntry("Commit Hash (Short)").setString(BuildConstants.GIT_SHA.substring(0, 8));
        table.getEntry("Commit Hash (Full)").setString(BuildConstants.GIT_SHA);
        table.getEntry("Dirty").setBoolean(BuildConstants.DIRTY == 1);
        table.getEntry("Build Date & Time").setString(BuildConstants.BUILD_DATE);
    }
}
