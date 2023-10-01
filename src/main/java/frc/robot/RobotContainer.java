package frc.robot;

import frc.robot.subsystems.Chassis;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class RobotContainer {
    public class Constants {
        public static final int JOYSTICK_1_PORT = 0;
        public static final int JOYSTICK_2_PORT = 1;
        public static final int CONTROLLER_PORT = 2;

        public static final double JOYSTICK_DEADBAND = 0.05;
    }

    private final Chassis chassis;

    private final CommandJoystick joystick_1;
    private final CommandJoystick joystick_2;
    private final SendableChooser<Command> autoSelector;
    private final DataLog log;
    private final StringLogEntry autoLog;

    public RobotContainer() {
        DataLogManager.start();
        log = DataLogManager.getLog();

        chassis = new Chassis(log);

        joystick_1 = new CommandJoystick(Constants.JOYSTICK_1_PORT);
        joystick_2 = new CommandJoystick(Constants.JOYSTICK_2_PORT);
        DriverStation.startDataLog(log);

        autoSelector = new SendableChooser<>();
        autoSelector.addOption("Test Auto Path", chassis.driveTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                new Pose2d(3, 0, new Rotation2d(0)),
                false));
        autoLog = new StringLogEntry(log, "/Auto/Command");

        configureButtonBindings(log);

        displayGitInfo(log);
    }

    private void configureButtonBindings(DataLog log) {
        chassis.setDefaultCommand(new Chassis.DriveCommand(
                chassis,
                () -> new ChassisSpeeds(
                        -MathUtil.applyDeadband(joystick_1.getY(), Constants.JOYSTICK_DEADBAND),
                        -MathUtil.applyDeadband(joystick_1.getX(), Constants.JOYSTICK_DEADBAND),
                        -MathUtil.applyDeadband(joystick_2.getX(), Constants.JOYSTICK_DEADBAND)),
                () -> joystick_1.getHID().getRawButton(1),
                true,
                true,
                log));

        joystick_1.trigger().whileTrue(chassis.turtle());
        joystick_2.trigger().whileTrue(chassis.zeroHeading());
    }

    public Command getAutonomousCommand() {
        Command auto = autoSelector.getSelected();
        autoLog.append(auto.getName());
        return auto;
    }

    private static void displayGitInfo(DataLog log) {
        final NetworkTable table = NetworkTableInstance.getDefault().getTable("Build Info");
        table.getEntry("Branch Name").setString(BuildConstants.GIT_BRANCH);
        table.getEntry("Commit Hash (Short)").setString(BuildConstants.GIT_SHA.substring(0, 8));
        table.getEntry("Commit Hash (Full)").setString(BuildConstants.GIT_SHA);
        table.getEntry("Dirty").setBoolean(BuildConstants.DIRTY == 1);
        table.getEntry("Build Date & Time").setString(BuildConstants.BUILD_DATE);

        log.setMetadata(0, BuildConstants.GIT_BRANCH);
        log.setMetadata(1, BuildConstants.GIT_SHA);
        log.setMetadata(2, String.valueOf(BuildConstants.DIRTY == 1));
        log.setMetadata(3, BuildConstants.BUILD_DATE);
    }
}
