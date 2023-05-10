package frc.robot;

import frc.robot.auto.OneElementWithMobility;
import frc.robot.auto.OneElementWithMobilityAndEngaged;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Schlucker;
import frc.robot.subsystems.SchluckerBag;
import frc.robot.subsystems.SchluckerNeo550;
import frc.robot.subsystems.Vision;
import frc.robot.util.GamePiece;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    public class Constants {
        public static final int JOYSTICK_PORT = 0;
        public static final int CONTROLLER_PORT = 1;
    }

    private final Chassis chassis;
    private final Arm arm;
    private final Schlucker schlucker;
    private final Vision vision;
    private final Lights lights;

    private final CommandJoystick joystick;
    private final CommandXboxController controller;
    private final SendableChooser<Command> autoSelector;
    private final DataLog log;
    private final StringLogEntry autoLog;

    public RobotContainer() {
        DataLogManager.start();
        log = DataLogManager.getLog();

        chassis = new Chassis(log);
        arm = new Arm(log);
        schlucker = new SchluckerBag(log); // new SchluckerNeo550();
        vision = new Vision(chassis::visionPose);
        lights = new Lights(schlucker::getHeldItem, schlucker::getWantedItem, chassis::isLevel,
                chassis::isOnChargeStation, chassis::isNotPitching);

        joystick = new CommandJoystick(Constants.JOYSTICK_PORT);
        controller = new CommandXboxController(Constants.CONTROLLER_PORT);
        DriverStation.startDataLog(log);

        autoSelector = new SendableChooser<>();
        autoSelector.setDefaultOption("1 Cone w/ Mobility & Engage",
                new OneElementWithMobilityAndEngaged(chassis, arm, schlucker, GamePiece.CONE));
        autoSelector.addOption("1 Cube w/ Mobility & Engage",
                new OneElementWithMobilityAndEngaged(chassis, arm, schlucker, GamePiece.CUBE));
        autoSelector.addOption("1 Cone w/ Mobility",
                new OneElementWithMobility(chassis, arm, schlucker, GamePiece.CONE));
        autoSelector.addOption("1 Cube w/ Mobility",
                new OneElementWithMobility(chassis, arm, schlucker, GamePiece.CUBE));
        autoLog = new StringLogEntry(log, "/Auto/Command");

        configureButtonBindings();

        displayGitInfo(log);
    }

    private void configureButtonBindings() {
        chassis.setDefaultCommand(
                chassis.defaultCommand(joystick::getY, joystick::getX,
                        () -> joystick.getHID().getRawButton(1)));

        arm.setDefaultCommand(arm.defaultCommand(controller::getLeftY, controller::getRightY));

        // Arm target point commands
        controller.povUp().onTrue(arm.substation());
        controller.povRight()
                .onTrue(arm.mid().andThen(schlucker.eject()))
                .onFalse(schlucker.stop());
        controller.povDown().onTrue(arm.groundCone());
        controller.povLeft()
                .onTrue(arm.high().andThen(schlucker.eject()))
                .onFalse(schlucker.stop());
        controller.leftBumper().onTrue(arm.groundCube());
        controller.b().onTrue(arm.stow());

        // Schlucker commands
        controller.a()
                .onTrue(schlucker.intakeCone())
                .onFalse(schlucker.hold());
        controller.x()
                .onTrue(schlucker.intakeCube())
                .onFalse(schlucker.hold());
        controller.y()
                .onTrue(schlucker.eject())
                .onFalse(schlucker.stop());

        joystick.button(3).onTrue(schlucker.wantCone());
        joystick.button(4).onTrue(schlucker.wantCube());
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
