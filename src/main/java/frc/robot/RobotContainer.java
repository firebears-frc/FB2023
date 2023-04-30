package frc.robot;

import frc.robot.auto.OneElementWithMobility;
import frc.robot.auto.OneElementWithMobilityAndEngaged;
import frc.robot.chassis.Chassis;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Schlucker;
import frc.robot.subsystems.Vision;
import frc.robot.util.GamePiece;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;

public class RobotContainer {
    public class RobotConstants {
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

    public RobotContainer() {
        chassis = new Chassis();
        arm = new Arm();
        schlucker = new Schlucker();
        vision = new Vision(chassis::visionPose);
        lights = new Lights(schlucker::getHeldItem, schlucker::getWantedItem, chassis::isLevel,
                chassis::isOnChargeStation, chassis::isNotPitching);
        joystick = new CommandJoystick(RobotConstants.JOYSTICK_PORT);
        controller = new CommandXboxController(RobotConstants.CONTROLLER_PORT);
        autoSelector = new SendableChooser<>();

        autoSelector.setDefaultOption("1 Cone w/ Mobility & Engage",
                new OneElementWithMobilityAndEngaged(chassis, arm, schlucker, GamePiece.CONE));
        autoSelector.addOption("1 Cube w/ Mobility & Engage",
                new OneElementWithMobilityAndEngaged(chassis, arm, schlucker, GamePiece.CUBE));
        autoSelector.addOption("1 Cone w/ Mobility",
                new OneElementWithMobility(chassis, arm, schlucker, GamePiece.CONE));
        autoSelector.addOption("1 Cube w/ Mobility",
                new OneElementWithMobility(chassis, arm, schlucker, GamePiece.CUBE));
        autoSelector.addOption("Drive Backwards", chassis.driveDistance(-2.0));
        autoSelector.addOption("Drive Forwards", chassis.driveDistance(2.0));

        configureButtonBindings();

        displayGitInfo();
    }

    private void configureButtonBindings() {
        chassis.setDefaultCommand(
                chassis.defaultCommand(joystick::getY, joystick::getX, () -> joystick.getHID().getRawButton(1)));

        arm.setDefaultCommand(arm.defaultCommand(controller::getLeftY, controller::getRightY));

        // Arm target point commands
        controller.povUp().onTrue(arm.substation());
        controller.povRight()
                .onTrue(arm.mid().andThen(schlucker.eject()))
                .onFalse(schlucker.stop());
        controller.povDown().onTrue(arm.ground());
        controller.povLeft()
                .onTrue(arm.high().andThen(schlucker.eject()))
                .onFalse(schlucker.stop());
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
        return autoSelector.getSelected();
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
