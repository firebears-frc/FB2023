package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static RobotContainer m_robotContainer = null;

  public final Lights m_lights;
  public final Vision m_vision;
  public final Schlucker m_schlucker;
  public final Arm m_arm;
  public final Chassis m_chassis;

  private final XboxController xboxController = new XboxController(1);
  private final Joystick joystick = new Joystick(0);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private RobotContainer() {

    m_lights = new Lights();
    m_vision = new Vision("MainC");
    m_schlucker = new Schlucker();
    m_arm = new Arm();
    m_chassis = new Chassis();

    SmartDashboard.putData("Autonomous Command", new AutonomousCommand());

    configureButtonBindings();

    m_chassis.setDefaultCommand(new ChassisDriveCommand(m_chassis));

    m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());

    SmartDashboard.putData("Auto Mode", m_chooser);

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

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //JoystickButton yButton = new JoystickButton(joystick, XboxController.Button.kY.value);
    //yButton.onTrue(new ChassisResetEncoderCommand(m_chassis));

  }

  public Joystick getJoystick() {
    return joystick;
  }

  public XboxController getxboxController() {
    return xboxController;
  }

  /**
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

}
