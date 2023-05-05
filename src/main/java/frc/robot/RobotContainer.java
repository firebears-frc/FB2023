package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

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
  private final UsbCamera usbcamera;
  private final XboxController xboxController = new XboxController(1);
  private final Joystick joystick = new Joystick(0);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private RobotContainer() {

    m_lights = new Lights();
    m_schlucker = new Schlucker();
    m_arm = new Arm();
    m_chassis = new Chassis();
    m_vision = new Vision("MainC", m_chassis);
    usbcamera = CameraServer.startAutomaticCapture();
    usbcamera.setResolution(320, 240);
    configureButtonBindings();

    m_chassis.setBrakemode(false);
    m_chassis.setDefaultCommand(new ChassisDriveCommand(m_chassis));
    m_arm.setDefaultCommand(new ArmManualCommand(m_arm, xboxController));

    m_chooser.setDefaultOption("Auto CUBE and Balance", new AutoCubeAndBalanceCommand(m_chassis, m_schlucker, m_arm));
    m_chooser.addOption("Auto cone and Balance", new AutoConeAndBalanceCommand(m_chassis, m_schlucker, m_arm));
    m_chooser.addOption("Cube", new AutoCubeGetOutCommand(m_chassis, m_schlucker, m_arm));
    m_chooser.addOption("Cone", new AutoConeGetOutCommand(m_chassis, m_schlucker, m_arm));
    

    m_chooser.addOption("Auto Balance", new AutoBalanceRoutine(m_chassis));

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

  public void armReset() {
    m_arm.setElbowSetpoint(m_arm.getElbowAngle());
    m_arm.setShoulderSetpoint(m_arm.getShoulderAngle());
  }

  private void displayGitInfo() {
    if (Constants.DEBUG) {
      // Get the branch name and display on the dashboard
      String branchName = getFileContents("branch.txt");
      SmartDashboard.putString("Branch Name", branchName);

      // Get the commit hash and display on the dashboard
      String commitHash = getFileContents("commit.txt");
      SmartDashboard.putString("Commit Hash", commitHash.substring(0, 8));
    }
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

    JoystickButton fiveButton = new JoystickButton(joystick, 5); // DO NOT DELETE
    fiveButton.onTrue(new AutonomousBalanceCommand(m_chassis)); // DO NOT DELETE

    //break mode
  
  
    /* Lights Controls */

    JoystickButton sixButton = new JoystickButton(joystick, 5);
    sixButton.onTrue(new InstantCommand(m_lights::showTeam, m_lights));

    JoystickButton sevenButton = new JoystickButton(joystick, 6);
    sevenButton.onTrue(new InstantCommand(m_lights::showCone, m_lights));

    JoystickButton eightButton = new JoystickButton(joystick, 7);
    eightButton.onTrue(new InstantCommand(m_lights::showCube, m_lights));


   JoystickButton twobutton = new JoystickButton(joystick, 2);
   twobutton.onTrue(new InstantCommand(m_chassis::toggleSlowMode, m_chassis));

    JoystickButton triggerButton = new JoystickButton(joystick, 1);
    triggerButton.onTrue(new InstantCommand(m_chassis::toggleBrakeMode,m_chassis));

    // Buttons

    // A button = picks up cone and drops cube
    JoystickButton xboxAButton = new JoystickButton(xboxController, XboxController.Button.kA.value);
    xboxAButton.onTrue(new InstantCommand(m_schlucker::intakeCone, m_schlucker));
    xboxAButton.onFalse(new InstantCommand(m_schlucker::hold, m_schlucker));

    // B button = reset position (stow)
    JoystickButton xboxBButton = new JoystickButton(xboxController, XboxController.Button.kB.value);
    xboxBButton.onTrue(
        (new ArmShoulderSetpointCommand(20, m_arm))
        .andThen(new ArmElbowSetpointCommand(220, m_arm)
        .andThen(new WaitCommand(1))
        .andThen(new ArmShoulderSetpointCommand(0, m_arm))
        .andThen(new ArmElbowSetpointCommand(209, m_arm)
        )));

    // X button = picks up cube and drops cone
    JoystickButton xboxXButton = new JoystickButton(xboxController, XboxController.Button.kX.value);
    xboxXButton.onTrue(new InstantCommand(m_schlucker::intakeCube, m_schlucker));
    xboxXButton.onFalse(new InstantCommand(m_schlucker::hold, m_schlucker));

    // Y button = eject button
    JoystickButton xboxYButton = new JoystickButton(xboxController, XboxController.Button.kY.value);
    xboxYButton.onTrue(new InstantCommand(m_schlucker::eject, m_schlucker));
    xboxYButton.onFalse(new InstantCommand(m_schlucker::stop, m_schlucker));

    // Dpad

    // Substation pickup
    POVButton xboxDpadUpButton = new POVButton(xboxController, 0);
    xboxDpadUpButton.onTrue((new ArmShoulderSetpointCommand(65, m_arm))
        .andThen(new ArmElbowSetpointCommand(264, m_arm)));

    // Mid level node
    POVButton xboxDpadRightButton = new POVButton(xboxController, 90);
    xboxDpadRightButton.onTrue((new ArmShoulderSetpointCommand(79, m_arm))
        .andThen(new ArmElbowSetpointCommand(265, m_arm)));

    // Cone Ground pickup
    POVButton xboxDpadDownButton = new POVButton(xboxController, 180);
    xboxDpadDownButton.onTrue((new ArmShoulderSetpointCommand(110, m_arm))
        .andThen(new ArmElbowSetpointCommand(230, m_arm)));

    // Cone Ground pickup
    JoystickButton xboxLeftBumperButton = new JoystickButton(xboxController, XboxController.Button.kLeftBumper.value);
    xboxLeftBumperButton.onTrue((new ArmShoulderSetpointCommand(127, m_arm))
          .andThen(new ArmElbowSetpointCommand(224, m_arm)));

    // High level mode
    POVButton xboxDpadLeftButton = new POVButton(xboxController, 270);
    xboxDpadLeftButton.onTrue((new ArmShoulderSetpointCommand(112, m_arm))
        .andThen(new ArmElbowSetpointCommand(329, m_arm)));
  }

  public XboxController getXboxController() {
    return xboxController;
  }

  public Joystick getJoystick() {
    return joystick;
  }

  /**
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

}
