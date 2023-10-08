package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmElbowSetpointCommand;
import frc.robot.commands.ArmManualCommand;
import frc.robot.commands.ArmShoulderSetpointCommand;
import frc.robot.commands.AutoBalanceRoutine;
import frc.robot.commands.AutoConeAndBalanceCommand;
import frc.robot.commands.AutoConeGetOutCommand;
import frc.robot.commands.AutoCubeAndBalanceCommand;
import frc.robot.commands.AutoCubeGetOutCommand;
import frc.robot.commands.AutonomousBalanceCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Schlucker;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

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

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  public final Lights m_lights;
  public final Vision m_vision;
  public final Schlucker m_schlucker;
  public final Arm m_arm;
  private final UsbCamera usbcamera;
  private final XboxController xboxController = new XboxController(2);
  // The driver's controller
  private final CommandJoystick one = new CommandJoystick(0);
  private final CommandJoystick two = new CommandJoystick(1);

  private final LoggedDashboardChooser<Command> m_chooser = new LoggedDashboardChooser<>("Auto Mode");

  private RobotContainer() {

    m_lights = new Lights();
    m_schlucker = new Schlucker();
    m_arm = new Arm();
    m_vision = new Vision("MainC", m_robotDrive);
    usbcamera = CameraServer.startAutomaticCapture();
    usbcamera.setResolution(320, 240);
    configureButtonBindings();

    m_arm.setDefaultCommand(new ArmManualCommand(m_arm, xboxController));

    m_chooser.addDefaultOption("Auto CUBE and Balance", new AutoCubeAndBalanceCommand(m_robotDrive, m_schlucker, m_arm));
    m_chooser.addOption("Auto cone and Balance", new AutoConeAndBalanceCommand(m_robotDrive, m_schlucker, m_arm));
    m_chooser.addOption("Cube", new AutoCubeGetOutCommand(m_robotDrive, m_schlucker, m_arm));
    m_chooser.addOption("Cone", new AutoConeGetOutCommand(m_robotDrive, m_schlucker, m_arm));
  
    m_chooser.addOption("Auto Balance", new AutoBalanceRoutine(m_robotDrive));


    // Configure default commands
    m_robotDrive.setDefaultCommand(
            // The left stick controls translation of the robot.
            // Turning is controlled by the X axis of the right stick.
            new RunCommand(
                    () -> m_robotDrive.drive(
                            -MathUtil.applyDeadband(one.getY(), OIConstants.kDriveDeadband),
                            -MathUtil.applyDeadband(one.getX(), OIConstants.kDriveDeadband),
                            -MathUtil.applyDeadband(two.getX(), OIConstants.kDriveDeadband),
                            true, true),
                    m_robotDrive));
  }

  public void armReset() {
    m_arm.setElbowSetpoint(m_arm.getElbowAngle());
    m_arm.setShoulderSetpoint(m_arm.getShoulderAngle());
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
    one.trigger().whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    two.trigger().whileTrue(new RunCommand(
            () -> m_robotDrive.zeroHeading(), m_robotDrive));

    one.button(5).onTrue(new AutonomousBalanceCommand(m_robotDrive)); // DO NOT DELETE

  
  
    /* Lights Controls */
    one.button(5).onTrue(new InstantCommand(m_lights::showTeam, m_lights));
    one.button(6).onTrue(new InstantCommand(m_lights::showCone, m_lights));

    one.button(7).onTrue(new InstantCommand(m_lights::showCube, m_lights));
    //one.button(2).onTrue(new InstantCommand(m_robotDrive::toggleSlowMode, m_robotDrive));

    one.trigger().onTrue(new InstantCommand(m_robotDrive::toggleBrakeMode, m_robotDrive));

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

  /**
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.get();
  }
}
