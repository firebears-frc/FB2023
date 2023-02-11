package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {

  private static RobotContainer m_robotContainer = new RobotContainer();

  public final Lights m_lights = new Lights();
  public final Vision m_vision = new Vision();
  public final Schlucker m_schlucker = new Schlucker();
  public final Arm m_arm = new Arm();
  public final Chassis m_chassis = new Chassis();

  private final XboxController xboxController2 = new XboxController(1);
  private final XboxController xboxController1 = new XboxController(0);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private RobotContainer() {

    SmartDashboard.putData("Autonomous Command", new AutonomousCommand());

    configureButtonBindings();

    m_chassis.setDefaultCommand(new ChassisDriveCommand(m_chassis));

    m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());

    SmartDashboard.putData("Auto Mode", m_chooser);
  }

  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }

  public XboxController getxboxController1() {
    return xboxController1;
  }

  public XboxController getxboxController2() {
    return xboxController2;
  }

  /**
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

}
