
package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

/** Default command to drive the Chassis. */
public class ArmShoulderDriveCommand extends CommandBase {
  private Chassis m_chassis;
  private XboxController xbox;

  /** Default command to drive the Chassis. */
  public ArmShoulderDriveCommand(Chassis c, XboxController x) {
    m_chassis = c;
    xbox = x;
    addRequirements(m_chassis);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
