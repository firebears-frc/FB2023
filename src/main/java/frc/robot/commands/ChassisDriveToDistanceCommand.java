package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class ChassisDriveToDistanceCommand extends Command {
    /** Creates a new DriveToPositionCommand. */
    private double distance;
    private DriveSubsystem m_chassis;
    private double speed = 0.6;
  private double start_x;

    public ChassisDriveToDistanceCommand(double d, DriveSubsystem c) {
        distance = d;
        m_chassis = c;

        addRequirements(m_chassis);
    }

    public ChassisDriveToDistanceCommand(double d, double s, DriveSubsystem c) {
        distance = d;
        m_chassis = c;
        speed = s;

        addRequirements(m_chassis);
    }

    @Override
    public void initialize() {
        start_x = m_chassis.getPose().getX();
    }

    @Override
    public void execute() {
        if (distance > 0) {
            m_chassis.arcadeDrive(speed, 0);
        } else {
            m_chassis.arcadeDrive(-1 * speed, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_chassis.arcadeDrive(0, 0);
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

        if (distance > 0 && (start_x - m_chassis.getPose().getX()) > distance) {
            return true;
        } else if (distance < 0 && (start_x - m_chassis.getPose().getX()) < distance) {
            return true;
        }
        return false;
    }
}
