package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ChassisDriveToDistanceCommand extends CommandBase {
    /** Creates a new DriveToPositionCommand. */
    private double distance;
    private DriveSubsystem m_chassis;
    private double speed = 0.6;

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
        m_chassis.resetOdometry(new Pose2d());
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

    @Override
    public boolean isFinished() {
        // return Math.abs(distance - m_chassis.getEncoderDistance()) < 0.1;

        if (distance > 0 && m_chassis.getPose().getX() > distance) {
            return true;
        } else if (distance < 0 && m_chassis.getPose().getX() < distance) {
            return true;
        }
        return false;
    }
}
