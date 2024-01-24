package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class ChassisDriveToPitch extends Command {
    /** Creates a new DriveToPositionCommand. */
    private double pitch;
    private double speed;
    private DriveSubsystem m_chassis;

    public ChassisDriveToPitch(double p, double s, DriveSubsystem c) {
        pitch = p;
        speed = s;
        m_chassis = c;

        addRequirements(m_chassis);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_chassis.arcadeDrive(speed, 0);
    }

    @Override
    public void end(boolean interrupted) {
        m_chassis.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return m_chassis.getPitch() >= pitch;
    }
}