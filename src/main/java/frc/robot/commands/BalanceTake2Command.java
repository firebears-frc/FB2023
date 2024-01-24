package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceTake2Command extends Command {

    DriveSubsystem m_chassis;
    double speed;
    double lastPitch;

    /** Creates a new BalanceTake2Command. */
    public BalanceTake2Command(DriveSubsystem chassis) {
        m_chassis = chassis;
        speed = 0.07;

        addRequirements(m_chassis);
    }

    public BalanceTake2Command(double s, DriveSubsystem chassis) {
        m_chassis = chassis;
        speed = s;

        addRequirements(m_chassis);
    }

    @Override
    public void initialize() {
        lastPitch = m_chassis.getPitch();
        m_chassis.setBrakemode(true);
        m_chassis.arcadeDrive(speed, 0);
    }

    @Override
    public void execute() {
        if (Math.abs(m_chassis.getpitchVelocity()) > 0.15) {
            m_chassis.arcadeDrive(0, 0);
        } else {
            if (m_chassis.getPitch() > 0) {
                m_chassis.arcadeDrive(speed, 0);
            } else {
                m_chassis.arcadeDrive(-speed, 0);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_chassis.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(m_chassis.getPitch()) < 3 && Math.abs(m_chassis.getpitchVelocity()) < 0.15;
        // m_chassis.getPitch() < 5 || m_chassis.getpitchVelocity() < -0.3;
        // m_chassis.getpitchVelocity() <= -0.5;
    }
}
