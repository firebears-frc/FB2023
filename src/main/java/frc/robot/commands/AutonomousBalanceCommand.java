package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutonomousBalanceCommand extends CommandBase {
    DriveSubsystem m_chassis;
    Timer timer;
    double lastPitch;
    double speed;
    Timer timer2;
    boolean moveForward;

    /** Creates a new ChassisAutoBalanceCommand. */
    public AutonomousBalanceCommand(DriveSubsystem chassis) {
        m_chassis = chassis;
        timer = new Timer();
        timer2 = new Timer();

        addRequirements(m_chassis);
    }

    @Override
    public void initialize() {
        lastPitch = m_chassis.getPitch();
        timer2.reset();
        timer2.start();
        speed = 0.07;
        moveForward = true;
    }

    @Override
    public void execute() {
        double pitchVelocity = 0; // m_chassis.getpitchVelocity();
        double pitchSpeed = Math.abs(pitchVelocity);

        SmartDashboard.putNumber("Speed", speed);
        SmartDashboard.putNumber("Timer2", timer2.get());
        SmartDashboard.putNumber(
                "CounterRot",
                pitchVelocity * Math.signum(m_chassis.getPitch()));
        if (pitchSpeed >= 0.3 && timer2.hasElapsed(1)) {
            speed = -0.05;
            moveForward = false;
        }

        if (m_chassis.getPitch() >= 2) {
            m_chassis.arcadeDrive(speed, 0);
            timer.reset();
        } else if (m_chassis.getPitch() <= -2) {
            m_chassis.arcadeDrive(-speed, 0);
            timer.reset();
        } else {
            timer.start();
            if (moveForward == false) {
                speed = 0;
            }
        }

    }

    @Override
    public void end(boolean interrupted) {
        m_chassis.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(2);
    }
}