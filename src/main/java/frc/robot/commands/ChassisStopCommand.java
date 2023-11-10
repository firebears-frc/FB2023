package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ChassisStopCommand extends CommandBase {
    /** Creates a new ChassisStopCommand. */
    private DriveSubsystem m_chassis;
    private Timer timer;
    private double seconds;

    /**
     * Sets the breaks then waits
     * 
     * @param seconds time in seconds.
     * @param chassis chassis subsystem.
     */
    public ChassisStopCommand(double seconds, DriveSubsystem chassis) {
        m_chassis = chassis;
        timer = new Timer();
        this.seconds = seconds;

        addRequirements(m_chassis);
    }

    @Override
    public void initialize() {
        m_chassis.setBrakemode(true);
        timer.reset();
        timer.start();
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_chassis.arcadeDrive(0, 0);
  }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(seconds);
    }
}
