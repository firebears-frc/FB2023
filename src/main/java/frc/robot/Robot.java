package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
    private RobotContainer robotContainer;
    private Command autoCommand = null;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
    }

    @Override
    public void autonomousInit() {
        if (autoCommand != null)
            autoCommand.cancel();

        autoCommand = robotContainer.getAutonomousCommand();
        if (autoCommand != null)
            autoCommand.schedule();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
