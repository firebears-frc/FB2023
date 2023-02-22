package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AutoCommand;

public class Robot extends TimedRobot {
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

        autoCommand = new AutoCommand();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }
}
