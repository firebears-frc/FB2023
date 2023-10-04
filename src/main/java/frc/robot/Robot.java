package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends LoggedRobot {
    private RobotContainer robotContainer;
    private Command autoCommand = null;

    @Override
    public void robotInit() {
        initializeLogging();

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

    private void initializeLogging() {
        Logger logger = Logger.getInstance();
        logger.recordMetadata("Project Name", BuildConstants.MAVEN_NAME);
        logger.recordMetadata("Branch Name", BuildConstants.GIT_BRANCH);
        logger.recordMetadata("Commit Hash (Short)", BuildConstants.GIT_SHA.substring(0, 8));
        logger.recordMetadata("Commit Hash (Full)", BuildConstants.GIT_SHA);
        logger.recordMetadata("Build Time", BuildConstants.BUILD_DATE);

        if (isReal()) {
            // Log to USB & Network Tables
            logger.addDataReceiver(new WPILOGWriter("/media/sda1/"));
            logger.addDataReceiver(new NT4Publisher());
        } else {
            // Replay from log and save to file
            setUseTiming(false);
            String logPath = LogFileUtil.findReplayLog();
            logger.setReplaySource(new WPILOGReader(logPath));
            logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        }

        logger.start();
    }
}
