package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalanceRoutine extends SequentialCommandGroup {
    /** Creates a new AutoBalanceRoutine. */
    private DriveSubsystem m_chassis;

    public AutoBalanceRoutine(DriveSubsystem c) {
        m_chassis = c;

        addCommands(
                new ChassisSetBrakeMode(true, m_chassis),
                // new ChassisDriveToPitch(10, 0.2, m_chassis),
                new ChassisDriveToPitch(10, 0.2, m_chassis), new PrintCommand("PITCH"),
                new ChassisDriveToDistanceCommand(0.65, 0.3, m_chassis), new PrintCommand("DRIVEN"),
                new BalanceTake2Command(0.07, m_chassis), new PrintCommand("BALANCED"),

                new ChassisStopCommand(3, m_chassis)
        // new ChassisDriveToDistanceCommand(-0.05, 0.15, m_chassis),
        // new WaitCommand(5)
        );
    }
}
