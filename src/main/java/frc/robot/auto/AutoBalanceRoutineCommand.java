package frc.robot.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Drivetrain;

public class AutoBalanceRoutineCommand extends SequentialCommandGroup {
    public AutoBalanceRoutineCommand(Drivetrain drivetrain) {
        addCommands(
                new AutoDriveOntoChargeStationCommand(drivetrain),
                new AutoBalanceCommand(drivetrain),
                new WaitUntilCommand(DriverStation::isDisabled));
    }
}
