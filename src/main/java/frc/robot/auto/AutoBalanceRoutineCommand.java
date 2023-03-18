package frc.robot.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.chassis.Chassis;

public class AutoBalanceRoutineCommand extends SequentialCommandGroup {
    public AutoBalanceRoutineCommand(Chassis chassis) {
        addCommands(
                new AutoDriveOntoChargeStationCommand(chassis),
                new AutoBalanceCommand(chassis),
                new WaitUntilCommand(DriverStation::isDisabled));
    }
}
