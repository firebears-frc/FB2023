package frc.robot.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.chassis.Chassis;

public class AutoDriveOntoChargeStationCommand extends SequentialCommandGroup {
    public AutoDriveOntoChargeStationCommand(Chassis chassis) {
        addCommands(
                new FunctionalCommand(
                        () -> chassis.drive(new ChassisSpeeds(
                                1.0,
                                0.0,
                                0.0)),
                        null,
                        null,
                        chassis::isOnChargeStation,
                        chassis),
                new WaitCommand(0.5),
                new WaitUntilCommand(chassis::isOnChargeStation));
    }
}
