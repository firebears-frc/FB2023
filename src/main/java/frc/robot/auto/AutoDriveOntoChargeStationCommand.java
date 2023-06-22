package frc.robot.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Drivetrain;

public class AutoDriveOntoChargeStationCommand extends SequentialCommandGroup {
    public AutoDriveOntoChargeStationCommand(Drivetrain drivetrain) {
        addCommands(
                new FunctionalCommand(
                        () -> drivetrain.drive(new ChassisSpeeds(
                                1.0,
                                0.0,
                                0.0)),
                        null,
                        null,
                        drivetrain::isOnChargeStation,
                        drivetrain),
                new WaitCommand(0.5),
                new WaitUntilCommand(drivetrain::isOnChargeStation));
    }
}
