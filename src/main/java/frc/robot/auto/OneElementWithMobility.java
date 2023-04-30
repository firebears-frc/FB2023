package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.chassis.Chassis;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Schlucker;
import frc.robot.util.GamePiece;

public class OneElementWithMobility extends SequentialCommandGroup {
    public OneElementWithMobility(Chassis chassis, Arm arm, Schlucker schlucker, GamePiece gamePiece) {
        addCommands(
                new AutoPlaceOneElementCommand(chassis, arm, schlucker, gamePiece),
                new AutoDriveCommand(chassis, -4.0));
    }
}
