package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Schlucker;
import frc.robot.util.GamePiece;

public class OneElementWithMobilityAndEngaged extends SequentialCommandGroup {
    public OneElementWithMobilityAndEngaged(Chassis chassis, Arm arm, Schlucker schlucker, GamePiece gamePiece) {
        addCommands(
                new AutoPlaceOneElementCommand(chassis, arm, schlucker, gamePiece),
                chassis.driveDistance(-2.0),
                chassis.driveDistance(1.0),
                new AutoBalanceCommand(chassis));
    }
}
