package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.chassis.Chassis;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Schlucker;
import frc.robot.util.GamePiece;

public class TwoElementWithMobility extends SequentialCommandGroup {
    public TwoElementWithMobility(Chassis chassis, Arm arm, Schlucker schlucker, GamePiece gamePiece) {
        addCommands(
                new AutoPlaceOneElementCommand(chassis, arm, schlucker, gamePiece),
                chassis.driveDistance(-4.0));
    }
}
