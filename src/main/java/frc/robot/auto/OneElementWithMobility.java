package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Schlucker;
import frc.robot.util.GamePiece;

public class OneElementWithMobility extends SequentialCommandGroup {
    private final GamePiece gamePiece;

    public OneElementWithMobility(Chassis chassis, Arm arm, Schlucker schlucker, GamePiece gamePiece) {
        this.gamePiece = gamePiece;
        addCommands(
                new AutoPlaceOneElementCommand(chassis, arm, schlucker, gamePiece),
                chassis.driveDistance(-4.0));
    }

    @Override
    public String getName() {
        return "OneElementWithMobility<" + gamePiece.name() + ">";
    }
}
