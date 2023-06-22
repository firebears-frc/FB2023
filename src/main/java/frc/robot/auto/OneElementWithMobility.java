package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Schlucker;
import frc.robot.util.GamePiece;

public class OneElementWithMobility extends SequentialCommandGroup {
    private final GamePiece gamePiece;

    public OneElementWithMobility(Drivetrain drivetrain, Arm arm, Schlucker schlucker, GamePiece gamePiece) {
        this.gamePiece = gamePiece;
        addCommands(
                new AutoPlaceOneElementCommand(drivetrain, arm, schlucker, gamePiece),
                drivetrain.driveDistance(-4.0));
    }

    @Override
    public String getName() {
        return "OneElementWithMobility<" + gamePiece.name() + ">";
    }
}
