package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Schlucker;
import frc.robot.util.GamePiece;

public class OneElementWithMobilityAndEngaged extends SequentialCommandGroup {
    private final GamePiece gamePiece;

    public OneElementWithMobilityAndEngaged(Drivetrain drivetrain, Arm arm, Schlucker schlucker, GamePiece gamePiece) {
        this.gamePiece = gamePiece;
        addCommands(
                new AutoPlaceOneElementCommand(drivetrain, arm, schlucker, gamePiece),
                drivetrain.driveDistance(-2.0),
                drivetrain.driveDistance(1.0),
                new AutoBalanceCommand(drivetrain));
    }

    @Override
    public String getName() {
        return "OneElementWithMobilityAndEngaged<" + gamePiece.name() + ">";
    }
}
