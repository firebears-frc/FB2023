package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Schlucker;
import frc.robot.util.GamePiece;

public class AutoPlaceOneElementCommand extends SequentialCommandGroup {
    public AutoPlaceOneElementCommand(Chassis chassis, Arm arm, Schlucker schlucker, GamePiece gamePiece) {
        addCommands(
                switch (gamePiece) {
                    case CONE -> schlucker.intakeCone();
                    case CUBE, NONE -> schlucker.intakeCube();
                },
                schlucker.hold(),
                arm.ready(),
                arm.high(),
                schlucker.eject());
    }
}
