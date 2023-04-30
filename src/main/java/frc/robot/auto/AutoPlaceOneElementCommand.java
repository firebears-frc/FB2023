package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Schlucker;
import frc.robot.util.GamePiece;

public class AutoPlaceOneElementCommand extends SequentialCommandGroup {
    public AutoPlaceOneElementCommand(Chassis chassis, Arm arm, Schlucker schlucker, GamePiece gamePiece) {
        boolean cone;
        switch (gamePiece) {
            case CONE:
                cone = true;
                break;
            case CUBE:
            case NONE:
            default:
                cone = false;
                break;
        }

        addCommands(
                (cone ? schlucker.intakeCone() : schlucker.intakeCube()),
                schlucker.hold(),
                arm.ready(),
                new WaitCommand(0.5),
                arm.high(),
                schlucker.eject(),
                new WaitCommand(0.5),
                chassis.driveDistance(-0.5),
                schlucker.stop(),
                arm.stow());
    }
}
