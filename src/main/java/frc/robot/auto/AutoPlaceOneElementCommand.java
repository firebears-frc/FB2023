package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.arm.Arm;
import frc.robot.arm.ArmHighCommand;
import frc.robot.arm.ArmReadyCommand;
import frc.robot.arm.ArmStowCommand;
import frc.robot.chassis.Chassis;
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
                new ArmReadyCommand(arm),
                new WaitCommand(0.5),
                new ArmHighCommand(arm),
                schlucker.eject(),
                new WaitCommand(0.5),
                new AutoDriveCommand(chassis, -0.5),
                schlucker.stop(),
                new ArmStowCommand(arm));
    }
}
