package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
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
                new InstantCommand(cone ? schlucker::intakeCone : schlucker::intakeCube, schlucker),
                new InstantCommand(schlucker::hold, schlucker),
                new ArmReadyCommand(arm),
                new WaitCommand(0.5),
                new ArmHighCommand(arm),
                new InstantCommand(schlucker::eject, schlucker),
                new WaitCommand(0.5),
                new AutoDriveCommand(chassis, -0.5),
                new InstantCommand(schlucker::stop, schlucker),
                new ArmStowCommand(arm));
    }
}
