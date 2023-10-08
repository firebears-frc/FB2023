package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
                new WaitCommand(0.5),
                arm.high(),
                schlucker.eject(),
                new WaitCommand(0.5),
                chassis.driveDistance(-0.5),
                schlucker.stop(),
                arm.stow());
    }
}
