package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Schlucker;
import frc.robot.util.GamePiece;
import java.util.List;

public class TwoElementCleanWithMobility extends SequentialCommandGroup {
    private static class Constants {
        public static final Pose2d GAME_PIECE_POSE = new Pose2d(-5, 0.5, Rotation2d.fromDegrees(180));
        public static final Translation2d MIDDLE_TRANSLATION = new Translation2d(-4, 0);
    }

    private final GamePiece gamePiece;

    public TwoElementCleanWithMobility(Chassis chassis, Arm arm, Schlucker schlucker, GamePiece gamePiece) {
        this.gamePiece = gamePiece;
        addCommands(
                // Place the element
                new AutoPlaceOneElementCommand(chassis, arm, schlucker, gamePiece),

                // Drive backwards while stowing the arm and stopping the schlucker
                chassis.driveTrajectory(
                        new Pose2d(),
                        List.of(Constants.MIDDLE_TRANSLATION),
                        Constants.GAME_PIECE_POSE,
                        false).alongWith(schlucker.stop(), arm.stow()),

                // Start running the schlucker
                switch (gamePiece) {
                    case CONE -> arm.groundCone();
                    case CUBE, NONE -> arm.groundCube();
                },
                switch (gamePiece) {
                    case CONE -> schlucker.intakeCone();
                    case CUBE, NONE -> schlucker.intakeCube();
                },

                // Drive backwards while intaking then retracting the arm
                chassis.driveTrajectory(
                        Constants.GAME_PIECE_POSE,
                        List.of(Constants.MIDDLE_TRANSLATION),
                        new Pose2d(),
                        false).alongWith(new WaitCommand(0.5).andThen(arm.stow().andThen(schlucker.hold()))),

                // Place the element then stop the sclucker and stow the arm
                new AutoPlaceOneElementCommand(chassis, arm, schlucker, gamePiece),
                chassis.driveDistance(-0.5).alongWith(schlucker.stop(), arm.stow()));
    }

    @Override
    public String getName() {
        return "TwoElementCleanWithMobility<" + gamePiece.name() + ">";
    }
}
