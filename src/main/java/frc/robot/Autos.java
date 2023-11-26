package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.arm.Arm;
import frc.robot.drive.Chassis;
import frc.robot.drive.Drive;
import frc.robot.intake.Intake;
import frc.robot.util.GamePiece;
import java.util.List;

public class Autos {
    private static final class Constants {
        // Robot dimensions
        private static final double BUMPER_SIZE = Units.inchesToMeters(3);
        private static final double ROBOT_OFFSET = ((Chassis.ROBOT_LENGTH / 2) + BUMPER_SIZE);

        // Field dimensions
        private static final double GRID_DEPTH = Units.feetToMeters(4) + Units.inchesToMeters(8.25);
        private static final double CHARGE_STATION_DEPTH = Units.feetToMeters(6) + Units.inchesToMeters(4.125);
        private static final double WALL_TO_FAR_CHARGE_STATION = Units.feetToMeters(16) + Units.inchesToMeters(1.25);
        private static final double WALL_TO_NEAR_CHARGE_STATION = WALL_TO_FAR_CHARGE_STATION - CHARGE_STATION_DEPTH;
        private static final double GRID_TO_ELEMENTS = Units.feetToMeters(18) + Units.inchesToMeters(8);

        // Useful coordinates
        private static final double GRID_X = GRID_DEPTH + ROBOT_OFFSET;
        private static final double CHARGE_STATION_FAR_X = WALL_TO_FAR_CHARGE_STATION + ROBOT_OFFSET;
        private static final double CHARGE_STATION_NEAR_X = WALL_TO_NEAR_CHARGE_STATION - ROBOT_OFFSET;
        private static final double ELEMENTS_X = GRID_X + GRID_TO_ELEMENTS - (ROBOT_OFFSET * 2);

        public static class Mobility {
            private static final Transform2d MOVEMENT = new Transform2d(ELEMENTS_X - GRID_X, 0,
                    Rotation2d.fromDegrees(-135));

            public static final Pose2d START_POSE = new Pose2d(GRID_X, Units.feetToMeters(1.5),
                    Rotation2d.fromDegrees(180));
            public static final Pose2d END_POSE = START_POSE.plus(MOVEMENT);
        }

        public static class Balance {
            private static final double REF_OFFSET = Units.feetToMeters(3);
            private static final Transform2d FIRST_MOVEMENT = new Transform2d(
                    CHARGE_STATION_FAR_X - GRID_X + REF_OFFSET, 0, Rotation2d.fromDegrees(0));
            private static final Transform2d SECOND_MOVEMENT = new Transform2d(-REF_OFFSET, 0,
                    Rotation2d.fromDegrees(0));

            public static final Pose2d START_POSE = new Pose2d(GRID_X, Units.feetToMeters(11),
                    Rotation2d.fromDegrees(180));
            public static final Pose2d MIDDLE_POSE = START_POSE.plus(FIRST_MOVEMENT);
            public static final Pose2d END_POSE = MIDDLE_POSE.plus(SECOND_MOVEMENT);
        }

        public static class Open {
            private static final Transform2d END_OFFSET = new Transform2d(0, -0.5, Rotation2d.fromDegrees(0));

            public static final Pose2d START_POSE = new Pose2d(GRID_X, Units.feetToMeters(16),
                    Rotation2d.fromDegrees(180));
            public static final Translation2d MIDDLE_TRANSLATION = new Translation2d(CHARGE_STATION_FAR_X - GRID_X, 0);
            public static final Pose2d GAME_PIECE_POSE = new Pose2d(ELEMENTS_X, -0.25, Rotation2d.fromDegrees(-45));
            public static final Pose2d END_POSE = START_POSE.plus(END_OFFSET);
        }

        public static class Cable {
            private static final Transform2d FIRST_TRANSFORM = new Transform2d(CHARGE_STATION_NEAR_X - GRID_X, 0,
                    Rotation2d.fromDegrees(-30));
            private static final Transform2d SECOND_TRANSFORM = new Transform2d(
                    CHARGE_STATION_FAR_X - CHARGE_STATION_NEAR_X, 0, Rotation2d.fromDegrees(-30));
            private static final Transform2d GAME_PIECE_TRANSFORM = new Transform2d(ELEMENTS_X - CHARGE_STATION_FAR_X,
                    0.25, Rotation2d.fromDegrees(-75));
            private static final Transform2d END_OFFSET = new Transform2d(0, 0.5, Rotation2d.fromDegrees(0));

            public static final Pose2d START_POSE = new Pose2d(GRID_X, Units.feetToMeters(1.5),
                    Rotation2d.fromDegrees(180));
            public static final Pose2d FIRST_POSE = START_POSE.plus(FIRST_TRANSFORM);
            public static final Pose2d SECOND_POSE = FIRST_POSE.plus(SECOND_TRANSFORM);
            public static final Pose2d GAME_PIECE_POSE = SECOND_POSE.plus(GAME_PIECE_TRANSFORM);
            public static final Pose2d END_POSE = START_POSE.plus(END_OFFSET);
        }
    }

    private final LoggedDashboardChooser<Command> autoSelector;

    public Autos(Drive drive, Arm arm, Intake intake) {
        autoSelector = new LoggedDashboardChooser<>("Auto Routine");
        autoSelector.addDefaultOption("1 Cone & Engage",
                oneElementWithMobilityAndBalance(drive, arm, intake, GamePiece.CONE));
        autoSelector.addOption("1 Cube & Engage", oneElementWithMobilityAndBalance(drive, arm, intake, GamePiece.CUBE));
        autoSelector.addOption("2 Open", twoElementWithMobilityOpenSide(drive, arm, intake));
        autoSelector.addOption("2 Cable", twoElementWithMobilityCableSide(drive, arm, intake));
        autoSelector.addOption("1 Cone", oneElementWithMobility(drive, arm, intake, GamePiece.CONE));
        autoSelector.addOption("1 Cube", oneElementWithMobility(drive, arm, intake, GamePiece.CUBE));
    }

    public Command get() {
        return autoSelector.get();
    }

    protected Command oneElementWithMobility(Drive drive, Arm arm, Intake intake, GamePiece gamePiece) {
        Command result = Commands.sequence(
                drive.setPose(Constants.Mobility.START_POSE),
                switch (gamePiece) {
                    case CONE -> intake.intakeCone();
                    case CUBE, NONE -> intake.intakeCube();
                },
                intake.hold(),

                arm.ready(),
                arm.high(),
                intake.eject(),
                Commands.waitSeconds(0.1),

                Commands.parallel(
                        Commands.sequence(
                                intake.stop(),
                                Commands.waitSeconds(0.25),
                                arm.stow()),
                        drive.driveTrajectory(
                                Constants.Mobility.START_POSE,
                                Constants.Mobility.END_POSE,
                                false)));
        result.setName("OneElementWithMobility<" + gamePiece.name() + ">");
        return result;
    }

    protected Command oneElementWithMobilityAndBalance(Drive drive, Arm arm, Intake intake, GamePiece gamePiece) {
        Command result = Commands.sequence(
                drive.setPose(Constants.Balance.START_POSE),
                switch (gamePiece) {
                    case CONE -> intake.intakeCone();
                    case CUBE, NONE -> intake.intakeCube();
                },
                intake.hold(),

                arm.ready(),
                arm.high(),
                intake.eject(),
                Commands.waitSeconds(0.1),

                Commands.parallel(
                        Commands.sequence(
                                intake.stop(),
                                Commands.waitSeconds(0.25),
                                arm.stow()),
                        Commands.sequence(
                                drive.driveTrajectory(
                                        Constants.Balance.START_POSE,
                                        Constants.Balance.MIDDLE_POSE,
                                        false),
                                drive.driveTrajectory(
                                        Constants.Balance.MIDDLE_POSE,
                                        Constants.Balance.END_POSE,
                                        false))),

                drive.autoBalance(),
                Commands.idle(drive).until(DriverStation::isDisabled));
        result.setName("OneElementWithMobilityAndBalance<" + gamePiece.name() + ">");
        return result;
    }

    protected Command twoElementWithMobilityOpenSide(Drive drive, Arm arm, Intake intake) {
        Command result = Commands.sequence(
                drive.setPose(Constants.Open.START_POSE),
                intake.intakeCone(),
                intake.hold(),

                arm.ready(),
                arm.high(),
                intake.eject(),
                Commands.waitSeconds(0.1),

                Commands.parallel(
                        Commands.sequence(
                                intake.stop(),
                                Commands.waitSeconds(0.25),
                                arm.stow()),
                        drive.driveTrajectory(
                                Constants.Open.START_POSE,
                                List.of(Constants.Open.MIDDLE_TRANSLATION),
                                Constants.Open.GAME_PIECE_POSE,
                                false)),

                intake.intakeCube(),
                arm.groundCube(),

                Commands.parallel(
                        Commands.sequence(
                                Commands.waitSeconds(2.5),
                                intake.stop(),
                                arm.stow()),
                        drive.driveTrajectory(
                                Constants.Open.GAME_PIECE_POSE,
                                List.of(Constants.Open.MIDDLE_TRANSLATION),
                                Constants.Open.END_POSE,
                                false)),

                arm.ready(),
                arm.high(),
                intake.eject());
        result.setName("TwoElementWithMobilityOpen");
        return result;
    }

    protected Command twoElementWithMobilityCableSide(Drive drive, Arm arm, Intake intake) {
        Command result = Commands.sequence(
                drive.setPose(Constants.Open.START_POSE),
                intake.intakeCone(),
                intake.hold(),

                arm.ready(),
                arm.high(),
                intake.eject(),
                Commands.waitSeconds(0.1),

                Commands.parallel(
                        Commands.sequence(
                                intake.stop(),
                                Commands.waitSeconds(0.25),
                                arm.stow()),
                        Commands.sequence(
                                drive.driveTrajectory(Constants.Cable.START_POSE, Constants.Cable.FIRST_POSE, false),
                                drive.driveTrajectory(Constants.Cable.FIRST_POSE, Constants.Cable.SECOND_POSE, false),
                                drive.driveTrajectory(Constants.Cable.SECOND_POSE, Constants.Cable.GAME_PIECE_POSE,
                                        false))),

                intake.intakeCube(),
                arm.groundCube(),

                Commands.parallel(
                        Commands.sequence(
                                Commands.waitSeconds(2.5),
                                intake.stop(),
                                arm.stow()),
                        Commands.sequence(
                                drive.driveTrajectory(Constants.Cable.GAME_PIECE_POSE, Constants.Cable.SECOND_POSE,
                                        false),
                                drive.driveTrajectory(Constants.Cable.SECOND_POSE, Constants.Cable.FIRST_POSE, false),
                                drive.driveTrajectory(Constants.Cable.FIRST_POSE, Constants.Cable.END_POSE, false))),

                arm.ready(),
                arm.high(),
                intake.eject());

        result.setName("TwoElementWithMobilityCable");
        return result;
    }
}
