package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Schlucker;
import frc.robot.util.GamePiece;
import java.util.List;

public class Autos {
    private static class Constants {
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

        // Speeds
        public static final double BALANCE_ON_CHARGE_STATION_SPEED = 0.375; // meters per second
        public static final double DRIVE_ONTO_CHARGE_STATION_SPEED = 1; // meters per second

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

    public Autos(Chassis chassis, Arm arm, Schlucker schlucker) {
        autoSelector = new LoggedDashboardChooser<>("Auto Routine");
        autoSelector.addDefaultOption("1 Cone & Engage",
                oneElementWithMobilityAndBalance(chassis, arm, schlucker, GamePiece.CONE));
        autoSelector.addOption("1 Cube & Engage",
                oneElementWithMobilityAndBalance(chassis, arm, schlucker, GamePiece.CUBE));
        autoSelector.addOption("2 Open", twoElementWithMobilityOpenSide(chassis, arm, schlucker));
        autoSelector.addOption("2 Cable", twoElementWithMobilityCableSide(chassis, arm, schlucker));
        autoSelector.addOption("1 Cone", oneElementWithMobility(chassis, arm, schlucker, GamePiece.CONE));
        autoSelector.addOption("1 Cube", oneElementWithMobility(chassis, arm, schlucker, GamePiece.CUBE));
    }

    public Command get() {
        return autoSelector.get();
    }

    private Command balanceOnChargeStation(Chassis chassis) {
        return new FunctionalCommand(
                () -> chassis.drive(new ChassisSpeeds(Constants.BALANCE_ON_CHARGE_STATION_SPEED, 0, 0), false),
                () -> {
                    if (!chassis.isNotPitching()) {
                        // Charge station is moving, stop!
                        chassis.setX();
                        return;
                    }

                    // Depending on what way the charge station is tipped, go to middle
                    if (chassis.getPitchDegrees() > 0) {
                        chassis.drive(new ChassisSpeeds(Constants.BALANCE_ON_CHARGE_STATION_SPEED, 0, 0), false);
                    } else {
                        chassis.drive(new ChassisSpeeds(-1 * Constants.BALANCE_ON_CHARGE_STATION_SPEED, 0, 0), false);
                    }
                },
                null,
                () -> chassis.isNotPitching() && chassis.isLevel(),
                chassis);
    }

    private Command driveOntoChargeStation(Chassis chassis) {
        return new FunctionalCommand(
                () -> chassis.drive(new ChassisSpeeds(Constants.DRIVE_ONTO_CHARGE_STATION_SPEED, 0, 0), false),
                null,
                null,
                chassis::isOnChargeStation,
                chassis);
    }

    private Command autoBalance(Chassis chassis) {
        return new SequentialCommandGroup(
                driveOntoChargeStation(chassis),
                balanceOnChargeStation(chassis),
                new WaitUntilCommand(DriverStation::isDisabled));
    }

    private Command placeElement(Arm arm, Schlucker schlucker, GamePiece gamePiece) {
        return new SequentialCommandGroup(
                switch (gamePiece) {
                    case CONE -> schlucker.intakeCone();
                    case CUBE, NONE -> schlucker.intakeCube();
                },
                schlucker.hold(),
                arm.ready(),
                arm.high(),
                schlucker.eject());
    }

    private Command stopSchluckerAndStowWhile(Command command, Arm arm, Schlucker schlucker, double delay) {
        return new ParallelCommandGroup(
                command,
                new SequentialCommandGroup(
                        new WaitCommand(delay),
                        schlucker.stop(),
                        arm.stow()));
    }

    private Command oneElementWithMobility(Chassis chassis, Arm arm, Schlucker schlucker, GamePiece gamePiece) {
        Command result = new SequentialCommandGroup(
                new InstantCommand(() -> chassis.setPose(Constants.Mobility.START_POSE), chassis),

                placeElement(arm, schlucker, gamePiece),

                stopSchluckerAndStowWhile(
                        chassis.driveTrajectory(
                                Constants.Mobility.START_POSE,
                                Constants.Mobility.END_POSE,
                                false),
                        arm, schlucker, 0.25));
        result.setName("OneElementWithMobility<" + gamePiece.name() + ">");
        return result;
    }

    private Command oneElementWithMobilityAndBalance(Chassis chassis, Arm arm, Schlucker schlucker,
            GamePiece gamePiece) {
        Command result = new SequentialCommandGroup(
                new InstantCommand(() -> chassis.setPose(Constants.Balance.START_POSE), chassis),

                placeElement(arm, schlucker, gamePiece),

                stopSchluckerAndStowWhile(
                        chassis.driveTrajectory(
                                Constants.Balance.START_POSE,
                                Constants.Balance.MIDDLE_POSE,
                                false),
                        arm, schlucker, 0.25),

                chassis.driveTrajectory(
                        Constants.Balance.MIDDLE_POSE,
                        Constants.Balance.END_POSE,
                        false),

                autoBalance(chassis));
        result.setName("OneElementWithMobilityAndBalance<" + gamePiece.name() + ">");
        return result;
    }

    private Command twoElementWithMobilityOpenSide(Chassis chassis, Arm arm, Schlucker schlucker) {
        Command result = new SequentialCommandGroup(
                new InstantCommand(() -> chassis.setPose(Constants.Open.START_POSE), chassis),

                placeElement(arm, schlucker, GamePiece.CONE),

                stopSchluckerAndStowWhile(
                        chassis.driveTrajectory(
                                Constants.Open.START_POSE,
                                List.of(Constants.Open.MIDDLE_TRANSLATION),
                                Constants.Open.GAME_PIECE_POSE,
                                false),
                        arm, schlucker, 0.25),

                arm.groundCube(),
                schlucker.intakeCube(),

                stopSchluckerAndStowWhile(
                        chassis.driveTrajectory(
                                Constants.Open.GAME_PIECE_POSE,
                                List.of(Constants.Open.MIDDLE_TRANSLATION),
                                Constants.Open.END_POSE,
                                false),
                        arm, schlucker, 2),

                placeElement(arm, schlucker, GamePiece.CUBE));

        result.setName("TwoElementWithMobilityOpen");
        return result;
    }

    private Command twoElementWithMobilityCableSide(Chassis chassis, Arm arm, Schlucker schlucker) {
        Command result = new SequentialCommandGroup(
                placeElement(arm, schlucker, GamePiece.CONE),

                stopSchluckerAndStowWhile(
                        chassis.driveTrajectory(Constants.Cable.START_POSE, Constants.Cable.FIRST_POSE, false),
                        arm, schlucker, 0.25),
                chassis.driveTrajectory(Constants.Cable.FIRST_POSE, Constants.Cable.SECOND_POSE, false),
                chassis.driveTrajectory(Constants.Cable.SECOND_POSE, Constants.Cable.GAME_PIECE_POSE, false),

                arm.groundCube(),
                schlucker.intakeCube(),

                stopSchluckerAndStowWhile(
                        chassis.driveTrajectory(Constants.Cable.GAME_PIECE_POSE, Constants.Cable.SECOND_POSE, false),
                        arm, schlucker, 2),
                chassis.driveTrajectory(Constants.Cable.SECOND_POSE, Constants.Cable.FIRST_POSE, false),
                chassis.driveTrajectory(Constants.Cable.FIRST_POSE, Constants.Cable.END_POSE, false),

                placeElement(arm, schlucker, GamePiece.CUBE));

        result.setName("TwoElementWithMobilityCable");
        return result;
    }
}
