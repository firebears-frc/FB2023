package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
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
        public static final double BALANCE_ON_CHARGE_STATION_SPEED = 0.375; // meters per second
        public static final double DRIVE_ONTO_CHARGE_STATION_SPEED = 1.0; // meters per second

        public static final Pose2d MOBILITY_FINAL_POSE = new Pose2d(-3.0, 0, Rotation2d.fromDegrees(-180.0));

        public static final Pose2d BALANCE_MIDDLE_POSE = new Pose2d(-2.0, 0, new Rotation2d());
        public static final Pose2d BALANCE_FINAL_POSE = new Pose2d(-1.0, 0, new Rotation2d());

        public static final Pose2d OPEN_GAME_PIECE_POSE = new Pose2d(-5.0, 0.25, Rotation2d.fromDegrees(150));
        public static final Translation2d OPEN_MIDDLE_TRANSLATION = new Translation2d(-3.0, 0);

        public static final Pose2d CABLE_FIRST_POSE = new Pose2d(-1.0, 0, Rotation2d.fromDegrees(30));
        public static final Pose2d CABLE_SECOND_POSE = new Pose2d(-2.0, 0, Rotation2d.fromDegrees(60));
        public static final Translation2d CABLE_MIDDLE_TRANSLATION = new Translation2d(-3.0, 0);
        public static final Pose2d CABLE_GAME_PIECE_POSE = new Pose2d(-5.0, -0.25, Rotation2d.fromDegrees(150));
        public static final Pose2d CABLE_END_POSE = new Pose2d(0, -0.5, new Rotation2d());
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
                        chassis.drive(new ChassisSpeeds(-1.0 * Constants.BALANCE_ON_CHARGE_STATION_SPEED, 0, 0), false);
                    }
                },
                null,
                () -> chassis.isNotPitching() && chassis.isLevel(),
                chassis);
    }

    private Command driveOntoChargeStation(Chassis chassis) {
        return new FunctionalCommand(
                () -> chassis.drive(new ChassisSpeeds(Constants.DRIVE_ONTO_CHARGE_STATION_SPEED, 0.0, 0.0), false),
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
                placeElement(arm, schlucker, gamePiece),

                stopSchluckerAndStowWhile(
                        chassis.driveTrajectory(
                                new Pose2d(),
                                Constants.MOBILITY_FINAL_POSE,
                                false),
                        arm, schlucker, 0.25));
        result.setName("OneElementWithMobility<" + gamePiece.name() + ">");
        return result;
    }

    private Command oneElementWithMobilityAndBalance(Chassis chassis, Arm arm, Schlucker schlucker,
            GamePiece gamePiece) {
        Command result = new SequentialCommandGroup(
                placeElement(arm, schlucker, gamePiece),

                stopSchluckerAndStowWhile(
                        chassis.driveTrajectory(
                                new Pose2d(),
                                Constants.BALANCE_MIDDLE_POSE,
                                false),
                        arm, schlucker, 0.25),

                chassis.driveTrajectory(
                        Constants.BALANCE_MIDDLE_POSE,
                        Constants.BALANCE_FINAL_POSE,
                        false),
                autoBalance(chassis));
        result.setName("OneElementWithMobilityAndBalance<" + gamePiece.name() + ">");
        return result;
    }

    private Command twoElementWithMobilityOpenSide(Chassis chassis, Arm arm, Schlucker schlucker) {
        Command result = new SequentialCommandGroup(
                placeElement(arm, schlucker, GamePiece.CONE),

                stopSchluckerAndStowWhile(
                        chassis.driveTrajectory(new Pose2d(), List.of(Constants.OPEN_MIDDLE_TRANSLATION),
                                Constants.OPEN_GAME_PIECE_POSE, false),
                        arm, schlucker, 0.25),

                arm.groundCube(),
                schlucker.intakeCube(),

                stopSchluckerAndStowWhile(
                        chassis.driveTrajectory(Constants.OPEN_GAME_PIECE_POSE,
                                List.of(Constants.OPEN_MIDDLE_TRANSLATION), new Pose2d(), false),
                        arm, schlucker, 2.0),

                placeElement(arm, schlucker, GamePiece.CUBE));

        result.setName("TwoElementWithMobilityOpen");
        return result;
    }

    private Command twoElementWithMobilityCableSide(Chassis chassis, Arm arm, Schlucker schlucker) {
        Command result = new SequentialCommandGroup(
                placeElement(arm, schlucker, GamePiece.CONE),

                stopSchluckerAndStowWhile(
                        chassis.driveTrajectory(new Pose2d(), Constants.CABLE_FIRST_POSE, false),
                        arm, schlucker, 0.25),
                chassis.driveTrajectory(Constants.CABLE_FIRST_POSE, Constants.CABLE_SECOND_POSE, false),
                chassis.driveTrajectory(Constants.CABLE_SECOND_POSE, List.of(Constants.CABLE_MIDDLE_TRANSLATION),
                        Constants.CABLE_GAME_PIECE_POSE, false),

                arm.groundCube(),
                schlucker.intakeCube(),

                stopSchluckerAndStowWhile(
                        chassis.driveTrajectory(Constants.CABLE_GAME_PIECE_POSE,
                                List.of(Constants.CABLE_MIDDLE_TRANSLATION), Constants.CABLE_SECOND_POSE, false),
                        arm, schlucker, 2.0),
                chassis.driveTrajectory(Constants.CABLE_SECOND_POSE, Constants.CABLE_FIRST_POSE, false),
                chassis.driveTrajectory(Constants.CABLE_FIRST_POSE, Constants.CABLE_END_POSE, false),

                placeElement(arm, schlucker, GamePiece.CUBE));

        result.setName("TwoElementWithMobilityCable");
        return result;
    }
}
