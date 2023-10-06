package frc.robot;

import frc.robot.auto.OneElementWithMobility;
import frc.robot.auto.OneElementWithMobilityAndEngaged;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Schlucker;
import frc.robot.subsystems.SchluckerBag;
import frc.robot.subsystems.SchluckerNeo550;
import frc.robot.subsystems.Vision;
import frc.robot.util.GamePiece;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    public class Constants {
        public static final int JOYSTICK_1_PORT = 0;
        public static final int JOYSTICK_2_PORT = 1;
        public static final int CONTROLLER_PORT = 2;

        public static final double JOYSTICK_DEADBAND = 0.05;

        public static final int PDH_CAN_ID = 1;
    }

    private final Chassis chassis;
    private final Arm arm;
    private final Schlucker schlucker;
    private final Vision vision;
    private final Lights lights;

    private final PowerDistribution pdh;
    private final CommandJoystick joystick_1;
    private final CommandJoystick joystick_2;
    private final CommandXboxController controller;
    private final LoggedDashboardChooser<Command> autoSelector;

    public RobotContainer() {
        chassis = new Chassis();
        arm = new Arm();
        schlucker = new SchluckerBag(); // new SchluckerNeo550();
        vision = new Vision(chassis::visionPose);
        lights = new Lights(schlucker::getHeldItem, schlucker::getWantedItem, chassis::isLevel,
                chassis::isOnChargeStation, chassis::isNotPitching);
        pdh = new PowerDistribution(Constants.PDH_CAN_ID, ModuleType.kRev);

        joystick_1 = new CommandJoystick(Constants.JOYSTICK_1_PORT);
        joystick_2 = new CommandJoystick(Constants.JOYSTICK_2_PORT);
        controller = new CommandXboxController(Constants.CONTROLLER_PORT);

        autoSelector = new LoggedDashboardChooser<>("Auto Routine");
        autoSelector.addDefaultOption("1 Cone w/ Mobility & Engage",
                new OneElementWithMobilityAndEngaged(chassis, arm, schlucker, GamePiece.CONE));
        autoSelector.addOption("1 Cube w/ Mobility & Engage",
                new OneElementWithMobilityAndEngaged(chassis, arm, schlucker, GamePiece.CUBE));
        autoSelector.addOption("1 Cone w/ Mobility",
                new OneElementWithMobility(chassis, arm, schlucker, GamePiece.CONE));
        autoSelector.addOption("1 Cube w/ Mobility",
                new OneElementWithMobility(chassis, arm, schlucker, GamePiece.CUBE));
        autoSelector.addOption("Test Auto Path", chassis.driveTrajectory(
                new Pose2d(),
                List.of(
                        new Translation2d(1, 1),
                        new Translation2d(2, -1)),
                new Pose2d(3, 0, new Rotation2d()),
                false));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        chassis.setDefaultCommand(new Chassis.DriveCommand(
                chassis,
                () -> new ChassisSpeeds(
                        -MathUtil.applyDeadband(joystick_1.getY(), Constants.JOYSTICK_DEADBAND),
                        -MathUtil.applyDeadband(joystick_1.getX(), Constants.JOYSTICK_DEADBAND),
                        -MathUtil.applyDeadband(joystick_2.getX(), Constants.JOYSTICK_DEADBAND)),
                () -> joystick_1.getHID().getRawButton(2),
                true,
                true));

        arm.setDefaultCommand(arm.defaultCommand(controller::getLeftY, controller::getRightY));

        // Arm target point commands
        controller.povUp().onTrue(arm.substation());
        controller.povRight()
                .onTrue(arm.mid().andThen(schlucker.eject()))
                .onFalse(schlucker.stop());
        controller.povDown().onTrue(arm.groundCone());
        controller.povLeft()
                .onTrue(arm.high().andThen(schlucker.eject()))
                .onFalse(schlucker.stop());
        controller.leftBumper().onTrue(arm.groundCube());
        controller.b().onTrue(arm.stow());

        // Schlucker commands
        controller.a()
                .onTrue(schlucker.intakeCone())
                .onFalse(schlucker.hold());
        controller.x()
                .onTrue(schlucker.intakeCube())
                .onFalse(schlucker.hold());
        controller.y()
                .onTrue(schlucker.eject())
                .onFalse(schlucker.stop());

        joystick_1.button(3).onTrue(schlucker.wantCone());
        joystick_1.button(4).onTrue(schlucker.wantCube());
    }

    public Command getAutonomousCommand() {
        return autoSelector.get();
    }
}
