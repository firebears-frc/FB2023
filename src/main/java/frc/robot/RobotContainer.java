package frc.robot;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.Trajectories;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeBag;
import frc.robot.subsystems.IntakeNeo550;
import frc.robot.subsystems.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

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
    private final Localization localization;
    private final Trajectories trajectories;
    private final Arm arm;
    private final Intake intake;
    private final Vision vision;
    private final Lights lights;

    private final PowerDistribution pdh;
    private final CommandJoystick one;
    private final CommandJoystick two;
    private final CommandXboxController controller;
    private final Autos autos;

    public RobotContainer() {
        chassis = new Chassis();
        localization = chassis.getLocalization();
        trajectories = chassis.getTrajectories();
        arm = new Arm();
        intake = new IntakeBag(); // new IntakeNeo550();
        vision = new Vision(localization::visionPose);
        lights = new Lights(intake::getHeldItem, intake::getWantedItem, localization::isLevel,
                localization::isOnChargeStation, localization::isNotPitching);
        pdh = new PowerDistribution(Constants.PDH_CAN_ID, ModuleType.kRev);

        one = new CommandJoystick(Constants.JOYSTICK_1_PORT);
        two = new CommandJoystick(Constants.JOYSTICK_2_PORT);
        controller = new CommandXboxController(Constants.CONTROLLER_PORT);

        autos = new Autos(chassis, localization, trajectories, arm, intake);

        configureButtonBindings();
    }

    private ChassisSpeeds getChassisSpeeds() {
        return new ChassisSpeeds(
                -MathUtil.applyDeadband(one.getY(), Constants.JOYSTICK_DEADBAND),
                -MathUtil.applyDeadband(one.getX(), Constants.JOYSTICK_DEADBAND),
                -MathUtil.applyDeadband(two.getX(), Constants.JOYSTICK_DEADBAND));
    }

    private void configureButtonBindings() {
        // Arm commands
        arm.setDefaultCommand(arm.defaultCommand(controller::getLeftY, controller::getRightY));
        controller.povUp().onTrue(arm.substation());
        controller.povRight()
                .onTrue(arm.mid().andThen(intake.eject()))
                .onFalse(intake.stop());
        controller.povDown().onTrue(arm.groundCone());
        controller.povLeft()
                .onTrue(arm.high().andThen(intake.eject()))
                .onFalse(intake.stop());
        controller.leftBumper().onTrue(arm.groundCube());
        controller.b().onTrue(arm.stow());

        // Chassis commands
        chassis.setDefaultCommand(chassis.defaultCommand(
                this::getChassisSpeeds,
                false,
                true,
                true));
        one.button(2).toggleOnTrue(chassis.defaultCommand(
                this::getChassisSpeeds,
                true,
                true,
                true));
        one.trigger().toggleOnTrue(chassis.turtle());
        two.trigger().onTrue(localization.zeroHeading());

        // Lights commands
        one.button(3).onTrue(intake.wantCone());
        one.button(4).onTrue(intake.wantCube());

        // intake commands
        controller.a()
                .onTrue(intake.intakeCone())
                .onFalse(intake.hold());
        controller.x()
                .onTrue(intake.intakeCube())
                .onFalse(intake.hold());
        controller.y()
                .onTrue(intake.eject())
                .onFalse(intake.stop());
    }

    public Command getAutonomousCommand() {
        return autos.get();
    }
}
