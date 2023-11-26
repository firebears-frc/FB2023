package frc.robot;

import frc.robot.arm.Arm;
import frc.robot.drive.Drive;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeBag;
import frc.robot.intake.IntakeNeo550;
import frc.robot.subsystems.Lights;

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

    private final Drive drive;
    private final Arm arm;
    private final Intake intake;
    // private final Lights lights;

    private final PowerDistribution pdh;
    private final CommandJoystick one;
    private final CommandJoystick two;
    private final CommandXboxController controller;
    private final Autos autos;

    public RobotContainer() {
        drive = new Drive();
        arm = new Arm();
        intake = new IntakeBag(); // new IntakeNeo550();
        // vision = new Vision(localization::visionPose);
        // lights = new Lights(intake::getHeldItem, intake::getWantedItem,
        // localization::isLevel,
        // localization::isOnChargeStation, localization::isNotPitching);
        pdh = new PowerDistribution(Constants.PDH_CAN_ID, ModuleType.kRev);

        one = new CommandJoystick(Constants.JOYSTICK_1_PORT);
        two = new CommandJoystick(Constants.JOYSTICK_2_PORT);
        controller = new CommandXboxController(Constants.CONTROLLER_PORT);

        autos = new Autos(drive, arm, intake);

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
        drive.setDefaultCommand(drive.defaultCommand(
                this::getChassisSpeeds,
                false));
        one.button(2).toggleOnTrue(drive.defaultCommand(
                this::getChassisSpeeds,
                true));
        one.trigger().toggleOnTrue(drive.turtle());
        two.trigger().onTrue(drive.zeroHeading());

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
