package frc.robot;

import frc.robot.subsystems.Chassis;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class RobotContainer {
    public class Constants {
        public static final int JOYSTICK_1_PORT = 0;
        public static final int JOYSTICK_2_PORT = 1;
        public static final int CONTROLLER_PORT = 2;

        public static final double JOYSTICK_DEADBAND = 0.05;

        public static final int PDH_CAN_ID = 1;
    }

    private final Chassis chassis;

    private final PowerDistribution pdh;
    private final CommandJoystick joystick_1;
    private final CommandJoystick joystick_2;
    private final SendableChooser<Command> autoSelector;

    public RobotContainer() {
        chassis = new Chassis();
        pdh = new PowerDistribution(Constants.PDH_CAN_ID, ModuleType.kRev);
        joystick_1 = new CommandJoystick(Constants.JOYSTICK_1_PORT);
        joystick_2 = new CommandJoystick(Constants.JOYSTICK_2_PORT);

        autoSelector = new SendableChooser<>();
        autoSelector.addOption("Test Auto Path", chassis.driveTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                new Pose2d(3, 0, new Rotation2d(0)),
                false));

        configureButtonBindings();
    }

    private void configureButtonBindings(DataLog log) {
        chassis.setDefaultCommand(new Chassis.DriveCommand(
                chassis,
                () -> new ChassisSpeeds(
                        -MathUtil.applyDeadband(joystick_1.getY(), Constants.JOYSTICK_DEADBAND),
                        -MathUtil.applyDeadband(joystick_1.getX(), Constants.JOYSTICK_DEADBAND),
                        -MathUtil.applyDeadband(joystick_2.getX(), Constants.JOYSTICK_DEADBAND)),
                () -> joystick_1.getHID().getRawButton(1),
                true,
                true,
                log));

        joystick_1.trigger().whileTrue(chassis.turtle());
        joystick_2.trigger().whileTrue(chassis.zeroHeading());
    }

    public Command getAutonomousCommand() {
        Command auto = autoSelector.getSelected();
        return auto;
    }
}
