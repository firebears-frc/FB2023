package frc.robot;

import frc.robot.commands.ArmGroundCommand;
import frc.robot.commands.ArmHighCommand;
import frc.robot.commands.ArmMidCommand;
import frc.robot.commands.ArmStowCommand;
import frc.robot.commands.ArmSubstationCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Schlucker;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Arm.ArmConstants;
import frc.robot.subsystems.Chassis.ChassisConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class RobotContainer {
    public class RobotConstants {
        public static final int JOYSTICK_PORT = 0;
        public static final int CONTROLLER_PORT = 1;
    }

    private final Chassis chassis;
    private final Arm arm;
    private final Schlucker schlucker;
    private final Vision vision;
    private final Lights lights;
    private final Joystick joystick;
    private final XboxController controller;

    public RobotContainer() {
        chassis = new Chassis();
        arm = new Arm();
        schlucker = new Schlucker();
        vision = new Vision(chassis::resetPose);
        lights = new Lights(chassis::getChargeStationStatus, schlucker::getHeldItem, schlucker::getWantedItem);
        joystick = new Joystick(RobotConstants.JOYSTICK_PORT);
        controller = new XboxController(RobotConstants.CONTROLLER_PORT);

        configureButtonBindings();

        displayGitInfo();
    }

    private void configureButtonBindings() {
        chassis.setDefaultCommand(new RunCommand(() -> {
            double forward = joystick.getY() * -1.0;
            double rotation = joystick.getX() * -1.0;

            if (joystick.getRawButton(1)) {
                forward *= ChassisConstants.SLOW_VELOCITY;
                rotation *= ChassisConstants.SLOW_ANGULAR_VELOCITY;
            } else {
                forward *= ChassisConstants.MAX_VELOCITY;
                rotation *= ChassisConstants.MAX_ANGULAR_VELOCITY;
            }

            chassis.drive(new ChassisSpeeds(forward, 0, rotation));
        }, chassis));

        arm.setDefaultCommand(new RunCommand(() -> {
            double elbow = arm.getElbowTargetAngle();
            elbow += controller.getLeftY() * ArmConstants.ELBOW_MANUAL_SPEED;

            double shoulder = arm.getShoulderTargetAngle();
            shoulder += controller.getRightY() * ArmConstants.SHOULDER_MANUAL_SPEED;

            arm.setAngles(elbow, shoulder);
        }, arm));

        // Arm target point commands
        POVButton upButton = new POVButton(controller, 0);
        upButton.onTrue(new ArmSubstationCommand(arm));
        POVButton rightButton = new POVButton(controller, 90);
        rightButton.onTrue(new ArmMidCommand(arm).andThen(new InstantCommand(schlucker::eject, schlucker)));
        rightButton.onFalse(new InstantCommand(schlucker::stop, schlucker));
        POVButton downButton = new POVButton(controller, 180);
        downButton.onTrue(new ArmGroundCommand(arm));
        POVButton leftButton = new POVButton(controller, 270);
        leftButton.onTrue(new ArmHighCommand(arm).andThen(new InstantCommand(schlucker::eject, schlucker)));
        leftButton.onFalse(new InstantCommand(schlucker::stop, schlucker));
        JoystickButton bButton = new JoystickButton(controller, XboxController.Button.kB.value);
        bButton.onTrue(new ArmStowCommand(arm));

        // Schlucker commands
        JoystickButton aButton = new JoystickButton(controller, XboxController.Button.kA.value);
        aButton.onTrue(new InstantCommand(schlucker::intakeCone, schlucker));
        aButton.onFalse(new InstantCommand(schlucker::hold, schlucker));
        JoystickButton xButton = new JoystickButton(controller, XboxController.Button.kX.value);
        xButton.onTrue(new InstantCommand(schlucker::intakeCube, schlucker));
        xButton.onFalse(new InstantCommand(schlucker::hold, schlucker));
        JoystickButton yButton = new JoystickButton(controller, XboxController.Button.kY.value);
        yButton.onTrue(new InstantCommand(schlucker::eject, schlucker));
        yButton.onFalse(new InstantCommand(schlucker::stop, schlucker));
    }

    public Command getAutonomousCommand() {
        Trajectory trajectory = chassis.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d()),
                new Pose2d(1.0, 0, new Rotation2d()));
        return chassis.generateRamseteCommand(trajectory);
    }

    private String getFileContents(String filename) {
        // Create the file object
        File file = new File(Filesystem.getDeployDirectory(), filename);
        // Open the file stream
        try (FileInputStream inputStream = new FileInputStream(file)) {
            // Prepare the buffer
            byte[] data = new byte[(int) file.length()];
            // Read the data
            data = inputStream.readAllBytes();
            // Format into string and return
            return new String(data, "UTF-8");
        } catch (IOException e) {
            // Print exception and return
            e.printStackTrace();
            return "Unknown";
        }
    }

    private void displayGitInfo() {
        // Get the branch name and display on the dashboard
        String branchName = getFileContents("branch.txt");
        SmartDashboard.putString("Branch Name", branchName);

        // Get the commit hash and display on the dashboard
        String commitHash = getFileContents("commit.txt");
        SmartDashboard.putString("Commit Hash", commitHash.substring(0, 8));
    }
}
