package frc.robot.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutoRotationCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final double deltaAngle;
    private double targetAngle;

    public AutoRotationCommand(Drivetrain drivetrain, double deltaAngle) {
        this.drivetrain = drivetrain;
        this.deltaAngle = deltaAngle;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        targetAngle = drivetrain.getYawDegrees() + deltaAngle;

        while (targetAngle > 180.0) {
            targetAngle -= 360.0;
        }
        while (targetAngle < -180.0) {
            targetAngle += 360.0;
        }
    }

    @Override
    public void execute() {
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, Math.copySign(Math.PI, deltaAngle)));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(drivetrain.getYawDegrees() - targetAngle) <= 5.0;
    }
}
