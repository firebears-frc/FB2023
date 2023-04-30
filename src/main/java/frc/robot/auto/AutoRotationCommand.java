package frc.robot.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class AutoRotationCommand extends CommandBase {
    private final Chassis chassis;
    private final double deltaAngle;
    private double targetAngle;

    public AutoRotationCommand(Chassis chassis, double deltaAngle) {
        this.chassis = chassis;
        this.deltaAngle = deltaAngle;

        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        targetAngle = chassis.getYawDegrees() + deltaAngle;

        while (targetAngle > 180.0) {
            targetAngle -= 360.0;
        }
        while (targetAngle < -180.0) {
            targetAngle += 360.0;
        }
    }

    @Override
    public void execute() {
        chassis.drive(new ChassisSpeeds(0.0, 0.0, Math.copySign(Math.PI, deltaAngle)));
    }

    @Override
    public void end(boolean interrupted) {
        chassis.drive(new ChassisSpeeds());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(chassis.getYawDegrees() - targetAngle) <= 5.0;
    }
}
