package frc.robot.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    private final Chassis chassis;
    private final Localization localization;
    // private final Trajectories trajectories;

    public Drive() {
        chassis = new Chassis();
        localization = new Localization(chassis.getKinematics(), chassis::getModulePositions);
    }

    @Override
    public void periodic() {
        localization.periodic();
    }

    public Command turtle() {
        return startEnd(chassis::setX, null);
    }

    public Command zeroHeading() {
        return runOnce(() -> {
            Pose2d pose = localization.getPose();
            localization.setPose(new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromDegrees(0.0)));
        });
    }

    public Command defaultCommand(Supplier<ChassisSpeeds> commandSupplier, boolean slowMode) {
        return new DefaultCommand(commandSupplier,
                speeds -> chassis.driveFieldRelative(speeds, localization.getRawYaw()), slowMode, this);
    }
}
