package frc.robot.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Localization {
    private static final class Constants {
        public static final Rotation2d LEVEL_TOLERANCE = Rotation2d.fromDegrees(2.0);
        public static final Rotation2d ON_TOLERANCE = Rotation2d.fromDegrees(10.0);
        public static final Rotation2d PITCH_VELOCITY_MAX = Rotation2d.fromDegrees(0.2); // per cycle
    }

    private final SwerveDrivePoseEstimator poseEstimator;
    private final Supplier<SwerveModulePosition[]> positionSupplier;

    private AHRS navX;

    // Charge Station
    @AutoLogOutput(key = "Localization/ChargeStation/LastPitch")
    private Rotation2d lastPitch;
    @AutoLogOutput(key = "Localization/ChargeStation/PitchVelocity")
    private Rotation2d pitchVelocity;

    public Localization(SwerveDriveKinematics kinematics, Supplier<SwerveModulePosition[]> positionSupplier) {
        this.positionSupplier = positionSupplier;

        try {
            navX = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            DriverStation.reportError(ex.getMessage(), true);
            navX = null;
            poseEstimator = null;
            return;
        }

        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                getRawYaw(),
                positionSupplier.get(),
                new Pose2d());

        lastPitch = Rotation2d.fromDegrees(0.0);
        pitchVelocity = Rotation2d.fromDegrees(0.0);
    }

    public boolean isActive() {
        return navX != null;
    }

    @AutoLogOutput(key = "Localization/RawYaw")
    public Rotation2d getRawYaw() {
        if (!isActive())
            return Rotation2d.fromDegrees(0.0);

        return navX.getRotation2d();
    }

    public Rotation2d getPitch() {
        return lastPitch;
    }

    public void visionPose(Pose2d pose, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(pose, timestampSeconds);
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getRawYaw(), positionSupplier.get(), pose);
    }

    @AutoLogOutput(key = "Localization/Pose")
    public Pose2d getPose() {
        if (!isActive())
            return new Pose2d();

        return poseEstimator.getEstimatedPosition();
    }

    public boolean isLevel() {
        return Math.abs(lastPitch.getRadians()) < Constants.LEVEL_TOLERANCE.getRadians();
    }

    public boolean isOnChargeStation() {
        return Math.abs(lastPitch.getRadians()) > Constants.ON_TOLERANCE.getRadians();
    }

    public boolean isNotPitching() {
        return Math.abs(pitchVelocity.getRadians()) < Constants.PITCH_VELOCITY_MAX.getRadians();
    }

    public void periodic() {
        if (!isActive())
            return;

        poseEstimator.update(getRawYaw(), positionSupplier.get());

        // Update pitch velocity
        Rotation2d currentPitch = Rotation2d.fromDegrees(navX.getPitch());
        pitchVelocity = currentPitch.minus(lastPitch);
        lastPitch = currentPitch;
    }
}
