package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chassis extends SubsystemBase {
    public static final double ROBOT_WIDTH = Units.inchesToMeters(25);
    public static final double ROBOT_LENGTH = Units.inchesToMeters(34);
    public static final double MAX_VELOCITY = 4.8; // meters per second

    private static final class Constants {
        // Wheels are offset 1.75" into the modules
        public static final double TRACK_WIDTH = ROBOT_WIDTH - (Units.inchesToMeters(1.75) * 2);
        public static final double WHEEL_BASE = ROBOT_LENGTH - (Units.inchesToMeters(1.75) * 2);

        public static final SwerveModule.SwerveModuleConfiguration MODULES[] = {
                new SwerveModule.SwerveModuleConfiguration(26, 27, -Math.PI / 2,
                        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), "Front Left"),
                new SwerveModule.SwerveModuleConfiguration(21, 20, 0,
                        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), "Front Right"),
                new SwerveModule.SwerveModuleConfiguration(24, 25, Math.PI,
                        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), "Rear Left"),
                new SwerveModule.SwerveModuleConfiguration(23, 22, Math.PI / 2,
                        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2), "Rear Right")
        };
    }

    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;

    private final Localization localization;
    private final Trajectories trajectories;

    public Chassis() {
        // Build up modules array
        modules = new SwerveModule[Constants.MODULES.length];
        for (int i = 0; i < Constants.MODULES.length; i++) {
            modules[i] = new SwerveModule(Constants.MODULES[i]);
        }
        // Build up position offset array for kinematics
        Translation2d positionOffsets[] = new Translation2d[Constants.MODULES.length];
        for (int i = 0; i < Constants.MODULES.length; i++) {
            positionOffsets[i] = Constants.MODULES[i].positionOffset;
        }
        kinematics = new SwerveDriveKinematics(positionOffsets);

        localization = new Localization(kinematics, this::getModulePositions);
        trajectories = new Trajectories(kinematics, localization::getPose, this::swerveDrive, this);
    }

    public Localization getLocalization() {
        return localization;
    }

    public Trajectories getTrajectories() {
        return trajectories;
    }

    @AutoLogOutput(key = "Chassis/ModulePositions")
    private SwerveModulePosition[] getModulePositions() {
        // Build up position array
        SwerveModulePosition result[] = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            result[i] = modules[i].getPosition();
        }
        return result;
    }

    @AutoLogOutput(key = "Chassis/ModuleStates")
    private SwerveModuleState[] getModuleStates() {
        // Build up state array
        SwerveModuleState result[] = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            result[i] = modules[i].getState();
        }
        return result;
    }

    public void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
        if (fieldRelative && localization.isActive())
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, localization.getRawYaw());

        swerveDrive(kinematics.toSwerveModuleStates(chassisSpeeds));
    }

    private void swerveDrive(SwerveModuleState states[]) {
        if (states.length != Constants.MODULES.length)
            throw new IllegalStateException(
                    "Swerve module count error: " + states.length + ", " + Constants.MODULES.length);

        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY);

        for (int i = 0; i < Constants.MODULES.length; i++) {
            modules[i].setDesiredState(states[i]);
        }
    }

    public void setX() {
        SwerveModuleState[] states = new SwerveModuleState[Constants.MODULES.length];
        for (int i = 0; i < Constants.MODULES.length; i++) {
            states[i] = new SwerveModuleState(0, Constants.MODULES[i].positionOffset.getAngle());
        }
        swerveDrive(states);
    }

    public Command turtle() {
        return startEnd(this::setX, null);
    }

    public Command defaultCommand(Supplier<ChassisSpeeds> commandSupplier, boolean slowMode, boolean fieldRelative,
            boolean rateLimit) {
        return new DefaultCommand(this, commandSupplier, slowMode, fieldRelative, rateLimit);
    }
}
