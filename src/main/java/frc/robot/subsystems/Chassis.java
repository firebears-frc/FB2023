package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class Chassis extends SubsystemBase {
    private static final class Constants {
        public static final double ROBOT_WIDTH = Units.inchesToMeters(25);
        public static final double ROBOT_LENGTH = Units.inchesToMeters(34);
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

        // Driving
        public static final double MAX_TELE_VELOCITY = 4.8; // meters per second
        public static final double SLOW_TELE_VELOCITY = 1.0; // meters per second
        public static final double MAX_TELE_ANGULAR_VELOCITY = 1.5 * Math.PI; // radians per second
        public static final double SLOW_TELE_ANGULAR_VELOCITY = Math.PI / 2; // radians per second

        // Trajectories
        public static final double MAX_AUTO_VELOCITY = 4.5; // meters per second
        public static final double MAX_AUTO_ACCELERATION = 3.0; // meters per second squared
        public static final double MAX_AUTO_ANGULAR_VELOCITY = Math.PI; // radians per second
        public static final double MAX_AUTO_ANGULAR_ACCELERATION = Math.PI; // radians per second squared
        public static final double X_CONTROLLER_P = 1.0;
        public static final double Y_CONTROLLER_P = 1.0;
        public static final double R_CONTROLLER_P = 1.0;

        // Charge Station
        public static final double LEVEL_TOLERANCE = 2.0; // degrees
        public static final double ON_TOLERANCE = 10.0; // degrees
        public static final double PITCH_VELOCITY_MAX = 0.2; // degrees per cycle

        // Slew Rate Limiting
        public static final double DIRECTION_SLEW_RATE = 1.2; // radians per second
        public static final double MAGNITUDE_SLEW_RATE = 1.8; // percent per second (1 = 100%)
        public static final double ROTATION_SLEW_RATE = 2.0; // percent per second (1 = 100%)
    }

    // Driving
    private final SwerveModule[] modules;
    private final SwerveDriveKinematics kinematics;

    // Localization
    private AHRS navX;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Field2d field;

    // Trajectories
    private final TrapezoidProfile.Constraints rotationConstraints;
    private final TrajectoryConfig config;

    // Charge Station
    private double lastPitch = 0;
    private double pitchVelocity = 0;

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

        try {
            navX = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            DriverStation.reportError(ex.getMessage(), true);
        }
        poseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                navX.getRotation2d(),
                getModulePositions(),
                new Pose2d());
        field = new Field2d();
        addChild("Field", field);

        rotationConstraints = new TrapezoidProfile.Constraints(Constants.MAX_AUTO_ANGULAR_VELOCITY,
                Constants.MAX_AUTO_ANGULAR_ACCELERATION);
        config = new TrajectoryConfig(Constants.MAX_AUTO_VELOCITY, Constants.MAX_AUTO_ACCELERATION);
        config.setKinematics(kinematics);
    }

    private SwerveModulePosition[] getModulePositions() {
        // Build up position array
        SwerveModulePosition result[] = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            result[i] = modules[i].getPosition();
        }
        return result;
    }

    private SwerveModuleState[] getModuleStates() {
        // Build up state array
        SwerveModuleState result[] = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            result[i] = modules[i].getState();
        }
        return result;
    }

    @Override
    public void periodic() {
        // Update pose estimate
        if (navX != null) {
            poseEstimator.update(
                    navX.getRotation2d(),
                    getModulePositions());
            field.setRobotPose(getPose());
        }

        // Update pitch velocity
        double currentPitch = getPitchDegrees();
        pitchVelocity = currentPitch - lastPitch;
        lastPitch = currentPitch;

        for (int i = 0; i < modules.length; i++) {
            modules[i].periodic();
        }

        Logger logger = Logger.getInstance();
        logger.recordOutput("Chassis/Pose", getPose());
        logger.recordOutput("Chassis/Pitch", currentPitch);
        logger.recordOutput("Chassis/PitchVelocity", pitchVelocity);
        logger.recordOutput("Chassis/Actual", getModuleStates());
    }

    /****************** DRIVING ******************/
    public void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
        if (fieldRelative && navX != null)
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, navX.getRotation2d());

        swerveDrive(kinematics.toSwerveModuleStates(chassisSpeeds));
    }

    public void swerveDrive(SwerveModuleState states[]) {
        if (states.length != Constants.MODULES.length)
            throw new IllegalStateException(
                    "Swerve module count error: " + states.length + ", " + Constants.MODULES.length);

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.MAX_TELE_VELOCITY);

        Logger.getInstance().recordOutput("Chassis/Target", states);
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

    /****************** LOCALIZATION ******************/
    public void visionPose(Pose2d pose, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(pose, timestampSeconds);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(navX.getRotation2d(), getModulePositions(), pose);
    }

    /****************** TRAJECTORIES ******************/
    private Trajectory generateTrajectory(Pose2d start, Pose2d end, boolean reversed) {
        return generateTrajectory(start, new ArrayList<>(), end, reversed);
    }

    private Trajectory generateTrajectory(Pose2d start, List<Translation2d> interior, Pose2d end, boolean reversed) {
        config.setReversed(reversed);
        return TrajectoryGenerator.generateTrajectory(start, interior, end, config);
    }

    private SwerveControllerCommand generateSwerveControllerCommand(Trajectory trajectory) {
        ProfiledPIDController rotationController = new ProfiledPIDController(Constants.R_CONTROLLER_P, 0.0, 0.0,
                rotationConstraints);
        return new SwerveControllerCommand(
                trajectory,
                this::getPose,
                kinematics,
                new PIDController(Constants.X_CONTROLLER_P, 0.0, 0.0),
                new PIDController(Constants.Y_CONTROLLER_P, 0.0, 0.0),
                rotationController,
                this::swerveDrive,
                this);
    }

    public Command driveTrajectory(Pose2d start, Pose2d end, boolean reversed) {
        Trajectory trajectory = generateTrajectory(start, end, reversed);
        return generateSwerveControllerCommand(trajectory);
    }

    public Command driveTrajectory(Pose2d start, List<Translation2d> interior, Pose2d end, boolean reversed) {
        Trajectory trajectory = generateTrajectory(start, interior, end, reversed);
        return generateSwerveControllerCommand(trajectory);
    }

    public Command driveDistance(double distance) {
        Trajectory trajectory = generateTrajectory(
                new Pose2d(0.0, 0.0, new Rotation2d()),
                new Pose2d(distance, 0.0, new Rotation2d()),
                distance < 0);
        return generateSwerveControllerCommand(trajectory);
    }

    /****************** CHARGE STATION ******************/
    public double getPitchDegrees() {
        if (navX == null)
            return 0;

        return navX.getPitch();
    }

    public boolean isLevel() {
        return Math.abs(getPitchDegrees()) < Constants.LEVEL_TOLERANCE;
    }

    public boolean isOnChargeStation() {
        return Math.abs(getPitchDegrees()) > Constants.ON_TOLERANCE;
    }

    private double getPitchVelocityDegrees() {
        return pitchVelocity;
    }

    public boolean isNotPitching() {
        return Math.abs(getPitchVelocityDegrees()) < Constants.PITCH_VELOCITY_MAX;
    }

    /****************** ROTATION ******************/
    public double getYawDegrees() {
        if (navX == null)
            return 0;

        return navX.getYaw();
    }

    /****************** COMMANDS ******************/
    public Command turtle() {
        return new StartEndCommand(this::setX, null, this);
    }

    public Command zeroHeading() {
        return new InstantCommand(() -> {
            setPose(new Pose2d());
        }, this);
    }

    public Command defaultCommand(Supplier<ChassisSpeeds> commandSupplier, boolean slowMode, boolean fieldRelative,
            boolean rateLimit) {
        return new DefaultCommand(this, commandSupplier, slowMode, fieldRelative, rateLimit);
    }

    private class DefaultCommand extends Command {
        private final Chassis chassis;
        private final Supplier<ChassisSpeeds> commandSupplier;
        private final boolean slowMode;
        private final boolean fieldRelative;
        private final boolean rateLimit;

        private final RateLimiter rateLimiter;

        public DefaultCommand(Chassis chassis, Supplier<ChassisSpeeds> commandSupplier,
                boolean slowMode, boolean fieldRelative, boolean rateLimit) {
            this.chassis = chassis;
            this.commandSupplier = commandSupplier;
            this.slowMode = slowMode;
            this.fieldRelative = fieldRelative;
            this.rateLimit = rateLimit;

            rateLimiter = new RateLimiter();

            addRequirements(chassis);
        }

        @Override
        public void execute() {
            ChassisSpeeds command = commandSupplier.get();

            Logger logger = Logger.getInstance();
            logger.recordOutput("Chassis/Input/X", command.vxMetersPerSecond);
            logger.recordOutput("Chassis/Input/Y", command.vyMetersPerSecond);
            logger.recordOutput("Chassis/Input/R", command.omegaRadiansPerSecond);

            if (rateLimit) {
                command = rateLimiter.calculate(command);
            }

            if (slowMode) {
                command.vxMetersPerSecond *= Constants.SLOW_TELE_VELOCITY;
                command.vyMetersPerSecond *= Constants.SLOW_TELE_VELOCITY;
                command.omegaRadiansPerSecond *= Constants.SLOW_TELE_ANGULAR_VELOCITY;
            } else {
                command.vxMetersPerSecond *= Constants.MAX_TELE_VELOCITY;
                command.vyMetersPerSecond *= Constants.MAX_TELE_VELOCITY;
                command.omegaRadiansPerSecond *= Constants.MAX_TELE_ANGULAR_VELOCITY;
            }

            logger.recordOutput("Chassis/Actual/X", command.vxMetersPerSecond);
            logger.recordOutput("Chassis/Actual/Y", command.vyMetersPerSecond);
            logger.recordOutput("Chassis/Actual/R", command.omegaRadiansPerSecond);

            chassis.drive(command, fieldRelative);
        }
    }

    private static class RateLimiter {
        private SlewRateLimiter magnitudeLimiter = new SlewRateLimiter(Constants.MAGNITUDE_SLEW_RATE);
        private SlewRateLimiter rotationLimiter = new SlewRateLimiter(Constants.ROTATION_SLEW_RATE);
        private double currentDirection = 0.0;
        private double currentMagnitude = 0.0;
        private double previousTime = WPIUtilJNI.now() * 1e-6;

        private static double angleDifference(double angleOne, double angleTwo) {
            double difference = Math.abs(angleOne - angleTwo);
            return difference > Math.PI ? (2 * Math.PI) - difference : difference;
        }

        private static double wrapAngle(double angle) {
            if (angle == (Math.PI * 2)) {
                return 0.0;
            } else if (angle > (Math.PI * 2)) {
                return angle - (Math.PI * 2) * Math.floor(angle / (Math.PI * 2));
            } else if (angle < 0.0) {
                return angle + (Math.PI * 2) * Math.floor((-angle / (Math.PI * 2)) + 1);
            } else {
                return angle;
            }
        }

        private static double stepTowardsCircular(double current, double target, double step) {
            current = wrapAngle(current);
            target = wrapAngle(target);

            double direction = Math.signum(target - current);
            double difference = Math.abs(current - target);

            if (difference <= step) {
                return target;
            }

            if (difference > Math.PI) {
                if (current + (2 * Math.PI) - target < step || target + (2 * Math.PI) - current < step) {
                    return target;
                } else {
                    return wrapAngle(current - direction * step);
                }
            }

            return current + direction * step;
        }

        public ChassisSpeeds calculate(ChassisSpeeds command) {
            // Convert XY to polar for rate limiting
            double inputDirection = Math.atan2(command.vyMetersPerSecond, command.vxMetersPerSecond);
            double inputMagnitude = Math
                    .sqrt(Math.pow(command.vxMetersPerSecond, 2) + Math.pow(command.vyMetersPerSecond, 2));

            // Calculate the direction slew rate based on an estimate of the lateral
            // acceleration
            double directionSlewRate;
            if (currentMagnitude != 0.0) {
                directionSlewRate = Math.abs(Constants.DIRECTION_SLEW_RATE / currentMagnitude);
            } else {
                directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - previousTime;
            double angleDif = angleDifference(inputDirection, currentDirection);
            if (angleDif < 0.45 * Math.PI) {
                currentDirection = stepTowardsCircular(currentDirection, inputDirection,
                        directionSlewRate * elapsedTime);
                currentMagnitude = magnitudeLimiter.calculate(inputMagnitude);
            } else if (angleDif > 0.85 * Math.PI) {
                if (currentMagnitude > 0.01) {
                    currentMagnitude = magnitudeLimiter.calculate(0.0);
                } else {
                    currentDirection = wrapAngle(currentDirection + Math.PI);
                    currentMagnitude = magnitudeLimiter.calculate(inputMagnitude);
                }
            } else {
                currentDirection = stepTowardsCircular(currentDirection, inputDirection,
                        directionSlewRate * elapsedTime);
                currentMagnitude = magnitudeLimiter.calculate(0.0);
            }
            previousTime = currentTime;

            command.vxMetersPerSecond = currentMagnitude * Math.cos(currentDirection);
            command.vyMetersPerSecond = currentMagnitude * Math.sin(currentDirection);
            command.omegaRadiansPerSecond = rotationLimiter.calculate(command.omegaRadiansPerSecond);

            Logger logger = Logger.getInstance();
            logger.recordOutput("Chassis/RateLimiter/Magnitude", currentMagnitude);
            logger.recordOutput("Chassis/RateLimiter/Direction", currentDirection);

            return command;
        }
    }
}
