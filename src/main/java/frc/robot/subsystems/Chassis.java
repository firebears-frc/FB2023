package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

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
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
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
                // Front Left
                new SwerveModule.SwerveModuleConfiguration(26, 27, -Math.PI / 2,
                        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2)),
                // Front Right
                new SwerveModule.SwerveModuleConfiguration(21, 20, 0,
                        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2)),
                // Rear Left
                new SwerveModule.SwerveModuleConfiguration(24, 25, Math.PI,
                        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2)),
                // Rear Right
                new SwerveModule.SwerveModuleConfiguration(23, 22, Math.PI / 2,
                        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2))
        };

        // Driving
        public static final double MAX_TELE_VELOCITY = 4.8; // meters per second
        public static final double SLOW_TELE_VELOCITY = 1.0; // meters per second
        public static final double MAX_TELE_ANGULAR_VELOCITY = 2 * Math.PI; // radians per second
        public static final double SLOW_TELE_ANGULAR_VELOCITY = Math.PI; // radians per second

        // Trajectories
        public static final double MAX_AUTO_VELOCITY = 3.0; // meters per second
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

    // Logging
    private final DataLog log;

    public Chassis(DataLog log) {
        // Build up modules array
        modules = new SwerveModule[Constants.MODULES.length];
        for (int i = 0; i < Constants.MODULES.length; i++) {
            modules[i] = new SwerveModule(Constants.MODULES[i], log, i);
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

        this.log = log;
    }

    private SwerveModulePosition[] getModulePositions() {
        // Build up position array
        SwerveModulePosition result[] = new SwerveModulePosition[Constants.MODULES.length];
        for (int i = 0; i < Constants.MODULES.length; i++) {
            result[i] = modules[i].getPosition();
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

        for (int i = 0; i < Constants.MODULES.length; i++) {
            modules[i].periodic();
        }
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

        for (int i = 0; i < Constants.MODULES.length; i++) {
            modules[i].setDesiredState(states[i]);
        }
    }

    public void setX() {
        modules[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        modules[1].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        modules[2].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        modules[3].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
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
        return new RunCommand(this::setX, this);
    }

    public Command zeroHeading() {
        return new RunCommand(() -> {
            setPose(new Pose2d());
        }, this);
    }

    public static class DriveCommand extends CommandBase {
        private final Chassis chassis;
        private final Supplier<Double> forwardSupplier;
        private final Supplier<Double> strafeSupplier;
        private final Supplier<Double> rotationSupplier;
        private final Supplier<Boolean> slowModeSupplier;
        private final boolean fieldRelative;

        private final DoubleLogEntry xLog;
        private final DoubleLogEntry yLog;
        private final DoubleLogEntry rLog;

        private SlewRateLimiter magnitude = new SlewRateLimiter(Constants.MAGNITUDE_SLEW_RATE);
        private SlewRateLimiter rotation = new SlewRateLimiter(Constants.ROTATION_SLEW_RATE);
        private double currentDirection = 0.0;
        private double currentMagnitude = 0.0;
        private double currentRotation = 0.0;
        private double previousTime = WPIUtilJNI.now() * 1e-6;

        public DriveCommand(Chassis chassis, Supplier<Double> forwardSupplier, Supplier<Double> strafeSupplier,
                Supplier<Double> rotationSupplier, Supplier<Boolean> slowModeSupplier, boolean fieldRelative,
                DataLog log) {
            this.chassis = chassis;
            this.forwardSupplier = forwardSupplier;
            this.strafeSupplier = strafeSupplier;
            this.rotationSupplier = rotationSupplier;
            this.slowModeSupplier = slowModeSupplier;
            this.fieldRelative = fieldRelative;

            xLog = new DoubleLogEntry(log, "Drive/Command/X");
            yLog = new DoubleLogEntry(log, "Drive/Command/Y");
            rLog = new DoubleLogEntry(log, "Drive/Command/R");

            addRequirements(chassis);
        }

        private void rateLimit(double forward, double strafe, double rotation) {
            double forwardCommanded;
            double strafeCommanded;

            // Convert XY to polar for rate limiting
            double inputTranslationDir = Math.atan2(strafe, forward);
            double inputTranslationMag = Math.sqrt(Math.pow(forward, 2) + Math.pow(strafe, 2));

            // Calculate the direction slew rate based on an estimate of the lateral acceleration
            double directionSlewRate;
            if (currentMagnitude != 0.0) {
                directionSlewRate = Math.abs(Constants.DIRECTION_SLEW_RATE / currentMagnitude);
            } else {
                directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
            }
            

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - previousTime;
            double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, currentDirection);
            if (angleDif < 0.45*Math.PI) {
                currentDirection = SwerveUtils.StepTowardsCircular(currentDirection, inputTranslationDir, directionSlewRate * elapsedTime);
                currentMagnitude = magnitude.calculate(inputTranslationMag);
            }
            else if (angleDif > 0.85*Math.PI) {
                if (currentMagnitude > 1e-4) { //some small number to avoid floating-point errors with equality checking
                // keep currentTranslationDir unchanged
                currentMagnitude = magnitude.calculate(0.0);
                }
                else {
                currentDirection = SwerveUtils.WrapAngle(currentDirection + Math.PI);
                currentMagnitude = magnitude.calculate(inputTranslationMag);
                }
            }
            else {
                currentDirection = SwerveUtils.StepTowardsCircular(currentDirection, inputTranslationDir, directionSlewRate * elapsedTime);
                currentMagnitude = magnitude.calculate(0.0);
            }
            previousTime = currentTime;
            
            forwardCommanded = currentMagnitude * Math.cos(currentDirection);
            strafeCommanded = currentMagnitude * Math.sin(currentDirection);
            currentRotation = this.rotation.calculate(rotation);
        }

        @Override
        public void execute() {
            double forward = forwardSupplier.get();
            double strafe = strafeSupplier.get();
            double rotation = rotationSupplier.get();

            if (slowModeSupplier.get()) {
                forward *= Constants.SLOW_TELE_VELOCITY;
                strafe *= Constants.SLOW_TELE_VELOCITY;
                rotation *= Constants.SLOW_TELE_ANGULAR_VELOCITY;
            } else {
                forward *= Constants.MAX_TELE_VELOCITY;
                strafe *= Constants.MAX_TELE_VELOCITY;
                rotation *= Constants.MAX_TELE_ANGULAR_VELOCITY;
            }
            xLog.append(forward);
            yLog.append(strafe);
            rLog.append(rotation);

            chassis.drive(new ChassisSpeeds(forward, strafe, rotation), fieldRelative);
        }
    }
}
