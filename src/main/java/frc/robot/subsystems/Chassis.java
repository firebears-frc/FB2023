package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Chassis extends SubsystemBase {
    public static class Constants {
        private static int RIGHT_FRONT_PORT = 15;
        private static int RIGHT_BACK_PORT = 17;
        private static int LEFT_FRONT_PORT = 16;
        private static int LEFT_BACK_PORT = 18;

        private static int STALL_CURRENT_LIMIT = 30;
        private static int FREE_CURRENT_LIMIT = 20;
        private static double SECONDARY_CURRENT_LIMIT = 60.0;

        private static double GEAR_RATIO = (52.0 / 10.0) * (68.0 / 30.0);
        private static double WHEEL_DIAMETER = 0.2032; // 8 inches
        private static double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        private static double METERS_PER_MOTOR_ROTATION = WHEEL_CIRCUMFERENCE / GEAR_RATIO;
        private static double CONVERSION_FACTOR = METERS_PER_MOTOR_ROTATION / 60; // The raw units are RPM
        private static double TRACK_WIDTH = 0.96679; // Meters

        public static double MAX_VELOCITY = 5; // Meters per second
        public static double MAX_ACCELERATION = 5; // Meters per second squared
        private static double MAX_ANGULAR_VELOCITY = 8; // Radians per second

        // Values spit out of sysid
        private static double P = 0.011179;
        private static double I = 0;
        private static double D = 0;
        private static double S = 0.15473;
        private static double V = 2.3007;
        private static double A = 0.22029;
    }

    private CANSparkMax rightFrontMotor;
    private RelativeEncoder rightEncoder;
    private CANSparkMax rightBackMotor;
    private double rightSetpoint;

    private CANSparkMax leftFrontMotor;
    private RelativeEncoder leftEncoder;
    private CANSparkMax leftBackMotor;
    private double leftSetpoint;

    private PIDController rightPID;
    private SimpleMotorFeedforward rightFF;
    private PIDController leftPID;
    private SimpleMotorFeedforward leftFF;
    private DifferentialDriveKinematics kinematics;
    private AHRS navX;
    private DifferentialDriveOdometry odometry;
    private Field2d field;

    public Chassis() {
        rightFrontMotor = new CANSparkMax(Constants.RIGHT_FRONT_PORT, MotorType.kBrushless);
        rightFrontMotor.restoreFactoryDefaults();
        rightFrontMotor.setInverted(true);
        rightFrontMotor.setIdleMode(IdleMode.kCoast);
        rightFrontMotor.setSmartCurrentLimit(Constants.STALL_CURRENT_LIMIT, Constants.FREE_CURRENT_LIMIT);
        rightFrontMotor.setSecondaryCurrentLimit(Constants.SECONDARY_CURRENT_LIMIT);
        rightEncoder = rightFrontMotor.getEncoder();
        rightEncoder.setPositionConversionFactor(Constants.METERS_PER_MOTOR_ROTATION);
        rightEncoder.setVelocityConversionFactor(Constants.CONVERSION_FACTOR);
        rightFrontMotor.burnFlash();

        rightBackMotor = new CANSparkMax(Constants.RIGHT_BACK_PORT, MotorType.kBrushless);
        rightBackMotor.restoreFactoryDefaults();
        rightBackMotor.setInverted(true);
        rightBackMotor.setIdleMode(IdleMode.kCoast);
        rightBackMotor.setSmartCurrentLimit(Constants.STALL_CURRENT_LIMIT, Constants.FREE_CURRENT_LIMIT);
        rightBackMotor.setSecondaryCurrentLimit(Constants.SECONDARY_CURRENT_LIMIT);
        rightBackMotor.follow(rightFrontMotor);
        rightBackMotor.burnFlash();

        leftFrontMotor = new CANSparkMax(Constants.LEFT_FRONT_PORT, MotorType.kBrushless);
        leftFrontMotor.restoreFactoryDefaults();
        leftFrontMotor.setInverted(false);
        leftFrontMotor.setIdleMode(IdleMode.kCoast);
        leftFrontMotor.setSmartCurrentLimit(Constants.STALL_CURRENT_LIMIT, Constants.FREE_CURRENT_LIMIT);
        leftFrontMotor.setSecondaryCurrentLimit(Constants.SECONDARY_CURRENT_LIMIT);
        leftEncoder = leftFrontMotor.getEncoder();
        leftEncoder.setPositionConversionFactor(Constants.METERS_PER_MOTOR_ROTATION);
        leftEncoder.setVelocityConversionFactor(Constants.CONVERSION_FACTOR);
        leftFrontMotor.burnFlash();

        leftBackMotor = new CANSparkMax(Constants.LEFT_BACK_PORT, MotorType.kBrushless);
        leftBackMotor.restoreFactoryDefaults();
        leftBackMotor.setInverted(false);
        leftBackMotor.setIdleMode(IdleMode.kCoast);
        leftBackMotor.setSmartCurrentLimit(Constants.STALL_CURRENT_LIMIT, Constants.FREE_CURRENT_LIMIT);
        leftBackMotor.setSecondaryCurrentLimit(Constants.SECONDARY_CURRENT_LIMIT);
        leftBackMotor.follow(leftFrontMotor);
        leftBackMotor.burnFlash();

        rightPID = new PIDController(Constants.P, Constants.I, Constants.D);
        rightFF = new SimpleMotorFeedforward(Constants.S, Constants.V, Constants.A);
        leftPID = new PIDController(Constants.P, Constants.I, Constants.D);
        leftFF = new SimpleMotorFeedforward(Constants.S, Constants.V, Constants.A);
        kinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);
        try {
            navX = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            DriverStation.reportError(ex.getMessage(), true);
        }
        odometry = new DifferentialDriveOdometry(
                navX.getRotation2d(),
                leftEncoder.getPosition(),
                rightEncoder.getPosition());
        field = new Field2d();
        SmartDashboard.putData(field);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Setpoint", leftSetpoint);
        double leftVelocity = leftEncoder.getVelocity();
        SmartDashboard.putNumber("Left Velocity", leftVelocity);
        double leftFFVoltage = leftFF.calculate(leftSetpoint);
        SmartDashboard.putNumber("Left FF", leftFFVoltage);
        double leftFBVoltage = leftPID.calculate(leftVelocity, leftSetpoint);
        SmartDashboard.putNumber("Left FB", leftFBVoltage);
        double leftVoltage = leftFFVoltage + leftFBVoltage;
        SmartDashboard.putNumber("Left Voltage", leftVoltage);
        leftFrontMotor.setVoltage(leftVoltage);

        SmartDashboard.putNumber("Right Setpoint", rightSetpoint);
        double rightVelocity = rightEncoder.getVelocity();
        SmartDashboard.putNumber("Right Velocity", rightVelocity);
        double rightFFVoltage = rightFF.calculate(rightSetpoint);
        SmartDashboard.putNumber("Right FF", rightFFVoltage);
        double rightFBVoltage = rightPID.calculate(rightVelocity, rightSetpoint);
        SmartDashboard.putNumber("Right FB", rightFBVoltage);
        double rightVoltage = rightFFVoltage + rightFBVoltage;
        SmartDashboard.putNumber("Right Voltage", rightVoltage);
        rightFrontMotor.setVoltage(rightVoltage);

        double leftDistance = leftEncoder.getPosition();
        SmartDashboard.putNumber("Left Distance", leftDistance);
        double rightDistance = rightEncoder.getPosition();
        SmartDashboard.putNumber("Right Distance", rightDistance);

        odometry.update(navX.getRotation2d(), leftDistance, rightDistance);
        field.setRobotPose(odometry.getPoseMeters());
    }

    public void resetPose(Pose2d pose) {
        navX.reset();
        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);
        odometry.resetPosition(
                navX.getRotation2d(),
                leftEncoder.getPosition(),
                rightEncoder.getPosition(),
                pose);
    }

    public void drive(double forward, double rotation) {
        drive(new ChassisSpeeds(
                forward * Constants.MAX_VELOCITY,
                0, // "Driving sideways is a waste of time"
                rotation * Constants.MAX_ANGULAR_VELOCITY));
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        drive(kinematics.toWheelSpeeds(chassisSpeeds));
    }

    public void drive(DifferentialDriveWheelSpeeds wheelSpeeds) {
        leftSetpoint = wheelSpeeds.leftMetersPerSecond;
        rightSetpoint = wheelSpeeds.rightMetersPerSecond;
    }
}
