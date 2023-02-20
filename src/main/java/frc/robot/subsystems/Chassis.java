package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Chassis extends SubsystemBase {
    private static class Constants {
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

        private static double MAX_VELOCITY = 3; // Meters per second
        private static double MAX_ANGULAR_VELOCITY = 10; // Radians per second

        private static double P = 0.0064971;
        private static double I = 0;
        private static double D = 0;
        private static double F = 1.0 / MAX_VELOCITY;
    }

    private CANSparkMax rightFrontMotor;
    private SparkMaxPIDController rightPID;
    private double rightSetpoint;
    private RelativeEncoder rightEncoder;
    private CANSparkMax rightBackMotor;
    private CANSparkMax leftFrontMotor;
    private SparkMaxPIDController leftPID;
    private double leftSetpoint;
    private RelativeEncoder leftEncoder;
    private CANSparkMax leftBackMotor;
    private DifferentialDriveKinematics kinematics;
    private AHRS navX;

    public Chassis() {
        rightFrontMotor = new CANSparkMax(Constants.RIGHT_FRONT_PORT, MotorType.kBrushless);
        rightFrontMotor.restoreFactoryDefaults();
        rightFrontMotor.setInverted(true);
        rightFrontMotor.setIdleMode(IdleMode.kCoast);
        rightFrontMotor.setSmartCurrentLimit(Constants.STALL_CURRENT_LIMIT, Constants.FREE_CURRENT_LIMIT);
        rightFrontMotor.setSecondaryCurrentLimit(Constants.SECONDARY_CURRENT_LIMIT);
        rightEncoder = rightFrontMotor.getEncoder();
        rightEncoder.setVelocityConversionFactor(Constants.CONVERSION_FACTOR);
        rightPID = rightFrontMotor.getPIDController();
        rightPID.setP(Constants.P);
        rightPID.setI(Constants.I);
        rightPID.setD(Constants.D);
        rightPID.setFF(Constants.F);
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
        leftEncoder.setVelocityConversionFactor(Constants.CONVERSION_FACTOR);
        leftPID = leftFrontMotor.getPIDController();
        leftPID.setP(Constants.P);
        leftPID.setI(Constants.I);
        leftPID.setD(Constants.D);
        leftPID.setFF(Constants.F);
        leftFrontMotor.burnFlash();

        leftBackMotor = new CANSparkMax(Constants.LEFT_BACK_PORT, MotorType.kBrushless);
        leftBackMotor.restoreFactoryDefaults();
        leftBackMotor.setInverted(false);
        leftBackMotor.setIdleMode(IdleMode.kCoast);
        leftBackMotor.setSmartCurrentLimit(Constants.STALL_CURRENT_LIMIT, Constants.FREE_CURRENT_LIMIT);
        leftBackMotor.setSecondaryCurrentLimit(Constants.SECONDARY_CURRENT_LIMIT);
        leftBackMotor.follow(leftFrontMotor);
        leftBackMotor.burnFlash();

        kinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH);

        try {
            navX = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            DriverStation.reportError(ex.getMessage(), true);
        }
    }

    @Override
    public void periodic() {
        double leftVelocity = leftEncoder.getVelocity();
        double rightVelocity = rightEncoder.getVelocity();

        SmartDashboard.putNumber("Left Velocity", leftVelocity);
        SmartDashboard.putNumber("Right Velocity", rightVelocity);

        SmartDashboard.putNumber("Left Error", leftVelocity - leftSetpoint);
        SmartDashboard.putNumber("Right Error", rightVelocity - rightSetpoint);
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

        SmartDashboard.putNumber("Left Setpoint", leftSetpoint);
        SmartDashboard.putNumber("Right Setpoint", rightSetpoint);

        rightPID.setReference(rightSetpoint, CANSparkMax.ControlType.kVelocity);
        leftPID.setReference(leftSetpoint, CANSparkMax.ControlType.kVelocity);
    }
}
