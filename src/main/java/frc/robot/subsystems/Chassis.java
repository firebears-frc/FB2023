package frc.robot.subsystems;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.*;

import com.revrobotics.RelativeEncoder;

public class Chassis extends SubsystemBase {
    private static int STALL_CURRENT_LIMIT = 40;
    private static int FREE_CURRENT_LIMIT = 20;
    private static int SECONDARY_CURRENT_LIMIT = 60;

    private CANSparkMax rightFrontMotor;
    private CANSparkMax rightBackMotor;
    private MotorControllerGroup rightMotors;
    private CANSparkMax leftFrontMotor;
    private CANSparkMax leftBackMotor;
    private MotorControllerGroup leftMotors;
    private DifferentialDriveKinematics differentialDriveKinematics;
    private AHRS navX;

    private RelativeEncoder rightEncoder;
    private RelativeEncoder leftEncoder;

    private final DifferentialDriveOdometry m_odometry;

    private final PIDController leftPID, rightPID;
    private final SimpleMotorFeedforward feedForward;

    public Chassis() {
        rightFrontMotor = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);

        rightFrontMotor.restoreFactoryDefaults();
        rightFrontMotor.setInverted(true);
        rightFrontMotor.setIdleMode(IdleMode.kCoast);
        rightFrontMotor.setSmartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT);
        rightFrontMotor.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT);

        rightBackMotor = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);

        rightBackMotor.restoreFactoryDefaults();
        rightBackMotor.setInverted(true);
        rightBackMotor.setIdleMode(IdleMode.kCoast);
        rightBackMotor.setSmartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT);
        rightBackMotor.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT);

        rightMotors = new MotorControllerGroup(rightFrontMotor, rightBackMotor);
        addChild("rightMotors", rightMotors);

        leftFrontMotor = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);

        leftFrontMotor.restoreFactoryDefaults();
        leftFrontMotor.setInverted(false);
        leftFrontMotor.setIdleMode(IdleMode.kCoast);
        leftFrontMotor.setSmartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT);
        leftFrontMotor.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT);

        leftBackMotor = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);

        leftBackMotor.restoreFactoryDefaults();
        leftBackMotor.setInverted(false);
        leftBackMotor.setIdleMode(IdleMode.kCoast);
        leftBackMotor.setSmartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT);
        leftBackMotor.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT);

        leftMotors = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
        addChild("leftMotors", leftMotors);

        // Create PID Controllers
        leftPID = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
        rightPID = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
        feedForward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);

        differentialDriveKinematics = new DifferentialDriveKinematics(DriveConstants.TrackWidth);

        rightEncoder = rightFrontMotor.getEncoder();
        leftEncoder = leftFrontMotor.getEncoder();

        leftEncoder.setPositionConversionFactor(kFeetToMeterFactor);
        rightEncoder.setPositionConversionFactor(kFeetToMeterFactor);

        leftEncoder.setVelocityConversionFactor(kFeetToMeterFactor / 60.f);
        rightEncoder.setVelocityConversionFactor(kFeetToMeterFactor / 60.f);

        try {
            navX = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            DriverStation.reportError(ex.getMessage(), true);
        }
        Timer.delay(1.0);
        // LiveWindow.addSensor("Chassis", "navX", navX);
        Rotation2d gyroAngleRadians = Rotation2d.fromDegrees(-getAngle());
        m_odometry = new DifferentialDriveOdometry(gyroAngleRadians, 0.0, 0.0);

        rightFrontMotor.burnFlash();
        rightBackMotor.burnFlash();
        leftFrontMotor.burnFlash();
        leftBackMotor.burnFlash();
    }

    @Override
    public void periodic() {
        double leftDist = leftEncoder.getPosition();
        double rightDist = rightEncoder.getPosition();

        double leftVel = leftEncoder.getVelocity();
        double rightVel = rightEncoder.getVelocity();

        double leftFeedForwardVoltage = feedForward.calculate(leftPID.getSetpoint());
        double rightFeedForwardVoltage = feedForward.calculate(rightPID.getSetpoint());

        double leftFeedBackVoltage = leftPID.calculate(leftVel);
        double rightFeedBackVoltage = rightPID.calculate(rightVel);

        leftMotors.setVoltage(leftFeedForwardVoltage + leftFeedBackVoltage);
        rightMotors.setVoltage(rightFeedForwardVoltage + rightFeedBackVoltage);
        SmartDashboard.putNumber("leftSetPoint", leftPID.getSetpoint());
        SmartDashboard.putNumber("rightSetPoint", rightPID.getSetpoint());
        
    }

    public void tankDrive(double left, double right){
        leftPID.setSetpoint(left);
        rightPID.setSetpoint(right);
    }

    public void tankDrive(DifferentialDriveWheelSpeeds speed){
        tankDrive(speed.leftMetersPerSecond,speed.rightMetersPerSecond);
    }

    public void drive(ChassisSpeeds speed){
        tankDrive(differentialDriveKinematics.toWheelSpeeds(speed));
    }

    public void arcadeDrive(double speed, double rotation) {
        drive(new ChassisSpeeds(speed * DriveConstants.MaxVelocity,0,rotation * DriveConstants.MaxAngularVelocity));
    }

    public double getPitch() {
        return (navX.getPitch()) * -1;
    }

    public double getRoll() {
        return navX.getRoll();
    }

    public double getAngle() {
        return navX.getAngle();
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        //resetEncoder();
        Rotation2d gyroAngleRadians = Rotation2d.fromDegrees(-getAngle());
        m_odometry.resetPosition(gyroAngleRadians, 0.0, 0.0, pose);
    }
}
