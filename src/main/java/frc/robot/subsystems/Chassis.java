package frc.robot.subsystems;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    private CANSparkMax rightFrontMotor;
    private CANSparkMax rightBackMotor;
    private MotorControllerGroup rightMotors;
    private CANSparkMax leftFrontMotor;
    private CANSparkMax leftBackMotor;
    private MotorControllerGroup leftMotors;
    private DifferentialDrive differentialDrive;
    private AHRS navX;

    private RelativeEncoder rightEncoder;
    private RelativeEncoder leftEncoder;
    private double leftOffSet = 0;
    private double rightOffSet = 0;

    private final DifferentialDriveOdometry m_odometry;

    public Chassis() {
        rightFrontMotor = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);

        rightFrontMotor.restoreFactoryDefaults();
        rightFrontMotor.setInverted(false);
        rightFrontMotor.setIdleMode(IdleMode.kCoast);

        rightBackMotor = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);

        rightBackMotor.restoreFactoryDefaults();
        rightBackMotor.setInverted(false);
        rightBackMotor.setIdleMode(IdleMode.kCoast);

        rightMotors = new MotorControllerGroup(rightFrontMotor, rightBackMotor);
        addChild("rightMotors", rightMotors);

        leftFrontMotor = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);

        leftFrontMotor.restoreFactoryDefaults();
        leftFrontMotor.setInverted(true);
        leftFrontMotor.setIdleMode(IdleMode.kCoast);

        leftBackMotor = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);

        leftBackMotor.restoreFactoryDefaults();
        leftBackMotor.setInverted(true);
        leftBackMotor.setIdleMode(IdleMode.kCoast);

        leftMotors = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
        addChild("leftMotors", leftMotors);

        differentialDrive = new DifferentialDrive(leftMotors, rightMotors);
        addChild("differentialDrive", differentialDrive);
        differentialDrive.setSafetyEnabled(true);
        differentialDrive.setExpiration(0.1);
        differentialDrive.setMaxOutput(1.0);

        rightEncoder = rightFrontMotor.getEncoder();
        leftEncoder = leftFrontMotor.getEncoder();
        leftEncoder.setPositionConversionFactor(1.0/kFeetToMeterFactor);
        rightEncoder.setPositionConversionFactor(1.0/kFeetToMeterFactor);
        
        try {
            navX = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            DriverStation.reportError(ex.getMessage(), true);
        }
        Timer.delay(1.0);
        // LiveWindow.addSensor("Chassis", "navX", navX);
        Rotation2d gyroAngleRadians = Rotation2d.fromDegrees(-getAngle());
        m_odometry = new DifferentialDriveOdometry(gyroAngleRadians, 0.0, 0.0);

    }

    @Override
    public void periodic() {

        Rotation2d gyroAngleRadians = Rotation2d.fromDegrees(-getAngle());
        double leftDistanceMeters = leftDistanceTraveled();
        double rightDistanceMeters = rightDistanceTraveled();
        m_odometry.update(gyroAngleRadians, leftDistanceMeters, rightDistanceMeters);

        SmartDashboard.putNumber("getEncoderDistance", getEncoderDistance());
        SmartDashboard.putNumber("getLeftDistance", leftDistanceTraveled());
        SmartDashboard.putNumber("getRightDistance", rightDistanceTraveled());
        SmartDashboard.putNumber("getPitch", getPitch());
        SmartDashboard.putNumber("getRoll", getRoll());
        SmartDashboard.putNumber("getAngle", getAngle());
    }

    @Override
    public void simulationPeriodic() {

    }

    public void arcadeDrive(double speed, double rotation) {
        speed /= 2;
        differentialDrive.arcadeDrive(speed, rotation);
    }

    public double getEncoderDistance() {
        double meters = ((leftEncoder.getPosition() - leftOffSet) + (rightEncoder.getPosition() * -1 - rightOffSet))
                / 2;

        return meters;
    }

    public double leftDistanceTraveled() {
        return (leftEncoder.getPosition() - leftOffSet);
    }

    public double rightDistanceTraveled() {
        return (rightEncoder.getPosition() - rightOffSet);
    }

    public void resetEncoder() {
        leftOffSet = leftEncoder.getPosition();
        rightOffSet = rightEncoder.getPosition();
    }

    public double getPitch(){
        return (navX.getPitch()) * -1;
    }

    public double getRoll(){
        return navX.getRoll();
    }

    public double getAngle(){
        return navX.getAngle();
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftFrontMotor.setVoltage(leftVolts);
        leftBackMotor.setVoltage(leftVolts);
        rightFrontMotor.setVoltage(rightVolts);
        rightBackMotor.setVoltage(rightVolts);
        differentialDrive.feed();
      }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        double leftMetersPerSecond = leftEncoder.getVelocity();
        double rightMetersPerSecond = rightEncoder.getVelocity();
        return new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond);
      }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
      }

    public void resetOdometry(Pose2d pose) {
        resetEncoder();
        Rotation2d gyroAngleRadians = Rotation2d.fromDegrees(-getAngle());
        m_odometry.resetPosition(gyroAngleRadians, 0.0, 0.0, pose);
      }
}
