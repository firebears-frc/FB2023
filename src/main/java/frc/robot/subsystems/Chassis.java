package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax.ControlType;
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
        public static int RIGHT_FRONT_PORT = 15;
        public static int RIGHT_BACK_PORT = 17;
        public static int LEFT_FRONT_PORT = 16;
        public static int LEFT_BACK_PORT = 18;

        public static int STALL_CURRENT_LIMIT = 30;
        public static int FREE_CURRENT_LIMIT = 20;
        public static double SECONDARY_CURRENT_LIMIT = 60.0;

        public static double TRACK_WIDTH = 0.96679; // Meters

        private static double P = 0.00036534;
        private static double I = 0;
        private static double D = 0;
    }

    private CANSparkMax rightFrontMotor;
    private SparkMaxPIDController rightPID;
    private RelativeEncoder rightEncoder;
    private CANSparkMax rightBackMotor;
    private CANSparkMax leftFrontMotor;
    private SparkMaxPIDController leftPID;
    private RelativeEncoder leftEncoder;
    private CANSparkMax leftBackMotor;
    private DifferentialDriveKinematics kinematics;
    private AHRS navX;

    public Chassis() {
        rightFrontMotor = new CANSparkMax(Constants.RIGHT_FRONT_PORT, MotorType.kBrushless);
        rightFrontMotor.restoreFactoryDefaults();
        rightFrontMotor.setInverted(false);
        rightFrontMotor.setIdleMode(IdleMode.kCoast);
        rightFrontMotor.setSmartCurrentLimit(Constants.STALL_CURRENT_LIMIT, Constants.FREE_CURRENT_LIMIT);
        rightFrontMotor.setSecondaryCurrentLimit(Constants.SECONDARY_CURRENT_LIMIT);
        rightEncoder = rightFrontMotor.getEncoder();
        rightPID = rightFrontMotor.getPIDController();
        rightPID.setP(Constants.P);
        rightPID.setI(Constants.I);
        rightPID.setD(Constants.D);
        rightFrontMotor.burnFlash();

        rightBackMotor = new CANSparkMax(Constants.RIGHT_BACK_PORT, MotorType.kBrushless);
        rightBackMotor.restoreFactoryDefaults();
        rightBackMotor.setInverted(false);
        rightBackMotor.setIdleMode(IdleMode.kCoast);
        rightBackMotor.setSmartCurrentLimit(Constants.STALL_CURRENT_LIMIT, Constants.FREE_CURRENT_LIMIT);
        rightBackMotor.setSecondaryCurrentLimit(Constants.SECONDARY_CURRENT_LIMIT);
        rightBackMotor.follow(rightFrontMotor);
        rightBackMotor.burnFlash();

        leftFrontMotor = new CANSparkMax(Constants.LEFT_FRONT_PORT, MotorType.kBrushless);
        leftFrontMotor.restoreFactoryDefaults();
        leftFrontMotor.setInverted(true);
        leftFrontMotor.setIdleMode(IdleMode.kCoast);
        leftFrontMotor.setSmartCurrentLimit(Constants.STALL_CURRENT_LIMIT, Constants.FREE_CURRENT_LIMIT);
        leftFrontMotor.setSecondaryCurrentLimit(Constants.SECONDARY_CURRENT_LIMIT);
        leftEncoder = leftFrontMotor.getEncoder();
        leftPID = leftFrontMotor.getPIDController();
        leftPID.setP(Constants.P);
        leftPID.setI(Constants.I);
        leftPID.setD(Constants.D);
        leftFrontMotor.burnFlash();

        leftBackMotor = new CANSparkMax(Constants.LEFT_BACK_PORT, MotorType.kBrushless);
        leftBackMotor.restoreFactoryDefaults();
        leftBackMotor.setInverted(true);
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

    public void drive(ChassisSpeeds chassisSpeeds) {
        SmartDashboard.putNumber("Forward", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Rotation", chassisSpeeds.omegaRadiansPerSecond);

        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

        SmartDashboard.putNumber("Left", wheelSpeeds.leftMetersPerSecond);
        SmartDashboard.putNumber("Right", wheelSpeeds.rightMetersPerSecond);

        rightPID.setReference(wheelSpeeds.rightMetersPerSecond, ControlType.kVelocity);
        leftPID.setReference(wheelSpeeds.rightMetersPerSecond, ControlType.kVelocity);
    }

    public static class OperatorInterface {
        private static class Constants {
            private static double MAX_VELOCITY = 3; // Meters per second
            private static double MAX_ANGULAR_VELOCITY = 16; // Radians per second
        }

        public static ChassisSpeeds toChassisSpeeds(double forward, double rotation) {
            return new ChassisSpeeds(
                    forward * Constants.MAX_VELOCITY,
                    0, // "Driving sideways is a waste of time"
                    rotation * Constants.MAX_ANGULAR_VELOCITY);
        }
    }
}
