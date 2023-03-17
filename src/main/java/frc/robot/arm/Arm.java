package frc.robot.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    public static class ArmConstants {
        public static final int ELBOW_PORT = 7;
        public static final int ELBOW_STALL_CURRENT_LIMIT = 40;
        public static final int ELBOW_FREE_CURRENT_LIMIT = 20;
        public static final double ELBOW_SECONDARY_CURRENT_LIMIT = 45.0;
        public static final double ELBOW_MANUAL_SPEED = 1.0; // degrees per loop
        public static final double ELBOW_P = 0.01;
        public static final double ELBOW_I = 0.0;
        public static final double ELBOW_D = 0.005;
        public static final double ELBOW_OFFSET = 240.0; // degrees

        public static final int SHOULDER_RIGHT_PORT = 12;
        public static final int SHOULDER_LEFT_PORT = 13;
        public static final int SHOULDER_STALL_CURRENT_LIMIT = 30;
        public static final int SHOULDER_FREE_CURRENT_LIMIT = 20;
        public static final double SHOULDER_SECONDARY_CURRENT_LIMIT = 35.0;
        public static final double SHOULDER_MANUAL_SPEED = 1.0; // degrees per loop
        public static final double SHOULDER_P = 0.0175;
        public static final double SHOULDER_I = 0.0;
        public static final double SHOULDER_D = 0.005;
        public static final double SHOULDER_OFFSET = 196.0; // degrees

        public static final double SHOULDER_SUBSTATION = 89.0; // degrees
        public static final double ELBOW_SUBSTATION = 295.0; // degrees
        public static final double SHOULDER_STOW = 20.0; // degrees
        public static final double ELBOW_STOW = 220.0; // degrees
        public static final double SHOULDER_GROUND = 130.0; // degrees
        public static final double ELBOW_GROUND = 288.0; // degrees
        public static final double SHOULDER_HIGH = 106.0; // degrees
        public static final double ELBOW_HIGH = 319.0; // degrees
        public static final double SHOULDER_MID = 76.0; // degrees
        public static final double ELBOW_MID = 267.0; // degrees

        public static final double ANGLE_TOLERANCE = 5.0; // degrees
    }

    private CANSparkMax elbowMotor;
    private SparkMaxAbsoluteEncoder elbowEncoder;
    private SparkMaxPIDController elbowPID;
    private double elbowSetpoint;

    private CANSparkMax shoulderMotorRight;
    private CANSparkMax shoulderMotorLeft;
    private SparkMaxAbsoluteEncoder shoulderEncoder;
    private SparkMaxPIDController shoulderPID;
    private double shoulderSetpoint;

    public Arm() {
        elbowMotor = new CANSparkMax(ArmConstants.ELBOW_PORT, MotorType.kBrushless);
        elbowMotor.restoreFactoryDefaults();
        elbowMotor.setInverted(true);
        elbowMotor.setIdleMode(IdleMode.kBrake);
        elbowMotor.setSmartCurrentLimit(ArmConstants.ELBOW_STALL_CURRENT_LIMIT, ArmConstants.ELBOW_FREE_CURRENT_LIMIT);
        elbowMotor.setSecondaryCurrentLimit(ArmConstants.ELBOW_SECONDARY_CURRENT_LIMIT);
        elbowEncoder = elbowMotor.getAbsoluteEncoder(Type.kDutyCycle);
        elbowEncoder.setPositionConversionFactor(360); // degrees
        elbowEncoder.setZeroOffset(ArmConstants.ELBOW_OFFSET);
        elbowEncoder.setInverted(true);
        elbowPID = elbowMotor.getPIDController();
        elbowPID.setFeedbackDevice(elbowEncoder);
        elbowPID.setPositionPIDWrappingEnabled(true);
        elbowPID.setPositionPIDWrappingMinInput(0);
        elbowPID.setPositionPIDWrappingMaxInput(360);
        elbowPID.setP(ArmConstants.ELBOW_P);
        elbowPID.setI(ArmConstants.ELBOW_I);
        elbowPID.setD(ArmConstants.ELBOW_D);
        elbowMotor.burnFlash();

        shoulderMotorRight = new CANSparkMax(ArmConstants.SHOULDER_RIGHT_PORT, MotorType.kBrushless);
        shoulderMotorRight.restoreFactoryDefaults();
        shoulderMotorRight.setInverted(true);
        shoulderMotorRight.setIdleMode(IdleMode.kBrake);
        shoulderMotorRight.setSmartCurrentLimit(ArmConstants.SHOULDER_STALL_CURRENT_LIMIT,
                ArmConstants.SHOULDER_FREE_CURRENT_LIMIT);
        shoulderMotorRight.setSecondaryCurrentLimit(ArmConstants.SHOULDER_SECONDARY_CURRENT_LIMIT);
        shoulderEncoder = shoulderMotorRight.getAbsoluteEncoder(Type.kDutyCycle);
        shoulderEncoder.setPositionConversionFactor(360); // degrees
        shoulderEncoder.setZeroOffset(ArmConstants.SHOULDER_OFFSET);
        shoulderEncoder.setInverted(true);
        shoulderPID = shoulderMotorRight.getPIDController();
        shoulderPID.setFeedbackDevice(shoulderEncoder);
        shoulderPID.setPositionPIDWrappingEnabled(true);
        shoulderPID.setPositionPIDWrappingMinInput(0);
        shoulderPID.setPositionPIDWrappingMaxInput(360);
        shoulderPID.setP(ArmConstants.SHOULDER_P);
        shoulderPID.setI(ArmConstants.SHOULDER_I);
        shoulderPID.setD(ArmConstants.SHOULDER_D);
        shoulderMotorRight.burnFlash();

        shoulderMotorLeft = new CANSparkMax(ArmConstants.SHOULDER_LEFT_PORT, MotorType.kBrushless);
        shoulderMotorLeft.restoreFactoryDefaults();
        shoulderMotorLeft.setInverted(false);
        shoulderMotorLeft.setIdleMode(IdleMode.kBrake);
        shoulderMotorLeft.setSmartCurrentLimit(ArmConstants.SHOULDER_STALL_CURRENT_LIMIT,
                ArmConstants.SHOULDER_FREE_CURRENT_LIMIT);
        shoulderMotorLeft.setSecondaryCurrentLimit(ArmConstants.SHOULDER_SECONDARY_CURRENT_LIMIT);
        shoulderMotorLeft.follow(shoulderMotorRight, true);
        shoulderMotorLeft.burnFlash();
    }

    public void setElbowAngle(double angle) {
        while (angle > 360) {
            angle -= 360;
        }
        while (angle < 0) {
            angle += 360;
        }
        elbowSetpoint = angle;
    }

    public double getElbowTargetAngle() {
        return elbowSetpoint;
    }

    public double getElbowAngle() {
        return elbowEncoder.getPosition();
    }

    public double getElbowError() {
        return getElbowAngle() - getElbowTargetAngle();
    }

    public void setShoulderAngle(double angle) {
        while (angle > 360) {
            angle -= 360;
        }
        while (angle < 0) {
            angle += 360;
        }
        shoulderSetpoint = angle;
    }

    public double getShoulderTargetAngle() {
        return shoulderSetpoint;
    }

    public double getShoulderAngle() {
        return shoulderEncoder.getPosition();
    }

    public double getShoulderError() {
        return getShoulderAngle() - getShoulderTargetAngle();
    }

    public boolean onTarget() {
        return Math.abs(getShoulderError()) < ArmConstants.ANGLE_TOLERANCE
                && Math.abs(getElbowError()) < ArmConstants.ANGLE_TOLERANCE;
    }

    public void setAngles(double elbowAngle, double shoulderAngle) {
        setElbowAngle(elbowAngle);
        setShoulderAngle(shoulderAngle);
    }

    @Override
    public void periodic() {
        elbowPID.setReference(elbowSetpoint, ControlType.kPosition);
        shoulderPID.setReference(shoulderSetpoint, ControlType.kPosition);
    }
}
