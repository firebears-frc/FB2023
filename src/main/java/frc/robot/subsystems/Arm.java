package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.ArmConstants;

public class Arm extends SubsystemBase {
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
