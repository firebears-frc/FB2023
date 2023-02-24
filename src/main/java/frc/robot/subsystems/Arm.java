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

    private CANSparkMax shoulderMotor1;
    private CANSparkMax shoulderMotor2;
    private SparkMaxAbsoluteEncoder shoulderEncoder;
    private SparkMaxPIDController shoulderPID;
    private double shoulderSetpoint;

    public Arm() {
        elbowMotor = new CANSparkMax(ArmConstants.ELBOW_PORT, MotorType.kBrushless);
        elbowMotor.restoreFactoryDefaults();
        elbowMotor.setInverted(false);
        elbowMotor.setIdleMode(IdleMode.kBrake);
        elbowMotor.setSmartCurrentLimit(ArmConstants.STALL_CURRENT_LIMIT, ArmConstants.FREE_CURRENT_LIMIT);
        elbowMotor.setSecondaryCurrentLimit(ArmConstants.SECONDARY_CURRENT_LIMIT);
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

        shoulderMotor1 = new CANSparkMax(ArmConstants.SHOULDER_1_PORT, MotorType.kBrushless);
        shoulderMotor1.restoreFactoryDefaults();
        shoulderMotor1.setInverted(false);
        shoulderMotor1.setIdleMode(IdleMode.kBrake);
        shoulderMotor1.setSmartCurrentLimit(ArmConstants.STALL_CURRENT_LIMIT, ArmConstants.FREE_CURRENT_LIMIT);
        shoulderMotor1.setSecondaryCurrentLimit(ArmConstants.SECONDARY_CURRENT_LIMIT);
        shoulderEncoder = shoulderMotor1.getAbsoluteEncoder(Type.kDutyCycle);
        shoulderEncoder.setPositionConversionFactor(360); // degrees
        shoulderEncoder.setZeroOffset(ArmConstants.SHOULDER_OFFSET);
        shoulderPID = shoulderMotor1.getPIDController();
        shoulderPID.setFeedbackDevice(shoulderEncoder);
        shoulderPID.setPositionPIDWrappingEnabled(true);
        shoulderPID.setPositionPIDWrappingMinInput(0);
        shoulderPID.setPositionPIDWrappingMaxInput(360);
        shoulderPID.setP(ArmConstants.SHOULDER_P);
        shoulderPID.setI(ArmConstants.SHOULDER_I);
        shoulderPID.setD(ArmConstants.SHOULDER_D);
        shoulderMotor1.burnFlash();

        shoulderMotor2 = new CANSparkMax(ArmConstants.SHOULDER_2_PORT, MotorType.kBrushless);
        shoulderMotor2.restoreFactoryDefaults();
        shoulderMotor2.setInverted(false);
        shoulderMotor2.setIdleMode(IdleMode.kBrake);
        shoulderMotor2.setSmartCurrentLimit(ArmConstants.STALL_CURRENT_LIMIT, ArmConstants.FREE_CURRENT_LIMIT);
        shoulderMotor2.setSecondaryCurrentLimit(ArmConstants.SECONDARY_CURRENT_LIMIT);
        shoulderMotor2.follow(shoulderMotor1);
        shoulderMotor2.burnFlash();
    }

    public void setElbowAngle(double angle) {
        elbowSetpoint = angle;
    }

    public double getElbowTargetAngle() {
        return elbowSetpoint;
    }

    public double getElbowAngle() {
        return elbowEncoder.getPosition();
    }

    public void setShoulderAngle(double angle) {
        shoulderSetpoint = angle;
    }

    public double getShoulderTargetAngle() {
        return shoulderSetpoint;
    }

    public double getShoulderAngle() {
        return shoulderEncoder.getPosition();
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
