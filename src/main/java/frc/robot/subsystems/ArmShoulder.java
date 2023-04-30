package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class ArmShoulder {
    private static class Constants {
        public static final int RIGHT_PORT = 12;
        public static final int LEFT_PORT = 13;

        public static final int STALL_CURRENT_LIMIT = 30;
        public static final int FREE_CURRENT_LIMIT = 20;
        public static final double SECONDARY_CURRENT_LIMIT = 35.0;

        public static final double P = 0.0175;
        public static final double I = 0.0;
        public static final double D = 0.005;

        public static final double OFFSET = 196.0; // degrees

        public static final double MAX_VELOCITY = 90.0; // degrees per second
        public static final double MAX_ACCELERATION = 90.0; // degrees per second squared
    }

    private CANSparkMax motorRight;
    private CANSparkMax motorLeft;
    private SparkMaxAbsoluteEncoder encoder;
    private SparkMaxPIDController pid;
    private double setpoint;
    
    public ArmShoulder() {
        motorRight = new CANSparkMax(Constants.RIGHT_PORT, MotorType.kBrushless);
        motorRight.restoreFactoryDefaults();
        motorRight.setInverted(true);
        motorRight.setIdleMode(IdleMode.kBrake);
        motorRight.setSmartCurrentLimit(Constants.STALL_CURRENT_LIMIT, Constants.FREE_CURRENT_LIMIT);
        motorRight.setSecondaryCurrentLimit(Constants.SECONDARY_CURRENT_LIMIT);
        encoder = motorRight.getAbsoluteEncoder(Type.kDutyCycle);
        encoder.setPositionConversionFactor(360); // degrees
        encoder.setZeroOffset(Constants.OFFSET);
        encoder.setInverted(true);
        pid = motorRight.getPIDController();
        pid.setFeedbackDevice(encoder);
        pid.setPositionPIDWrappingEnabled(true);
        pid.setPositionPIDWrappingMinInput(0);
        pid.setPositionPIDWrappingMaxInput(360);
        pid.setP(Constants.P, 0);
        pid.setI(Constants.I, 0);
        pid.setD(Constants.D, 0);
        motorRight.burnFlash();

        motorLeft = new CANSparkMax(Constants.LEFT_PORT, MotorType.kBrushless);
        motorLeft.restoreFactoryDefaults();
        motorLeft.setInverted(false);
        motorLeft.setIdleMode(IdleMode.kBrake);
        motorLeft.setSmartCurrentLimit(Constants.STALL_CURRENT_LIMIT, Constants.FREE_CURRENT_LIMIT);
        motorLeft.setSecondaryCurrentLimit(Constants.SECONDARY_CURRENT_LIMIT);
        motorLeft.follow(motorRight, true);
        motorLeft.burnFlash();
    }

    public void setAngle(double angle) {
        while (angle > 360) {
            angle -= 360;
        }
        while (angle < 0) {
            angle += 360;
        }
        setpoint = angle;
    }

    public double getTargetAngle() {
        return setpoint;
    }

    public double getAngle() {
        return encoder.getPosition();
    }

    public double getError() {
        return getAngle() - getTargetAngle();
    }

    public void periodic() {
        pid.setReference(setpoint, ControlType.kPosition);
    }
}
