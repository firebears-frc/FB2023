package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmShoulder {
    private static class Constants {
        public static final int RIGHT_CAN_ID = 12;
        public static final int LEFT_CAN_ID = 13;

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

    private final CANSparkMax motorRight;
    private final CANSparkMax motorLeft;
    private final SparkMaxAbsoluteEncoder encoder;
    private final SparkMaxPIDController pid;
    private Rotation2d setpoint;
    private Rotation2d position;

    public ArmShoulder() {
        motorRight = new CANSparkMax(Constants.RIGHT_CAN_ID, MotorType.kBrushless);
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

        motorLeft = new CANSparkMax(Constants.LEFT_CAN_ID, MotorType.kBrushless);
        motorLeft.restoreFactoryDefaults();
        motorLeft.setInverted(false);
        motorLeft.setIdleMode(IdleMode.kBrake);
        motorLeft.setSmartCurrentLimit(Constants.STALL_CURRENT_LIMIT, Constants.FREE_CURRENT_LIMIT);
        motorLeft.setSecondaryCurrentLimit(Constants.SECONDARY_CURRENT_LIMIT);
        motorLeft.follow(motorRight, true);

        motorRight.burnFlash();
        motorLeft.burnFlash();
    }

    public void setAngle(Rotation2d angle) {
        setpoint = angle;
    }

    public Rotation2d getTargetAngle() {
        return setpoint;
    }

    public Rotation2d getAngle() {
        return position;
    }

    public Rotation2d getError() {
        return getAngle().minus(getTargetAngle());
    }

    public void periodic() {
        position = Rotation2d.fromDegrees(encoder.getPosition());
        pid.setReference(setpoint.getDegrees(), ControlType.kPosition);

        Logger logger = Logger.getInstance();
        logger.recordOutput("Arm/Shoulder/Setpoint", setpoint.getDegrees());
        logger.recordOutput("Arm/Shoulder/Position", position.getDegrees());
    }
}
