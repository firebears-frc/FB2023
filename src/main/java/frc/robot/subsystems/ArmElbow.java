package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class ArmElbow {
    private static class Constants {
        public static final int PORT = 7;

        public static final int STALL_CURRENT_LIMIT = 40;
        public static final int FREE_CURRENT_LIMIT = 20;
        public static final double SECONDARY_CURRENT_LIMIT = 45.0;

        public static final double P = 0.01;
        public static final double I = 0.0;
        public static final double D = 0.005;

        public static final double OFFSET = 240.0; // degrees

        public static final double MAX_VELOCITY = 90.0; // degrees per second
        public static final double MAX_ACCELERATION = 90.0; // degrees per second squared
    }

    private final CANSparkMax motor;
    private final SparkMaxAbsoluteEncoder encoder;
    private final SparkMaxPIDController pid;
    private double setpoint;
    private double position;

    public ArmElbow() {
        motor = new CANSparkMax(Constants.PORT, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setInverted(true);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(Constants.STALL_CURRENT_LIMIT, Constants.FREE_CURRENT_LIMIT);
        motor.setSecondaryCurrentLimit(Constants.SECONDARY_CURRENT_LIMIT);
        encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
        encoder.setPositionConversionFactor(360); // degrees
        encoder.setZeroOffset(Constants.OFFSET);
        encoder.setInverted(true);
        pid = motor.getPIDController();
        pid.setFeedbackDevice(encoder);
        pid.setPositionPIDWrappingEnabled(true);
        pid.setPositionPIDWrappingMinInput(0);
        pid.setPositionPIDWrappingMaxInput(360);
        pid.setP(Constants.P, 0);
        pid.setI(Constants.I, 0);
        pid.setD(Constants.D, 0);

        motor.burnFlash();
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
        return position;
    }

    public double getError() {
        return getAngle() - getTargetAngle();
    }

    public void periodic() {
        position = encoder.getPosition();
        pid.setReference(setpoint, ControlType.kPosition);

        Logger logger = Logger.getInstance();
        logger.recordOutput("Arm/Elbow/Setpoint", setpoint);
        logger.recordOutput("Arm/Elbow/Position", position);
    }
}
