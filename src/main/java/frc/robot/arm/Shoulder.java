package frc.robot.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.sparkmax.ComplexCurrentLimitConfiguration;
import frc.robot.util.sparkmax.SparkMaxConfiguration;
import frc.robot.util.sparkmax.StatusFrameConfiguration;

public class Shoulder extends Ligament {
    private static final class Constants {
        public static final int RIGHT_CAN_ID = 8;
        public static final int LEFT_CAN_ID = 9;

        public static final SparkMaxConfiguration CONFIG_RIGHT = new SparkMaxConfiguration(
                true,
                IdleMode.kBrake,
                new ComplexCurrentLimitConfiguration(30, 20, 10, 35.0),
                StatusFrameConfiguration.leadingAbsoluteEncoder());
        public static final SparkMaxConfiguration CONFIG_LEFT = new SparkMaxConfiguration(
                false,
                IdleMode.kBrake,
                new ComplexCurrentLimitConfiguration(30, 20, 10, 35.0),
                StatusFrameConfiguration.normal());

        public static final double P = 0.0175;
        public static final double I = 0.0;
        public static final double D = 0.005;
    }

    private final CANSparkMax motorRight;
    private final CANSparkMax motorLeft;
    private final SparkMaxAbsoluteEncoder encoder;
    private final SparkMaxPIDController pid;

    public Shoulder() {
        motorRight = new CANSparkMax(Constants.RIGHT_CAN_ID, MotorType.kBrushless);
        Constants.CONFIG_RIGHT.apply(motorRight);
        encoder = motorRight.getAbsoluteEncoder(Type.kDutyCycle);
        encoder.setPositionConversionFactor(360); // degrees
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
        Constants.CONFIG_LEFT.apply(motorLeft);
        motorLeft.follow(motorRight, true);

        motorRight.burnFlash();
        motorLeft.burnFlash();

        name = "Shoulder";
    }

    public void periodic() {
        position = Rotation2d.fromDegrees(encoder.getPosition());
        pid.setReference(setpoint.getDegrees(), ControlType.kPosition);
    }
}
