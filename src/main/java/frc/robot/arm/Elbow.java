package frc.robot.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.sparkmax.ComplexCurrentLimitConfiguration;
import frc.robot.util.sparkmax.SparkMaxConfiguration;
import frc.robot.util.sparkmax.StatusFrameConfiguration;

public class Elbow extends Ligament {
    private static final class Constants {
        public static final int PORT = 7;

        public static SparkMaxConfiguration CONFIG = new SparkMaxConfiguration(
            true,
            IdleMode.kBrake,
            new ComplexCurrentLimitConfiguration(40, 20, 10, 45.0),
            StatusFrameConfiguration.absoluteEncoder()
        );

        public static final double P = 0.01;
        public static final double I = 0.0;
        public static final double D = 0.005;
    }

    private final CANSparkMax motor;
    private final SparkMaxAbsoluteEncoder encoder;
    private final SparkMaxPIDController pid;

    public Elbow() {
        motor = new CANSparkMax(Constants.PORT, MotorType.kBrushless);
        Constants.CONFIG.apply(motor);
        encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
        encoder.setPositionConversionFactor(360); // degrees
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

        name = "Elbow";
    }

    public void periodic() {
        position = Rotation2d.fromDegrees(encoder.getPosition());
        pid.setReference(setpoint.getDegrees(), ControlType.kPosition);
    }
}
