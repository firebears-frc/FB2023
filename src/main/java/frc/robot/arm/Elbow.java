package frc.robot.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.sparkmax.ClosedLoopConfiguration;
import frc.robot.util.sparkmax.CurrentLimitConfiguration;
import frc.robot.util.sparkmax.FeedbackConfiguration;
import frc.robot.util.sparkmax.SparkMaxConfiguration;
import frc.robot.util.sparkmax.StatusFrameConfiguration;

public class Elbow extends Ligament {
    private static final class Constants {
        public static final int PORT = 7;

        public static final SparkMaxConfiguration CONFIG = new SparkMaxConfiguration(
                true,
                IdleMode.kBrake,
                CurrentLimitConfiguration.complex(40, 20, 10, 45.0),
                StatusFrameConfiguration.absoluteEncoder(),
                ClosedLoopConfiguration.wrapping(0.01, 0.0, 0.005, 0.0, 0, 360),
                FeedbackConfiguration.absoluteEncoder(true, 360));
    }

    private final CANSparkMax motor;
    private final SparkMaxAbsoluteEncoder encoder;
    private final SparkMaxPIDController pid;

    public Elbow() {
        motor = new CANSparkMax(Constants.PORT, MotorType.kBrushless);
        encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
        pid = motor.getPIDController();

        Constants.CONFIG.apply(motor);

        name = "Elbow";
    }

    public void periodic() {
        position = Rotation2d.fromDegrees(encoder.getPosition());
        pid.setReference(setpoint.getDegrees(), ControlType.kPosition);
    }
}
