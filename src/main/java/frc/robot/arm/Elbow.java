package frc.robot.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.spark.ClosedLoopConfiguration;
import frc.robot.util.spark.CurrentLimitConfiguration;
import frc.robot.util.spark.FeedbackConfiguration;
import frc.robot.util.spark.SparkConfiguration;
import frc.robot.util.spark.StatusFrameConfiguration;

public class Elbow extends Ligament {
    private static final class Constants {
        public static final int PORT = 7;

        public static final SparkConfiguration CONFIG = new SparkConfiguration(
                true,
                IdleMode.kBrake,
                CurrentLimitConfiguration.complex(40, 20, 10, 45.0),
                StatusFrameConfiguration.absoluteEncoder(),
                ClosedLoopConfiguration.wrapping(0.01, 0.0, 0.005, 0.0, 0, 360),
                FeedbackConfiguration.absoluteEncoder(true, 360));
    }

    private final CANSparkMax motor;
    private final SparkAbsoluteEncoder encoder;
    private final SparkPIDController pid;

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
