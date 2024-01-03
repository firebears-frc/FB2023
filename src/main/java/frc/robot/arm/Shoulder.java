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
import frc.robot.util.spark.FollowingConfiguration;
import frc.robot.util.spark.SparkMaxConfiguration;
import frc.robot.util.spark.StatusFrameConfiguration;

public class Shoulder extends Ligament {
    private static final class Constants {
        public static final int RIGHT_CAN_ID = 8;
        public static final int LEFT_CAN_ID = 9;

        public static final SparkMaxConfiguration CONFIG_RIGHT = new SparkMaxConfiguration(
                true,
                IdleMode.kBrake,
                CurrentLimitConfiguration.complex(30, 20, 10, 35.0),
                StatusFrameConfiguration.leadingAbsoluteEncoder(),
                ClosedLoopConfiguration.wrapping(0.0175, 0.0, 0.005, 0.0, 0, 360),
                FeedbackConfiguration.absoluteEncoder(true, 360));
        public static final SparkMaxConfiguration CONFIG_LEFT = new SparkMaxConfiguration(
                false,
                IdleMode.kBrake,
                CurrentLimitConfiguration.complex(30, 20, 10, 35.0),
                StatusFrameConfiguration.normal(),
                FollowingConfiguration.sparkMax(RIGHT_CAN_ID, true));
    }

    private final CANSparkMax motorRight;
    private final CANSparkMax motorLeft;
    private final SparkAbsoluteEncoder encoder;
    private final SparkPIDController pid;

    public Shoulder() {
        motorRight = new CANSparkMax(Constants.RIGHT_CAN_ID, MotorType.kBrushless);
        motorLeft = new CANSparkMax(Constants.LEFT_CAN_ID, MotorType.kBrushless);
        encoder = motorRight.getAbsoluteEncoder(Type.kDutyCycle);
        pid = motorRight.getPIDController();

        Constants.CONFIG_RIGHT.apply(motorRight);
        Constants.CONFIG_LEFT.apply(motorLeft);

        name = "Shoulder";
    }

    public void periodic() {
        position = Rotation2d.fromDegrees(encoder.getPosition());
        pid.setReference(setpoint.getDegrees(), ControlType.kPosition);
    }
}
