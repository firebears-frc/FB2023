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
import frc.robot.util.sparkmax.FollowingConfiguration;
import frc.robot.util.sparkmax.SparkMaxConfiguration;
import frc.robot.util.sparkmax.StatusFrameConfiguration;

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
    private final SparkMaxAbsoluteEncoder encoder;
    private final SparkMaxPIDController pid;

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
