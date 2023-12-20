package frc.robot.intake;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.util.sparkmax.ClosedLoopConfiguration;
import frc.robot.util.sparkmax.CurrentLimitConfiguration;
import frc.robot.util.sparkmax.FeedbackConfiguration;
import frc.robot.util.sparkmax.SparkMaxConfiguration;
import frc.robot.util.sparkmax.StatusFrameConfiguration;

public class IntakeNeo550 extends Intake {
    public static final class Constants {
        public static final SparkMaxConfiguration CONFIG = new SparkMaxConfiguration(
                false,
                IdleMode.kBrake,
                CurrentLimitConfiguration.complex(20, 10, 10, 25.0),
                StatusFrameConfiguration.normal(),
                ClosedLoopConfiguration.simple(3.0, 0.0, 0.0, 0.0),
                FeedbackConfiguration.relativeEncoder(false, 1));

        public static final double INTAKE_SPEED = 0.01; // rotations per cycle
    }

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkMaxPIDController pid;

    @AutoLogOutput(key = "Intake/Speed")
    private double speed;
    @AutoLogOutput(key = "Intake/Actual")
    private double actual;
    @AutoLogOutput(key = "Intake/Target")
    private double target;

    public IntakeNeo550() {
        motor = new CANSparkMax(Intake.Constants.MOTOR_CAN_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();
        pid = motor.getPIDController();

        Constants.CONFIG.apply(motor);

        motor.burnFlash();
    }

    @Override
    public void periodic() {
        // Figure out what speed we should be running
        double speed = switch (state) {
            case INTAKE -> switch (itemHeld) {
                case CUBE -> -1.0 * Constants.INTAKE_SPEED;
                case CONE, NONE -> Constants.INTAKE_SPEED;
            };

            case EJECT -> switch (itemHeld) {
                case CUBE -> Constants.INTAKE_SPEED;
                case CONE -> -1.0 * Constants.INTAKE_SPEED;
                case NONE -> switch (lastItemHeld) {
                    case CUBE -> Constants.INTAKE_SPEED;
                    case CONE, NONE -> -1.0 * Constants.INTAKE_SPEED;
                };
            };

            case HOLD, STOP -> 0;
        };

        // Update the position controller
        actual = encoder.getPosition();
        target = speed + actual;
        pid.setReference(target, ControlType.kPosition);
    }
}
