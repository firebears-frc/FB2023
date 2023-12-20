package frc.robot.intake;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.util.sparkmax.CurrentLimitConfiguration;
import frc.robot.util.sparkmax.SparkMaxConfiguration;
import frc.robot.util.sparkmax.StatusFrameConfiguration;

public class IntakeBag extends Intake {
    public static final class Constants {
        public static final SparkMaxConfiguration CONFIG = new SparkMaxConfiguration(
                false,
                IdleMode.kBrake,
                CurrentLimitConfiguration.complex(10, 5, 10, 20.0),
                StatusFrameConfiguration.normal());

        public static final double INTAKE_SPEED = 0.7;
        public static final double HOLD_SPEED = 0.3;
    }

    private final CANSparkMax motor;

    @AutoLogOutput(key = "Intake/Speed")
    private double speed;

    public IntakeBag() {
        motor = new CANSparkMax(Intake.Constants.MOTOR_CAN_ID, MotorType.kBrushed);
        Constants.CONFIG.apply(motor);

        speed = 0;
    }

    @Override
    public void periodic() {
        // Figure out what speed we should be running
        double speed = switch (state) {
            case INTAKE -> switch (itemHeld) {
                case CUBE -> Constants.INTAKE_SPEED;
                case CONE, NONE -> -1.0 * Constants.INTAKE_SPEED;
            };

            case EJECT -> switch (itemHeld) {
                case CUBE -> -1.0 * Constants.INTAKE_SPEED;
                case CONE -> Constants.INTAKE_SPEED;
                case NONE -> switch (lastItemHeld) {
                    case CUBE -> -1.0 * Constants.INTAKE_SPEED;
                    case CONE, NONE -> Constants.INTAKE_SPEED;
                };
            };

            case HOLD -> switch (itemHeld) {
                case CUBE -> Constants.HOLD_SPEED;
                case CONE, NONE -> -1.0 * Constants.HOLD_SPEED;
            };

            case STOP -> 0;
        };

        motor.set(speed);
    }
}
