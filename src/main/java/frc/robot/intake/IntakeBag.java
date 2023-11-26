package frc.robot.intake;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

public class IntakeBag extends Intake {
    public static class Constants {
        public static final int STALL_CURRENT_LIMIT = 10;
        public static final int FREE_CURRENT_LIMIT = 10;
        public static final double SECONDARY_CURRENT_LIMIT = 20.0;

        public static final double INTAKE_SPEED = 0.7;
        public static final double HOLD_SPEED = 0.3;
    }

    private final CANSparkMax motor;

    @AutoLogOutput(key = "Intake/Speed")
    private double speed;

    public IntakeBag() {
        motor = new CANSparkMax(Intake.Constants.MOTOR_CAN_ID, MotorType.kBrushed);
        motor.restoreFactoryDefaults();
        motor.setInverted(false);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(Constants.STALL_CURRENT_LIMIT, Constants.FREE_CURRENT_LIMIT);
        motor.setSecondaryCurrentLimit(Constants.SECONDARY_CURRENT_LIMIT);

        motor.burnFlash();

        // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

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
