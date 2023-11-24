package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

public class IntakeNeo550 extends Intake {
    public static class Constants {
        public static final int STALL_CURRENT_LIMIT = 20;
        public static final int FREE_CURRENT_LIMIT = 20;
        public static final double SECONDARY_CURRENT_LIMIT = 25.0;

        public static final double INTAKE_SPEED = 0.01; // rotations per cycle

        public static final double P = 3.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
    }

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkMaxPIDController pid;

    public IntakeNeo550() {
        Logger.recordMetadata("Intake/Type", "Neo550");

        motor = new CANSparkMax(Intake.Constants.MOTOR_CAN_ID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setInverted(false);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(Constants.STALL_CURRENT_LIMIT, Constants.FREE_CURRENT_LIMIT);
        motor.setSecondaryCurrentLimit(Constants.SECONDARY_CURRENT_LIMIT);
        encoder = motor.getEncoder();
        pid = motor.getPIDController();
        pid.setP(Constants.P);
        pid.setI(Constants.I);
        pid.setD(Constants.D);

        motor.burnFlash();

        // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);
    }

    @Override
    public void periodic() {
        super.periodic();

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
        double position = encoder.getPosition();
        position += speed;
        pid.setReference(position, ControlType.kPosition);

        Logger.recordOutput("Intake/Speed", speed);
        Logger.recordOutput("Intake/Position", position);
    }
}
