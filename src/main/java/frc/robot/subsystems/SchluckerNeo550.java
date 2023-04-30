package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SchluckerNeo550 extends Schlucker {
    public static class SchluckerNeo550Constants {
        public static final int STALL_CURRENT_LIMIT = 20;
        public static final int FREE_CURRENT_LIMIT = 20;
        public static final double SECONDARY_CURRENT_LIMIT = 25.0;

        public static final double INTAKE_SPEED = 0.01; // rotations per cycle

        public static final double P = 1.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
    }

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkMaxPIDController pid;

    public SchluckerNeo550() {
        motor = new CANSparkMax(SchluckerConstants.MOTOR_PORT, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setInverted(false);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(SchluckerNeo550Constants.STALL_CURRENT_LIMIT, SchluckerNeo550Constants.FREE_CURRENT_LIMIT);
        motor.setSecondaryCurrentLimit(SchluckerNeo550Constants.SECONDARY_CURRENT_LIMIT);
        encoder = motor.getEncoder();
        pid = motor.getPIDController();
        pid.setP(SchluckerNeo550Constants.P);
        pid.setI(SchluckerNeo550Constants.I);
        pid.setD(SchluckerNeo550Constants.D);
        motor.burnFlash();
    }

    @Override
    public void periodic() {
        // Figure out what speed we should be running
        double speed;
        switch (state) {
            case INTAKE:
                switch (itemHeld) {
                    case CUBE:
                        speed = -1.0 * SchluckerNeo550Constants.INTAKE_SPEED;
                        break;
                    case CONE:
                    case NONE:
                    default:
                        speed = SchluckerNeo550Constants.INTAKE_SPEED;
                        break;
                }
                break;

            case EJECT:
                switch (itemHeld) {
                    case CUBE:
                        speed = -1.0 * SchluckerNeo550Constants.INTAKE_SPEED;
                        break;
                    case CONE:
                        speed = SchluckerNeo550Constants.INTAKE_SPEED;
                        break;
                    case NONE:
                    default:
                        switch (lastItemHeld) {
                            case CUBE:
                                speed = -1.0 * SchluckerNeo550Constants.INTAKE_SPEED;
                                break;
                            case CONE:
                            case NONE:
                            default:
                                speed = SchluckerNeo550Constants.INTAKE_SPEED;
                                break;
                        }
                }
                break;

            case HOLD:
            case STOP:
            default:
                speed = 0;
                break;
        }

        // Update the position controller
        double position = encoder.getPosition();
        position += speed;
        pid.setReference(position, ControlType.kPosition);
    }
}
