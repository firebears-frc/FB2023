package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SchluckerBag extends Schlucker {
    public static class Constants {
        public static final int STALL_CURRENT_LIMIT = 10;
        public static final int FREE_CURRENT_LIMIT = 10;
        public static final double SECONDARY_CURRENT_LIMIT = 20.0;

        public static final double INTAKE_SPEED = 0.7;
        public static final double HOLD_SPEED = 0.3;
    }

    private final CANSparkMax motor;

    public SchluckerBag() {
        Logger.getInstance().recordMetadata("Schlucker/Type", "Bag");
        motor = new CANSparkMax(Schlucker.Constants.MOTOR_CAN_ID, MotorType.kBrushed);
        motor.restoreFactoryDefaults();
        motor.setInverted(false);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(Constants.STALL_CURRENT_LIMIT, Constants.FREE_CURRENT_LIMIT);
        motor.setSecondaryCurrentLimit(Constants.SECONDARY_CURRENT_LIMIT);

        motor.burnFlash();
    }

    @Override
    public void periodic() {
        super.periodic();

        // Figure out what speed we should be running
        double speed;
        switch (state) {
            case INTAKE:
                switch (itemHeld) {
                    case CUBE:
                        speed = Constants.INTAKE_SPEED;
                        break;
                    case CONE:
                    case NONE:
                    default:
                        speed = -1.0 * Constants.INTAKE_SPEED;
                        break;
                }
                break;

            case EJECT:
                switch (itemHeld) {
                    case CUBE:
                        speed = -1.0 * Constants.INTAKE_SPEED;
                        break;
                    case CONE:
                        speed = Constants.INTAKE_SPEED;
                        break;
                    case NONE:
                    default:
                        switch (lastItemHeld) {
                            case CUBE:
                                speed = -1.0 * Constants.INTAKE_SPEED;
                                break;
                            case CONE:
                            case NONE:
                            default:
                                speed = Constants.INTAKE_SPEED;
                                break;
                        }
                }
                break;

            case HOLD:
                switch (itemHeld) {
                    case CUBE:
                        speed = Constants.HOLD_SPEED;
                        break;
                    case CONE:
                    case NONE:
                    default:
                        speed = -1.0 * Constants.HOLD_SPEED;
                        break;
                }
                break;

            case STOP:
            default:
                speed = 0;
                break;
        }

        motor.set(speed);
        Logger.getInstance().recordOutput("Schlucker/Speed", speed);
    }
}
