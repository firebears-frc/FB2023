package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;

public class SchluckerNeo550 extends Schlucker {
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
    private final DoubleLogEntry speedLog;
    private final DoubleLogEntry positionLog;

    public SchluckerNeo550(DataLog log) {
        super(log);
        typeLog.append("Neo550");

        motor = new CANSparkMax(Schlucker.Constants.MOTOR_PORT, MotorType.kBrushless);
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

        speedLog = new DoubleLogEntry(log, "Schlucker/Speed");
        positionLog = new DoubleLogEntry(log, "Schlucker/Position");

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
                        speed = -1.0 * Constants.INTAKE_SPEED;
                        break;
                    case CONE:
                    case NONE:
                    default:
                        speed = Constants.INTAKE_SPEED;
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
            case STOP:
            default:
                speed = 0;
                break;
        }

        // Update the position controller
        double position = encoder.getPosition();
        position += speed;
        pid.setReference(position, ControlType.kPosition);

        speedLog.append(speed);
        positionLog.append(position);
    }
}
