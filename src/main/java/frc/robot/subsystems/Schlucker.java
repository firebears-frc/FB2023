package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GamePiece;

public class Schlucker extends SubsystemBase {
    public static class SchluckerConstants {
        public static final int MOTOR_PORT = 6;

        public static final int STALL_CURRENT_LIMIT = 20;
        public static final int FREE_CURRENT_LIMIT = 5;
        public static final double SECONDARY_CURRENT_LIMIT = 30.0;

        public static final double INTAKE_SPEED = 0.7;
        public static final double HOLD_CURRENT = 1.0;

        public static final double P = 1.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
    }

    private final CANSparkMax motor;
    private final SparkMaxPIDController pid;
    private GamePiece itemHeld = GamePiece.NONE;
    private GamePiece lastItemHeld = GamePiece.NONE;
    private GamePiece itemWanted = GamePiece.NONE;

    public Schlucker() {
        motor = new CANSparkMax(SchluckerConstants.MOTOR_PORT, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setInverted(false);
        motor.setIdleMode(IdleMode.kCoast);
        motor.setSmartCurrentLimit(SchluckerConstants.STALL_CURRENT_LIMIT, SchluckerConstants.FREE_CURRENT_LIMIT);
        motor.setSecondaryCurrentLimit(SchluckerConstants.SECONDARY_CURRENT_LIMIT);
        pid = motor.getPIDController();
        pid.setP(SchluckerConstants.P);
        pid.setI(SchluckerConstants.I);
        pid.setD(SchluckerConstants.D);
        motor.burnFlash();
    }

    public void intakeCone() {
        pid.setReference(SchluckerConstants.INTAKE_SPEED, ControlType.kDutyCycle);
        itemHeld = GamePiece.CONE;
        lastItemHeld = GamePiece.CONE;
        itemWanted = GamePiece.NONE;
    }

    public void intakeCube() {
        pid.setReference(-1.0 * SchluckerConstants.INTAKE_SPEED, ControlType.kDutyCycle);
        itemHeld = GamePiece.CUBE;
        lastItemHeld = GamePiece.CUBE;
        itemWanted = GamePiece.NONE;
    }

    public void hold() {
        switch (itemHeld) {
            case CONE:
                pid.setReference(-1.0 * SchluckerConstants.HOLD_CURRENT, ControlType.kCurrent);
                break;
            case CUBE:
                pid.setReference(SchluckerConstants.HOLD_CURRENT, ControlType.kCurrent);
                break;
            case NONE:
            default:
                break;
        }
    }

    public void eject() {
        switch (itemHeld) {
            case CONE:
                motor.set(SchluckerConstants.INTAKE_SPEED);
                break;
            case CUBE:
                motor.set(-1.0 * SchluckerConstants.INTAKE_SPEED);
                break;
            case NONE:
            default:
                switch (lastItemHeld) {
                    case CONE:
                        motor.set(SchluckerConstants.INTAKE_SPEED);
                        break;
                    case CUBE:
                        motor.set(-1.0 * SchluckerConstants.INTAKE_SPEED);
                        break;
                    case NONE:
                    default:
                        break;
                }
        }
        itemHeld = GamePiece.NONE;
    }

    public void stop() {
        pid.setReference(0, ControlType.kDutyCycle);
    }

    public GamePiece getHeldItem() {
        return itemHeld;
    }

    public void wantCone() {
        itemWanted = GamePiece.CONE;
    }

    public void wantCube() {
        itemWanted = GamePiece.CUBE;
    }

    public GamePiece getWantedItem() {
        return itemWanted;
    }
}
