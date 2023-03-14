package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GamePiece;
import frc.robot.util.Constants.SchluckerConstants;

public class Schlucker extends SubsystemBase {
    private final CANSparkMax motor;
    private final SparkMaxPIDController pid;
    private GamePiece itemHeld = GamePiece.NONE;
    private GamePiece itemWanted = GamePiece.NONE;

    public Schlucker() {
        motor = new CANSparkMax(SchluckerConstants.MOTOR_PORT, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setInverted(false);
        motor.setIdleMode(IdleMode.kCoast);
        motor.setSmartCurrentLimit(SchluckerConstants.STALL_CURRENT_LIMIT, SchluckerConstants.FREE_CURRENT_LIMIT);
        motor.setSecondaryCurrentLimit(SchluckerConstants.SECONDARY_CURRENT_LIMIT);
        pid = motor.getPIDController();
        motor.burnFlash();
    }

    public void intakeCone() {
        pid.setReference(SchluckerConstants.INTAKE_SPEED, ControlType.kDutyCycle);
        itemHeld = GamePiece.CONE;
        itemWanted = GamePiece.NONE;
    }

    public void intakeCube() {
        pid.setReference(-1.0 * SchluckerConstants.INTAKE_SPEED, ControlType.kDutyCycle);
        itemHeld = GamePiece.CUBE;
        itemWanted = GamePiece.NONE;
    }

    public void hold() {
        switch (itemHeld) {
            case CONE:
                pid.setReference(-1.0 * SchluckerConstants.HOLD_CURRENT, ControlType.kCurrent);
                break;
            case CUBE:
            case NONE:
                pid.setReference(SchluckerConstants.HOLD_CURRENT, ControlType.kCurrent);
                break;
        }
    }

    public void eject() {
        switch (itemHeld) {
            case CONE:
                motor.set(SchluckerConstants.INTAKE_SPEED);
                break;
            case CUBE:
            case NONE:
                motor.set(-1.0 * SchluckerConstants.INTAKE_SPEED);
                break;
        }
        itemHeld = GamePiece.NONE;
    }

    public void stop() {
        pid.setReference(0, ControlType.kDutyCycle);
    }

    public GamePiece getHeldItem() {
        return itemHeld;
    }

    public GamePiece getWantedItem() {
        return itemWanted;
    }
}
