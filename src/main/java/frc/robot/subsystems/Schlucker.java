package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.SchluckerConstants;

public class Schlucker extends SubsystemBase {
    public enum HeldItem {
        NONE,
        CUBE,
        CONE
    }

    private final CANSparkMax motor;
    private final SparkMaxPIDController pid;
    private HeldItem item = HeldItem.NONE;

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
        item = HeldItem.CONE;
    }

    public void intakeCube() {
        pid.setReference(-1.0 * SchluckerConstants.INTAKE_SPEED, ControlType.kDutyCycle);
        item = HeldItem.CUBE;
    }

    public void hold() {
        switch (item) {
            case CONE:
                pid.setReference(-1.0 * SchluckerConstants.HOLD_CURRENT, ControlType.kCurrent);
                break;
            case CUBE:
            default:
                pid.setReference(SchluckerConstants.HOLD_CURRENT, ControlType.kCurrent);
                break;
        }
    }

    public void eject() {
        switch (item) {
            case CONE:
                motor.set(SchluckerConstants.INTAKE_SPEED);
                break;
            case CUBE:
            default:
                motor.set(-1.0 * SchluckerConstants.INTAKE_SPEED);
                break;
        }
        item = HeldItem.NONE;
    }

    public void stop() {
        pid.setReference(0, ControlType.kDutyCycle);
    }
}
