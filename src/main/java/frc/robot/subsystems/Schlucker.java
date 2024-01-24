package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.*;

public class Schlucker extends SubsystemBase {
    private CANSparkMax shluckerMotor;
    private SparkPIDController pid;

    public enum ItemHeld {
        CONE,
        CUBE,
        NONE
    }

    private ItemHeld item_held = ItemHeld.NONE;
    private ItemHeld saved_item_held = ItemHeld.NONE;

    public Schlucker() {
        shluckerMotor = new CANSparkMax(6, MotorType.kBrushed);
        shluckerMotor.setSmartCurrentLimit(10, 10);
        shluckerMotor.setSecondaryCurrentLimit(20);
        shluckerMotor.restoreFactoryDefaults();
        shluckerMotor.setInverted(false);
        shluckerMotor.setIdleMode(IdleMode.kBrake);
        pid = shluckerMotor.getPIDController();

        // set p value of pid to 1
        pid.setP(1.0);
        pid.setI(0);
        pid.setD(0);
        shluckerMotor.burnFlash();
    }

    public void intakeCone() {
        pid.setReference(-0.7, ControlType.kDutyCycle);
        item_held = ItemHeld.CONE;
        saved_item_held = item_held;
    }

    public void intakeCube() {
        pid.setReference(0.7, ControlType.kDutyCycle);
        item_held = ItemHeld.CUBE;
        saved_item_held = item_held;
    }

    public void eject() {
        switch (saved_item_held) {
            case CONE:
                pid.setReference(0.7, ControlType.kDutyCycle);
                break;
            case CUBE:
                pid.setReference(-0.7, ControlType.kDutyCycle);
                break;
            default:
                break;
        }
        item_held = ItemHeld.NONE;
    }

    public void hold() {
        switch (item_held) {
            case CONE:
                pid.setReference(-SCHLUCKER_HOLD_PERCENT, ControlType.kDutyCycle);
                break;
            case CUBE:
                pid.setReference(SCHLUCKER_HOLD_PERCENT, ControlType.kDutyCycle);
                break;
            default:
                break;
        }
    }

    public void stop() {
        pid.setReference(0, ControlType.kDutyCycle);
    }

    public ItemHeld getHeldPiece() {
        return item_held;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shlucker output", shluckerMotor.getAppliedOutput());
    }

    @Override
    public void simulationPeriodic() {

    }
}
