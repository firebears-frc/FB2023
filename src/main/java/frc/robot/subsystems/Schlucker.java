package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LogEntryDouble;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.*;

public class Schlucker extends SubsystemBase {

    private CANSparkMax shluckerMotor;
    private SparkMaxPIDController pid;

    private final double HOLD_POSITIVE_PERCENT = 0.3;
    private final double HOLD_NEGATIVE_PERCENT = -HOLD_POSITIVE_PERCENT;
    // variable to hold if an item was Ejected by the robot. variable needs to be
    // used by lights
    // should it be initially set to true or false? do we need to set it?
    // should it be private or public?

    public enum ItemHeld {
        CONE,
        CUBE,
        NONE
    }

    private ItemHeld item_held = ItemHeld.NONE;
    private ItemHeld saved_item_held = ItemHeld.NONE;

    private LogEntryDouble motorLog;

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

        if (LOGGING) {
            motorLog = new LogEntryDouble(DataLogManager.getLog(), "/schlucker/motor");
        }
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
                pid.setReference(HOLD_NEGATIVE_PERCENT, ControlType.kDutyCycle);
                break;
            case CUBE:
                pid.setReference(HOLD_POSITIVE_PERCENT, ControlType.kDutyCycle);
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
        double motorOutput = shluckerMotor.getAppliedOutput();
        if (DEBUG) {
            SmartDashboard.putNumber("shlucker output", motorOutput);
        }
        if (LOGGING) {
            motorLog.append(motorOutput);
        }
    }

    @Override
    public void simulationPeriodic() {

    }
}
