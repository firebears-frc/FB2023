package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.SchluckerConstants;

public class Schlucker extends SubsystemBase {
    private CANSparkMax motor;

    public Schlucker() {
        motor = new CANSparkMax(SchluckerConstants.MOTOR_PORT, MotorType.kBrushed);
        motor.restoreFactoryDefaults();
        motor.setInverted(false);
        motor.setIdleMode(IdleMode.kCoast);
        motor.setSmartCurrentLimit(SchluckerConstants.STALL_CURRENT_LIMIT, SchluckerConstants.FREE_CURRENT_LIMIT);
        motor.setSecondaryCurrentLimit(SchluckerConstants.SECONDARY_CURRENT_LIMIT);
        motor.burnFlash();
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }
}
