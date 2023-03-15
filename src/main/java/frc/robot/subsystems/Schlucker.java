package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Schlucker extends SubsystemBase {

    private CANSparkMax shluckerMotor;
    private SparkMaxPIDController pid;

    private final int HOLD_CURRENT = 5;
    public enum ItemHeld {
        CONE,
        CUBE,
        NONE  
    }

    private ItemHeld item_held;
    
    public Schlucker() {
        shluckerMotor = new CANSparkMax(6, MotorType.kBrushed);

        shluckerMotor.restoreFactoryDefaults();
        shluckerMotor.setInverted(false);
        shluckerMotor.setIdleMode(IdleMode.kBrake);
        pid = shluckerMotor.getPIDController();
    }


    public void intakeCone(){
        pid.setReference(-0.7, ControlType.kDutyCycle);
        item_held = ItemHeld.CONE;
    }

    public void intakeCube(){
        pid.setReference(0.7, ControlType.kDutyCycle);
        item_held = ItemHeld.CUBE;
    }

    public void eject(){
        switch(item_held) {
        case CONE:
            pid.setReference(0.7, ControlType.kDutyCycle);
            break;
        case CUBE:
            pid.setReference(-0.7, ControlType.kDutyCycle);
            break;
        }
    }

    public void hold() {
        switch(item_held) {
        case CONE:
            pid.setReference(-0.2, ControlType.kDutyCycle);
            break;
        case CUBE:
            pid.setReference(0.2, ControlType.kDutyCycle);
            break;
        }
    }

    public void stop() {
        pid.setReference(0, ControlType.kDutyCycle);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
