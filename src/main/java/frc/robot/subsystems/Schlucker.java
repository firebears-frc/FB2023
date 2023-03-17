package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Schlucker extends SubsystemBase {

    private CANSparkMax shluckerMotor;
    private SparkMaxPIDController pid;

    private final double HOLD_POSITIVE_CURRENT = 0.3; 
    private final double HOLD_NEGATIVE_CURRENT = -HOLD_POSITIVE_CURRENT; 
    // variable to hold if an item was Ejected by the robot. variable needs to be used by lights
    public boolean ejectPushed;
    // should it be initially set to true or false? do we need to set it?
    // should it be private or public?

    public enum ItemHeld {
        CONE,
        CUBE,
        NONE  
    }

    private ItemHeld item_held;
    
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
        default:
            break;
        }
        ejectPushed = true;
    }

    public void hold() {
        switch(item_held) {
        case CONE:
            pid.setReference(HOLD_NEGATIVE_CURRENT, ControlType.kDutyCycle);
            break;
        case CUBE:
            pid.setReference(HOLD_POSITIVE_CURRENT, ControlType.kDutyCycle);
            break;
        default:
            break;
        }
    }

    public void stop() {
        pid.setReference(0, ControlType.kDutyCycle);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shluckerOutput", shluckerMotor.getAppliedOutput());
    }

    @Override
    public void simulationPeriodic() {

    }
}
