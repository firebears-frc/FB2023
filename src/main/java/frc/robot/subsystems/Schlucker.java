package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Schlucker extends SubsystemBase {

    private CANSparkMax shluckerMotor;

    public enum ItemHeld {
        CONE,
        CUBE,
        NONE  
    }

    // TODO: Define ItemHeld var (should it be public or private?)
    public ItemHeld _item_held;
    
    public Schlucker() {
        shluckerMotor = new CANSparkMax(6, MotorType.kBrushed);

        shluckerMotor.restoreFactoryDefaults();
        shluckerMotor.setInverted(false);
        shluckerMotor.setIdleMode(IdleMode.kBrake);

    }

    public void setShluckerSpeed(double speed) {
        shluckerMotor.set(speed);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }

}
