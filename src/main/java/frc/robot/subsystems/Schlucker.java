package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Schlucker extends SubsystemBase {

    private CANSparkMax shluckerMotor;

    public Schlucker() {
        shluckerMotor = new CANSparkMax(5, MotorType.kBrushed);

        shluckerMotor.restoreFactoryDefaults();
        shluckerMotor.setInverted(false);
        shluckerMotor.setIdleMode(IdleMode.kCoast);

    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }

}
