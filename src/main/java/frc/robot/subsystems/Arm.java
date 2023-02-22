package frc.robot.subsystems;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Arm extends SubsystemBase {

    private CANSparkMax elbowMotor;
    private CANSparkMax shoulderMotor1;
    private CANSparkMax shoulderMotor2;
    private CANSparkMax shulkerMotor;

    public Arm() {
        elbowMotor = new CANSparkMax(7, MotorType.kBrushless);

        elbowMotor.restoreFactoryDefaults();
        elbowMotor.setInverted(false);
        elbowMotor.setIdleMode(IdleMode.kBrake);

        shoulderMotor1 = new CANSparkMax(12, MotorType.kBrushless);

        shoulderMotor1.restoreFactoryDefaults();
        shoulderMotor1.setInverted(false);
        shoulderMotor1.setIdleMode(IdleMode.kBrake);

        shoulderMotor2 = new CANSparkMax(13, MotorType.kBrushless);

        shoulderMotor2.restoreFactoryDefaults();
        shoulderMotor2.setInverted(false);
        shoulderMotor2.setIdleMode(IdleMode.kBrake);

        shulkerMotor = new CANSparkMax(6, MotorType.kBrushed);

        shulkerMotor.restoreFactoryDefaults();
        shulkerMotor.setInverted(false);
        shulkerMotor.setIdleMode(IdleMode.kBrake);


    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }

}
