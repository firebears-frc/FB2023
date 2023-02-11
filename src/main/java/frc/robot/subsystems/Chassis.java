package frc.robot.subsystems;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Chassis extends SubsystemBase {

    private CANSparkMax rightFrontMotor;
    private CANSparkMax rightBackMotor;
    private MotorControllerGroup rightMotors;
    private CANSparkMax leftFrontMotor;
    private CANSparkMax leftBackMotor;
    private MotorControllerGroup leftMotors;
    private DifferentialDrive differentialDrive;
    private AHRS navX;

    public Chassis() {
        rightFrontMotor = new CANSparkMax(1, MotorType.kBrushless);

        rightFrontMotor.restoreFactoryDefaults();
        rightFrontMotor.setInverted(false);
        rightFrontMotor.setIdleMode(IdleMode.kCoast);

        rightBackMotor = new CANSparkMax(0, MotorType.kBrushless);

        rightBackMotor.restoreFactoryDefaults();
        rightBackMotor.setInverted(false);
        rightBackMotor.setIdleMode(IdleMode.kCoast);

        rightMotors = new MotorControllerGroup(rightFrontMotor, rightBackMotor);
        addChild("rightMotors", rightMotors);

        leftFrontMotor = new CANSparkMax(18, MotorType.kBrushless);

        leftFrontMotor.restoreFactoryDefaults();
        leftFrontMotor.setInverted(true);
        leftFrontMotor.setIdleMode(IdleMode.kCoast);

        leftBackMotor = new CANSparkMax(19, MotorType.kBrushless);

        leftBackMotor.restoreFactoryDefaults();
        leftBackMotor.setInverted(true);
        leftBackMotor.setIdleMode(IdleMode.kCoast);

        leftMotors = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
        addChild("leftMotors", leftMotors);

        differentialDrive = new DifferentialDrive(leftMotors, rightMotors);
        addChild("differentialDrive", differentialDrive);
        differentialDrive.setSafetyEnabled(true);
        differentialDrive.setExpiration(0.1);
        differentialDrive.setMaxOutput(1.0);

        try {
            navX = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            DriverStation.reportError(ex.getMessage(), true);
        }
        Timer.delay(1.0);
        // LiveWindow.addSensor("Chassis", "navX", navX);

    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }

    public void arcadeDrive(double speed, double rotation) {
        differentialDrive.arcadeDrive(speed, rotation);
    }

}
