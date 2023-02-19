package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DriverStation;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Chassis extends SubsystemBase {
    private static int STALL_CURRENT_LIMIT = 30;
    private static int FREE_CURRENT_LIMIT = 20;
    private static int SECONDARY_CURRENT_LIMIT = 60;

    private CANSparkMax rightFrontMotor;
    private CANSparkMax rightBackMotor;
    private MotorControllerGroup rightMotors;
    private CANSparkMax leftFrontMotor;
    private CANSparkMax leftBackMotor;
    private MotorControllerGroup leftMotors;
    private DifferentialDrive differentialDrive;
    private AHRS navX;

    public Chassis() {
        rightFrontMotor = new CANSparkMax(15, MotorType.kBrushless);

        rightFrontMotor.restoreFactoryDefaults();
        rightFrontMotor.setInverted(false);
        rightFrontMotor.setIdleMode(IdleMode.kCoast);
        rightFrontMotor.setSmartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT);
        rightFrontMotor.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT);
        rightFrontMotor.burnFlash();

        rightBackMotor = new CANSparkMax(17, MotorType.kBrushless);

        rightBackMotor.restoreFactoryDefaults();
        rightBackMotor.setInverted(false);
        rightBackMotor.setIdleMode(IdleMode.kCoast);
        rightBackMotor.setSmartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT);
        rightBackMotor.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT);
        rightBackMotor.burnFlash();

        rightMotors = new MotorControllerGroup(rightFrontMotor, rightBackMotor);
        addChild("rightMotors", rightMotors);

        leftFrontMotor = new CANSparkMax(16, MotorType.kBrushless);

        leftFrontMotor.restoreFactoryDefaults();
        leftFrontMotor.setInverted(true);
        leftFrontMotor.setIdleMode(IdleMode.kCoast);
        leftFrontMotor.setSmartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT);
        leftFrontMotor.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT);
        leftFrontMotor.burnFlash();

        leftBackMotor = new CANSparkMax(17, MotorType.kBrushless);

        leftBackMotor.restoreFactoryDefaults();
        leftBackMotor.setInverted(true);
        leftBackMotor.setIdleMode(IdleMode.kCoast);
        leftBackMotor.setSmartCurrentLimit(STALL_CURRENT_LIMIT, FREE_CURRENT_LIMIT);
        leftBackMotor.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT);
        leftBackMotor.burnFlash();

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
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
