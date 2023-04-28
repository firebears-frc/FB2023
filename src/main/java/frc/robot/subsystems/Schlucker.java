package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.GamePiece;

public class Schlucker extends SubsystemBase {
    public static class SchluckerConstants {
        public static final int MOTOR_PORT = 6;

        public static final int STALL_CURRENT_LIMIT = 20;
        public static final int FREE_CURRENT_LIMIT = 20;
        public static final double SECONDARY_CURRENT_LIMIT = 25.0;

        public static final double INTAKE_SPEED = 0.01; // rotations per cycle

        public static final double P = 1.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
    }

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkMaxPIDController pid;
    private double speed = 0;
    private GamePiece itemHeld = GamePiece.NONE;
    private GamePiece lastItemHeld = GamePiece.NONE;
    private GamePiece itemWanted = GamePiece.NONE;

    public Schlucker() {
        motor = new CANSparkMax(SchluckerConstants.MOTOR_PORT, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setInverted(false);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(SchluckerConstants.STALL_CURRENT_LIMIT, SchluckerConstants.FREE_CURRENT_LIMIT);
        motor.setSecondaryCurrentLimit(SchluckerConstants.SECONDARY_CURRENT_LIMIT);
        encoder = motor.getEncoder();
        pid = motor.getPIDController();
        pid.setP(SchluckerConstants.P);
        pid.setI(SchluckerConstants.I);
        pid.setD(SchluckerConstants.D);
        motor.burnFlash();
    }

    public Command intakeCone() {
        return new InstantCommand(() -> {
            speed = SchluckerConstants.INTAKE_SPEED;
            itemHeld = GamePiece.CONE;
            lastItemHeld = GamePiece.CONE;
            itemWanted = GamePiece.NONE;
        }, this);
    }

    public Command intakeCube() {
        return new InstantCommand(() -> {
            speed = -1.0 * SchluckerConstants.INTAKE_SPEED;
            itemHeld = GamePiece.CUBE;
            lastItemHeld = GamePiece.CUBE;
            itemWanted = GamePiece.NONE;
        }, this);
    }

    public Command hold() {
        return new InstantCommand(() -> {
            speed = 0;
        }, this);
    }

    public Command eject() {
        return new InstantCommand(() -> {
            switch (itemHeld) {
                case CONE:
                    speed = SchluckerConstants.INTAKE_SPEED;
                    break;
                case CUBE:
                    speed = -1.0 * SchluckerConstants.INTAKE_SPEED;
                    break;
                case NONE:
                default:
                    switch (lastItemHeld) {
                        case CONE:
                            speed = SchluckerConstants.INTAKE_SPEED;
                            break;
                        case CUBE:
                            speed = -1.0 * SchluckerConstants.INTAKE_SPEED;
                            break;
                        case NONE:
                        default:
                            break;
                    }
            }
            itemHeld = GamePiece.NONE;
        }, this);
    }

    public Command stop() {
        return new InstantCommand(() -> {
            speed = 0;
        }, this);
    }

    public Command wantCone() {
        return new InstantCommand(() -> {
            itemWanted = GamePiece.CONE;
        }, this);
    }

    public Command wantCube() {
        return new InstantCommand(() -> {
            itemWanted = GamePiece.CUBE;
        }, this);
    }

    public GamePiece getHeldItem() {
        return itemHeld;
    }

    public GamePiece getWantedItem() {
        return itemWanted;
    }

    @Override
    public void periodic() {
        double position = encoder.getPosition();
        position += speed;
        pid.setReference(position, ControlType.kPosition);
    }
}
