package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private static class ArmConstants {
        public static final int ELBOW_PORT = 7;
        public static final int ELBOW_STALL_CURRENT_LIMIT = 40;
        public static final int ELBOW_FREE_CURRENT_LIMIT = 20;
        public static final double ELBOW_SECONDARY_CURRENT_LIMIT = 45.0;
        public static final double ELBOW_MANUAL_SPEED = 1.0; // degrees per loop
        public static final double ELBOW_P = 0.01;
        public static final double ELBOW_I = 0.0;
        public static final double ELBOW_D = 0.005;
        public static final double ELBOW_OFFSET = 240.0; // degrees
        public static final double ELBOW_MAX_VELOCITY = 90.0; // degrees per second
        public static final double ELBOW_MAX_ACCELERATION = 90.0; // degrees per second squared

        public static final int SHOULDER_RIGHT_PORT = 12;
        public static final int SHOULDER_LEFT_PORT = 13;
        public static final int SHOULDER_STALL_CURRENT_LIMIT = 30;
        public static final int SHOULDER_FREE_CURRENT_LIMIT = 20;
        public static final double SHOULDER_SECONDARY_CURRENT_LIMIT = 35.0;
        public static final double SHOULDER_MANUAL_SPEED = 1.0; // degrees per loop
        public static final double SHOULDER_P = 0.0175;
        public static final double SHOULDER_I = 0.0;
        public static final double SHOULDER_D = 0.005;
        public static final double SHOULDER_OFFSET = 196.0; // degrees
        public static final double SHOULDER_MAX_VELOCITY = 90.0; // degrees per second
        public static final double SHOULDER_MAX_ACCELERATION = 90.0; // degrees per second squared

        public static final double SHOULDER_SUBSTATION = 65.0; // degrees
        public static final double ELBOW_SUBSTATION = 275.0; // degrees
        public static final double SHOULDER_STOW = 20.0; // degrees
        public static final double ELBOW_STOW = 220.0; // degrees
        public static final double SHOULDER_GROUND_CONE = 110.0; // degrees
        public static final double ELBOW_GROUND_CONE = 230.0; // degrees
        public static final double SHOULDER_GROUND_CUBE = 127.0; // degrees
        public static final double ELBOW_GROUND_CUBE = 224.0; // degrees
        public static final double SHOULDER_READY = 122.0; // degrees
        public static final double ELBOW_READY = 355.0; // degrees
        public static final double SHOULDER_HIGH = 106.0; // degrees
        public static final double ELBOW_HIGH = 319.0; // degrees
        public static final double SHOULDER_MID = 76.0; // degrees
        public static final double ELBOW_MID = 267.0; // degrees

        public static final double ANGLE_TOLERANCE = 2.5; // degrees
    }

    private CANSparkMax elbowMotor;
    private SparkMaxAbsoluteEncoder elbowEncoder;
    private SparkMaxPIDController elbowPID;
    private double elbowSetpoint;

    private CANSparkMax shoulderMotorRight;
    private CANSparkMax shoulderMotorLeft;
    private SparkMaxAbsoluteEncoder shoulderEncoder;
    private SparkMaxPIDController shoulderPID;
    private double shoulderSetpoint;

    public Arm() {
        elbowMotor = new CANSparkMax(ArmConstants.ELBOW_PORT, MotorType.kBrushless);
        elbowMotor.restoreFactoryDefaults();
        elbowMotor.setInverted(true);
        elbowMotor.setIdleMode(IdleMode.kBrake);
        elbowMotor.setSmartCurrentLimit(ArmConstants.ELBOW_STALL_CURRENT_LIMIT, ArmConstants.ELBOW_FREE_CURRENT_LIMIT);
        elbowMotor.setSecondaryCurrentLimit(ArmConstants.ELBOW_SECONDARY_CURRENT_LIMIT);
        elbowEncoder = elbowMotor.getAbsoluteEncoder(Type.kDutyCycle);
        elbowEncoder.setPositionConversionFactor(360); // degrees
        elbowEncoder.setZeroOffset(ArmConstants.ELBOW_OFFSET);
        elbowEncoder.setInverted(true);
        elbowPID = elbowMotor.getPIDController();
        elbowPID.setFeedbackDevice(elbowEncoder);
        elbowPID.setPositionPIDWrappingEnabled(true);
        elbowPID.setPositionPIDWrappingMinInput(0);
        elbowPID.setPositionPIDWrappingMaxInput(360);
        elbowPID.setP(ArmConstants.ELBOW_P, 0);
        elbowPID.setI(ArmConstants.ELBOW_I, 0);
        elbowPID.setD(ArmConstants.ELBOW_D, 0);
        elbowMotor.burnFlash();

        shoulderMotorRight = new CANSparkMax(ArmConstants.SHOULDER_RIGHT_PORT, MotorType.kBrushless);
        shoulderMotorRight.restoreFactoryDefaults();
        shoulderMotorRight.setInverted(true);
        shoulderMotorRight.setIdleMode(IdleMode.kBrake);
        shoulderMotorRight.setSmartCurrentLimit(ArmConstants.SHOULDER_STALL_CURRENT_LIMIT,
                ArmConstants.SHOULDER_FREE_CURRENT_LIMIT);
        shoulderMotorRight.setSecondaryCurrentLimit(ArmConstants.SHOULDER_SECONDARY_CURRENT_LIMIT);
        shoulderEncoder = shoulderMotorRight.getAbsoluteEncoder(Type.kDutyCycle);
        shoulderEncoder.setPositionConversionFactor(360); // degrees
        shoulderEncoder.setZeroOffset(ArmConstants.SHOULDER_OFFSET);
        shoulderEncoder.setInverted(true);
        shoulderPID = shoulderMotorRight.getPIDController();
        shoulderPID.setFeedbackDevice(shoulderEncoder);
        shoulderPID.setPositionPIDWrappingEnabled(true);
        shoulderPID.setPositionPIDWrappingMinInput(0);
        shoulderPID.setPositionPIDWrappingMaxInput(360);
        shoulderPID.setP(ArmConstants.SHOULDER_P, 0);
        shoulderPID.setI(ArmConstants.SHOULDER_I, 0);
        shoulderPID.setD(ArmConstants.SHOULDER_D, 0);
        shoulderMotorRight.burnFlash();

        shoulderMotorLeft = new CANSparkMax(ArmConstants.SHOULDER_LEFT_PORT, MotorType.kBrushless);
        shoulderMotorLeft.restoreFactoryDefaults();
        shoulderMotorLeft.setInverted(false);
        shoulderMotorLeft.setIdleMode(IdleMode.kBrake);
        shoulderMotorLeft.setSmartCurrentLimit(ArmConstants.SHOULDER_STALL_CURRENT_LIMIT,
                ArmConstants.SHOULDER_FREE_CURRENT_LIMIT);
        shoulderMotorLeft.setSecondaryCurrentLimit(ArmConstants.SHOULDER_SECONDARY_CURRENT_LIMIT);
        shoulderMotorLeft.follow(shoulderMotorRight, true);
        shoulderMotorLeft.burnFlash();
    }

    private void setElbowAngle(double angle) {
        while (angle > 360) {
            angle -= 360;
        }
        while (angle < 0) {
            angle += 360;
        }
        elbowSetpoint = angle;
    }

    private double getElbowTargetAngle() {
        return elbowSetpoint;
    }

    private double getElbowAngle() {
        return elbowEncoder.getPosition();
    }

    private double getElbowError() {
        return getElbowAngle() - getElbowTargetAngle();
    }

    private void setShoulderAngle(double angle) {
        while (angle > 360) {
            angle -= 360;
        }
        while (angle < 0) {
            angle += 360;
        }
        shoulderSetpoint = angle;
    }

    private double getShoulderTargetAngle() {
        return shoulderSetpoint;
    }

    private double getShoulderAngle() {
        return shoulderEncoder.getPosition();
    }

    private double getShoulderError() {
        return getShoulderAngle() - getShoulderTargetAngle();
    }

    private boolean onTarget() {
        return Math.abs(getShoulderError()) < ArmConstants.ANGLE_TOLERANCE
                && Math.abs(getElbowError()) < ArmConstants.ANGLE_TOLERANCE;
    }

    private void setAngles(double elbowAngle, double shoulderAngle) {
        setElbowAngle(elbowAngle);
        setShoulderAngle(shoulderAngle);
    }

    @Override
    public void periodic() {
        elbowPID.setReference(elbowSetpoint, ControlType.kPosition);
        shoulderPID.setReference(shoulderSetpoint, ControlType.kPosition);
    }

    private class PositionCommand extends CommandBase {
        private final Arm arm;
        private final double elbowSetpoint;
        private final double shoulderSetpoint;

        public PositionCommand(Arm arm, double elbowSetpoint, double shoulderSetpoint) {
            this.arm = arm;
            this.elbowSetpoint = elbowSetpoint;
            this.shoulderSetpoint = shoulderSetpoint;
            addRequirements(arm);
        }

        @Override
        public void execute() {
            arm.setAngles(elbowSetpoint, shoulderSetpoint);
        }

        @Override
        public boolean isFinished() {
            return arm.onTarget();
        }
    }

    public Command groundCone() {
        return new PositionCommand(
                this,
                ArmConstants.ELBOW_GROUND_CONE,
                ArmConstants.SHOULDER_GROUND_CONE);
    }

    public Command groundCube() {
        return new PositionCommand(
                this,
                ArmConstants.ELBOW_GROUND_CUBE,
                ArmConstants.SHOULDER_GROUND_CUBE);
    }

    public Command high() {
        return new PositionCommand(
                this,
                ArmConstants.ELBOW_HIGH,
                ArmConstants.SHOULDER_HIGH);
    }

    public Command mid() {
        return new PositionCommand(
                this,
                ArmConstants.ELBOW_MID,
                ArmConstants.SHOULDER_MID);
    }

    public Command ready() {
        return new PositionCommand(
                this,
                ArmConstants.ELBOW_READY,
                ArmConstants.SHOULDER_READY);
    }

    public Command stow() {
        return new PositionCommand(
                this,
                ArmConstants.ELBOW_STOW,
                ArmConstants.SHOULDER_STOW);
    }

    public Command substation() {
        return new PositionCommand(
                this,
                ArmConstants.ELBOW_SUBSTATION,
                ArmConstants.SHOULDER_SUBSTATION);
    }

    public Command defaultCommand(Supplier<Double> elbowChange, Supplier<Double> shoulderChange) {
        return new RunCommand(() -> {
            double elbow = getElbowTargetAngle();
            elbow += elbowChange.get() * ArmConstants.ELBOW_MANUAL_SPEED;

            double shoulder = getShoulderTargetAngle();
            shoulder += shoulderChange.get() * ArmConstants.SHOULDER_MANUAL_SPEED;

            setAngles(elbow, shoulder);
        }, this);
    }
}
