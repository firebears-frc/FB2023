package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkAbsoluteEncoder;
import frc.robot.util.SparkEncoder;
import frc.robot.util.SparkMotor;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import static frc.robot.Constants.*;

public class Arm extends SubsystemBase {

    private SparkMotor elbowMotor;
    private SparkMotor shoulderMotorLeft;
    private SparkMotor shoulderMotorRight;
    private static SparkMaxAbsoluteEncoder elbowEncoder;
    private static SparkMaxAbsoluteEncoder shoulderEncoder;
    private static SparkMaxPIDController elbowPID;
    private SparkMaxPIDController shoulderPID;
    private double elbowSetpoint;
    private double shoulderSetpoint;
    Arm arm;
    private DoubleLogEntry elbowLog;
    private DoubleLogEntry shoulderLog;
    private double prevShoulderAngle;
    private double prevElbowAngle;

    public Arm() {

        elbowMotor = new SparkMotor(7, MotorType.kBrushless);

        elbowMotor.restoreFactoryDefaults();
        elbowMotor.setInverted(true);
        elbowMotor.setIdleMode(IdleMode.kBrake);

        elbowPID = elbowMotor.getPIDController();
        elbowEncoder = elbowMotor.getAbsoluteEncoder(Type.kDutyCycle);
        elbowPID.setP(.01);
        elbowPID.setI(0.0);
        elbowPID.setD(0.0005);
        elbowPID.setFeedbackDevice(elbowEncoder);
        elbowPID.setPositionPIDWrappingEnabled(true);
        elbowPID.setPositionPIDWrappingMinInput(0.0);
        elbowPID.setPositionPIDWrappingMaxInput(360);
        elbowEncoder.setPositionConversionFactor(360);
        elbowEncoder.setZeroOffset(ELBOW_ENCODER_OFFSET);
        elbowMotor.burnFlash();

        shoulderMotorLeft = new SparkMotor(12, MotorType.kBrushless);

        shoulderMotorLeft.restoreFactoryDefaults();
        shoulderMotorLeft.setInverted(false);
        shoulderMotorLeft.setIdleMode(IdleMode.kBrake);

        shoulderMotorRight = new SparkMotor(13, MotorType.kBrushless);

        shoulderMotorRight.restoreFactoryDefaults();
        shoulderMotorRight.setInverted(true);
        shoulderMotorRight.setIdleMode(IdleMode.kBrake);
        shoulderMotorRight.follow(shoulderMotorLeft);

        shoulderPID = shoulderMotorLeft.getPIDController();
        shoulderEncoder = shoulderMotorLeft.getAbsoluteEncoder(Type.kDutyCycle);
        shoulderPID.setP(0.01);
        shoulderPID.setI(0.0);
        shoulderPID.setD(0.0005);
        shoulderPID.setFeedbackDevice(shoulderEncoder);
        shoulderPID.setPositionPIDWrappingEnabled(true);
        shoulderPID.setPositionPIDWrappingMinInput(0.0);
        shoulderPID.setPositionPIDWrappingMaxInput(360);
        shoulderEncoder.setZeroOffset(SHOULDER_ENCODER_OFFSET);
        shoulderMotorLeft.burnFlash();
        shoulderMotorRight.burnFlash();
        elbowSetpoint = getElbowAngle();
        shoulderSetpoint = getShoulderAngle();

        if (LOGGING) {
            elbowLog = new DoubleLogEntry(DataLogManager.getLog(), "/arm/elbow");
            shoulderLog = new DoubleLogEntry(DataLogManager.getLog(), "/arm/shoulder");
        }
    }

    public double getShoulderAngle() {
        double angle = shoulderEncoder.getPosition();
        return angle;
    }

    public double getElbowAngle() {
        double angle = elbowEncoder.getPosition();
        return angle;
    }

    public void setShoulderSetpoint(double setpoint) {
        while (setpoint > 360) {
            setpoint -= 360;
        }
        while (setpoint < 0) {
            setpoint += 360;
        }

        if (setpoint > 15 && setpoint < 180) {
            System.out.println("Shoulder1: " + setpoint);
            setpoint = 15;
        } else if (setpoint < 207 && setpoint > 180) {
            System.out.println("Shoulder2: " + setpoint);
            setpoint = 207;
        }
        shoulderSetpoint = setpoint;
    }

    public void setElbowSetpoint(double setpoint) {
        while (setpoint > 360) {
            setpoint -= 360;
        }
        while (setpoint < 0) {
            setpoint += 360;
        }

        if (setpoint > 15 && setpoint < 180) {
            System.out.println("Elbow1: " + setpoint);
            setpoint = 15;
        } else if (setpoint < 207 && setpoint > 180) {
            System.out.println("Elbow2: " + setpoint);
            setpoint = 207;
        }
        elbowSetpoint = setpoint;
    }

    public double getElbowSetpoint() {
        return elbowSetpoint;
    }

    public double getShoulderSetpoint() {
        return shoulderSetpoint;
    }

    @Override
    public void periodic() {
        double shoulderAngle = getShoulderAngle();
        double elbowAngle = getElbowAngle();
        if (DEBUG) {
            SmartDashboard.putNumber("Shoulder Angle", shoulderAngle);
            SmartDashboard.putNumber("Elbow Angle", elbowAngle);
            SmartDashboard.putNumber("Setpoint", elbowSetpoint);
        }
        elbowPID.setReference(elbowSetpoint, ControlType.kPosition);
        if (LOGGING && (shoulderAngle != prevShoulderAngle || elbowAngle != prevElbowAngle)) {
            shoulderLog.append(shoulderAngle);
            elbowLog.append(elbowAngle);
        }
        prevElbowAngle = elbowAngle;
        prevShoulderAngle = shoulderAngle;
    }
}
