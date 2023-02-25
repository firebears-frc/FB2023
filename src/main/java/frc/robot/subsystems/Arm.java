package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
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
    private double elbowSetpoint = 330;

    Arm arm;

    public Arm() {

        elbowMotor = new SparkMotor(7, MotorType.kBrushless);

        elbowMotor.restoreFactoryDefaults();
        elbowMotor.setInverted(true);
        elbowMotor.setIdleMode(IdleMode.kBrake);


        elbowPID = elbowMotor.getPIDController();
        elbowEncoder = elbowMotor.getAbsoluteEncoder(Type.kDutyCycle);
        elbowPID.setP(.005);
        elbowPID.setI(0.0);
        elbowPID.setD(0.0005);
        elbowPID.setFeedbackDevice(elbowEncoder);
       // elbowPID.setPositionPIDWrappingEnabled(true);
       // elbowPID.setPositionPIDWrappingMinInput(0.0);
       // elbowPID.setPositionPIDWrappingMaxInput(1.0);
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
        shoulderPID.setP(0.1);
        shoulderPID.setI(0.0);
        shoulderPID.setD(0.0);
        shoulderPID.setFeedbackDevice(shoulderEncoder);
        // shoulderPID.setPositionPIDWrappingEnabled(true);
        // shoulderPID.setPositionPIDWrappingMinInput(0.0);
        // shoulderPID.setPositionPIDWrappingMaxInput(1.0);
        shoulderEncoder.setZeroOffset(SHOULDER_ENCODER_OFFSET);
        shoulderMotorLeft.burnFlash();
        shoulderMotorRight.burnFlash();
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
    }

    public void setElbowSetpoint(double setpoint) {
        System.out.println("Setting Elbow: " + setpoint);
        elbowSetpoint = setpoint;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shoulder Angle", getShoulderAngle());
        SmartDashboard.putNumber("Elbow Angle", getElbowAngle());
        SmartDashboard.putNumber("Setpoint", elbowSetpoint);
        elbowPID.setReference(elbowSetpoint, ControlType.kPosition);
    }
}
