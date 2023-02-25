package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkAbsoluteEncoder;
import frc.robot.util.SparkEncoder;
import frc.robot.util.SparkMotor;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import static frc.robot.Constants.*;

public class Arm extends SubsystemBase {

    private SparkMotor elbowMotorLeft;
    private SparkMotor elbowMotorRight;
    private SparkMotor shoulderMotorLeft;
    private SparkMotor shoulderMotorRight;
    private SparkAbsoluteEncoder elbowEncoder;
    private SparkAbsoluteEncoder shoulderEncoder;
    private SparkMaxPIDController elbowPID;
    private SparkMaxPIDController shoulderPID;

    public Arm() {

        elbowMotorLeft = new SparkMotor(12, MotorType.kBrushless);

        elbowMotorLeft.restoreFactoryDefaults();
        elbowMotorLeft.setInverted(false);
        elbowMotorLeft.setIdleMode(IdleMode.kBrake);

        elbowMotorRight = new SparkMotor(12, MotorType.kBrushless);

        elbowMotorRight.restoreFactoryDefaults();
        elbowMotorRight.setInverted(false);
        elbowMotorRight.setIdleMode(IdleMode.kBrake);


        elbowPID = elbowMotorLeft.getPIDController();
        elbowEncoder = new SparkAbsoluteEncoder(elbowMotorLeft.getAbsoluteEncoder(Type.kDutyCycle));
        elbowPID.setP(0.001);
        elbowPID.setI(0.0);
        elbowPID.setD(0.0);
        elbowPID.setFeedbackDevice(elbowEncoder);
        elbowPID.setPositionPIDWrappingEnabled(true);
        elbowPID.setPositionPIDWrappingMinInput(0.0);
        elbowPID.setPositionPIDWrappingMaxInput(1.0);

        shoulderMotorLeft = new SparkMotor(12, MotorType.kBrushless);

        shoulderMotorLeft.restoreFactoryDefaults();
        shoulderMotorLeft.setInverted(false);
        shoulderMotorLeft.setIdleMode(IdleMode.kBrake);

        shoulderMotorRight = new SparkMotor(13, MotorType.kBrushless);

        shoulderMotorRight.restoreFactoryDefaults();
        shoulderMotorRight.setInverted(true);
        shoulderMotorRight.setIdleMode(IdleMode.kBrake);

        shoulderPID = elbowMotorLeft.getPIDController();
        shoulderEncoder = new SparkAbsoluteEncoder(elbowMotorLeft.getAbsoluteEncoder(Type.kDutyCycle));
        shoulderPID.setP(0.001);
        shoulderPID.setI(0.0);
        shoulderPID.setD(0.0);
        shoulderPID.setFeedbackDevice(elbowEncoder);
        shoulderPID.setPositionPIDWrappingEnabled(true);
        shoulderPID.setPositionPIDWrappingMinInput(0.0);
        shoulderPID.setPositionPIDWrappingMaxInput(1.0);

        //shoulderEncoder = shoulderMotorLeft.getAbsoluteEncoder(Type.kDutyCycle);
        //elbowEncoder = elbowMotor.getAbsoluteEncoder(Type.kDutyCycle);

        shoulderMotorLeft.follow(shoulderMotorRight);
        elbowEncoder.setZeroOffset(ELBOW_ENCODER_OFFSET);
        addChild("elbowEncoder", elbowEncoder);

        shoulderMotorLeft = new SparkMotor(12, MotorType.kBrushless);
        shoulderMotorLeft.restoreFactoryDefaults();
        shoulderMotorLeft.setInverted(false);
        shoulderMotorLeft.setIdleMode(IdleMode.kBrake);
        addChild("shoulderMotorLeft", shoulderMotorLeft);

        shoulderMotorRight = new SparkMotor(13, MotorType.kBrushless);
        shoulderMotorRight.restoreFactoryDefaults();
        shoulderMotorRight.setInverted(true);
        shoulderMotorRight.setIdleMode(IdleMode.kBrake);
        addChild("shoulderMotorRight", shoulderMotorRight);

        shoulderEncoder = new SparkAbsoluteEncoder(shoulderMotorLeft.getAbsoluteEncoder(Type.kDutyCycle));
        shoulderEncoder.setZeroOffset(SHOULDER_ENCODER_OFFSET);
        addChild("shoulderEncoder", shoulderEncoder);
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
        // TODO : get PID to work here
    }

    public void setElbowSetpoint(double setpoint) {
         // TODO : get PID to work here
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shoulder Angle", getShoulderAngle());
        SmartDashboard.putNumber("Elbow Angle", getElbowAngle());
    }
}
