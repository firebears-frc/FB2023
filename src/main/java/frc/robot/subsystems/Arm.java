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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import static frc.robot.Constants.*;

public class Arm extends SubsystemBase {

    private SparkMotor elbowMotor;
    private SparkMotor shoulderMotorLeft;
    private SparkMotor shoulderMotorRight;
    private SparkAbsoluteEncoder elbowEncoder;
    private SparkAbsoluteEncoder shoulderEncoder;

    public Arm() {
        elbowMotor = new SparkMotor(7, MotorType.kBrushless);
        elbowMotor.restoreFactoryDefaults();
        elbowMotor.setInverted(false);
        elbowMotor.setIdleMode(IdleMode.kBrake);
        addChild("elbowMotor", elbowMotor);

        //shoulderMotorLeft = new SparkMotor(12, MotorType.kBrushless);

        /*shoulderMotorLeft.restoreFactoryDefaults();
        shoulderMotorLeft.setInverted(false);
        shoulderMotorLeft.setIdleMode(IdleMode.kBrake);

        shoulderMotorRight = new SparkMotor(13, MotorType.kBrushless);

        shoulderMotorRight.restoreFactoryDefaults();
        shoulderMotorRight.setInverted(true);
        shoulderMotorRight.setIdleMode(IdleMode.kBrake);*/

        //shoulderEncoder = shoulderMotorLeft.getAbsoluteEncoder(Type.kDutyCycle);
        //elbowEncoder = elbowMotor.getAbsoluteEncoder(Type.kDutyCycle);

        //shoulderMotorLeft.follow(shoulderMotorRight);
        elbowEncoder = new SparkAbsoluteEncoder(elbowMotor.getAbsoluteEncoder(Type.kDutyCycle));
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
