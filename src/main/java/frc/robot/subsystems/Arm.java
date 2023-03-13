package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.SparkMotor;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import static frc.robot.Constants.*;

public class Arm extends SubsystemBase {
    private static int STALL_CURRENT_LIMIT_SHOULDER = 30;
    private static int FREE_CURRENT_LIMIT_SHOULDER = 25;
    private static int SECONDARY_CURRENT_LIMIT_SHOULDER = 35;

    private static int STALL_CURRENT_LIMIT_ELBOW = 40;
    private static int FREE_CURRENT_LIMIT_ELBOW = 35;
    private static int SECONDARY_CURRENT_LIMIT_ELBOW = 45;

    private SparkMotor elbowMotor;
    private SparkMotor shoulderMotorRight;
    private SparkMotor shoulderMotorLeft;
    private static SparkMaxAbsoluteEncoder elbowEncoder;
    private static SparkMaxAbsoluteEncoder shoulderEncoder;
    private static SparkMaxPIDController elbowPID;
    private SparkMaxPIDController shoulderPID;
    private double elbowSetpoint;
    private double shoulderSetpoint;
    Arm arm;

    public Arm() {

        elbowMotor = new SparkMotor(7, MotorType.kBrushless);

        elbowMotor.restoreFactoryDefaults();
        elbowMotor.setInverted(true);
        elbowMotor.setIdleMode(IdleMode.kBrake);
        elbowMotor.setSmartCurrentLimit(STALL_CURRENT_LIMIT_ELBOW, FREE_CURRENT_LIMIT_ELBOW);
        elbowMotor.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT_ELBOW);


        elbowPID = elbowMotor.getPIDController();
        elbowEncoder = elbowMotor.getAbsoluteEncoder(Type.kDutyCycle);
        elbowPID.setP(0.03);
        elbowPID.setI(0.0);
        elbowPID.setD(0.0005);
        elbowPID.setFeedbackDevice(elbowEncoder);
        elbowPID.setPositionPIDWrappingEnabled(true);
        elbowPID.setPositionPIDWrappingMinInput(0.0);
        elbowPID.setPositionPIDWrappingMaxInput(360);
        elbowEncoder.setPositionConversionFactor(360);
        elbowEncoder.setZeroOffset(ELBOW_ENCODER_OFFSET);
        elbowMotor.burnFlash();


        shoulderMotorRight = new SparkMotor(8, MotorType.kBrushless);

        shoulderMotorRight.restoreFactoryDefaults();
        shoulderMotorRight.setInverted(true);
        shoulderMotorRight.setIdleMode(IdleMode.kBrake);
        shoulderMotorRight.setSmartCurrentLimit(STALL_CURRENT_LIMIT_SHOULDER, FREE_CURRENT_LIMIT_SHOULDER);
        shoulderMotorRight.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT_SHOULDER);

        shoulderMotorLeft = new SparkMotor(9, MotorType.kBrushless);

        shoulderMotorLeft.restoreFactoryDefaults();
        shoulderMotorLeft.setInverted(false);
        shoulderMotorLeft.setIdleMode(IdleMode.kBrake);
        shoulderMotorLeft.setSmartCurrentLimit(STALL_CURRENT_LIMIT_SHOULDER, FREE_CURRENT_LIMIT_SHOULDER);
        shoulderMotorLeft.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT_SHOULDER);
        shoulderMotorLeft.follow(shoulderMotorRight, true);
        shoulderMotorLeft.burnFlash();

        shoulderPID = shoulderMotorRight.getPIDController();
        shoulderEncoder = shoulderMotorRight.getAbsoluteEncoder(Type.kDutyCycle);
        shoulderPID.setP(0.02);
        shoulderPID.setI(0);
        shoulderPID.setD(0.001);
        shoulderPID.setFeedbackDevice(shoulderEncoder);
        shoulderPID.setPositionPIDWrappingEnabled(true);
        shoulderPID.setPositionPIDWrappingMinInput(0.0);
        shoulderPID.setPositionPIDWrappingMaxInput(360);
        shoulderEncoder.setPositionConversionFactor(360);
        shoulderEncoder.setZeroOffset(SHOULDER_ENCODER_OFFSET);
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
        while (setpoint > 360) {
            setpoint -= 360;
        }
        while (setpoint < 0) {
            setpoint += 360;
        }

         if(setpoint < 0 || setpoint > 280){

            setpoint = 0;
        } else if (setpoint > 200 && setpoint < 280) {
        
            setpoint = 199;
        } else {
    
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

        if(setpoint > 15 && setpoint < 180){
       
            setpoint = 14;
        } else if (setpoint < 200 && setpoint > 180) {
    
            setpoint = 201;
   
        }
        elbowSetpoint = setpoint;
    }

    public double getElbowSetpoint() {
        return elbowSetpoint;
    }
    public double getShoulderSetpoint() {
        return shoulderSetpoint;
    }
    public Translation2d getArmPosition(){
        double elbowX=Math.cos(Math.toRadians(getShoulderAngle()));
        double elbowY=Math.sin(Math.toRadians(getShoulderAngle()));
        elbowX*=shoulderArmLength;
        elbowY*=shoulderArmLength;

        double shluckerX=Math.cos(Math.toRadians(getElbowAngle()+getShoulderAngle()));
        double ShluckerY=Math.sin(Math.toRadians(getElbowAngle()+getShoulderAngle()));
        shluckerX*=elbowArmLength;
        ShluckerY*=elbowArmLength;

        Translation2d output=new Translation2d(elbowX+shluckerX,elbowY+ShluckerY);
        return output;
    }

    @Override
    public void periodic() {
        if (DEBUG)
        {SmartDashboard.putNumber("shoulder angle", getShoulderAngle());
        SmartDashboard.putNumber("elbow angle", getElbowAngle());
        SmartDashboard.putNumber("setpoint", elbowSetpoint);
        SmartDashboard.putNumber("shoulder setpoint", shoulderSetpoint);
        elbowPID.setReference(elbowSetpoint, ControlType.kPosition);
        shoulderPID.setReference(shoulderSetpoint, ControlType.kPosition);}

        //SmartDashboard.putString("shlucker position",getArmPosition().getX()+"+"+getArmPosition().getY());

        //SmartDashboard.putNumber("Shoulder Left Output", shoulderMotorRight.getAppliedOutput());
        //SmartDashboard.putNumber("Shoulder Right Output", shoulderMotorLeft.getAppliedOutput());
        //SmartDashboard.putNumber("Elbow Output", elbowMotor.getAppliedOutput());
    }
}
