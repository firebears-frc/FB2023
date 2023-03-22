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

    public Arm() {

        elbowMotor = new SparkMotor(7, MotorType.kBrushless);

        elbowMotor.restoreFactoryDefaults();
        elbowMotor.setInverted(true);
        elbowMotor.setIdleMode(IdleMode.kBrake);
        elbowMotor.setSmartCurrentLimit(STALL_CURRENT_LIMIT_ELBOW, FREE_CURRENT_LIMIT_ELBOW);
        elbowMotor.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT_ELBOW);

        elbowPID = elbowMotor.getPIDController();
        elbowEncoder = elbowMotor.getAbsoluteEncoder(Type.kDutyCycle);

        if (PRACTICE_ROBOT) {
            elbowPID.setP(PracticeArmConstants.elbowP);
            elbowPID.setI(PracticeArmConstants.elbowI);
            elbowPID.setD(PracticeArmConstants.elbowD);
        } else {
            elbowPID.setP(CompArmConstants.elbowP);
            elbowPID.setI(CompArmConstants.elbowI);
            elbowPID.setD(CompArmConstants.elbowD);
        }


        elbowPID.setFeedbackDevice(elbowEncoder);
        elbowPID.setPositionPIDWrappingEnabled(true);
        elbowPID.setPositionPIDWrappingMinInput(0.0);
        elbowPID.setPositionPIDWrappingMaxInput(360);
        elbowEncoder.setPositionConversionFactor(360);
        elbowEncoder.setZeroOffset(ELBOW_ENCODER_OFFSET);
        elbowEncoder.setInverted(true); 
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
        shoulderEncoder.setInverted(true); 
        if (PRACTICE_ROBOT) {
            shoulderPID.setP(PracticeArmConstants.shoulderP);
            shoulderPID.setI(PracticeArmConstants.shoulderI);
            shoulderPID.setD(PracticeArmConstants.shoulderD);
        } else {
            shoulderPID.setP(CompArmConstants.shoulderP);
            shoulderPID.setI(CompArmConstants.shoulderI);
            shoulderPID.setD(CompArmConstants.shoulderD);
        }

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

        if (setpoint < 0 || setpoint > 280) {

            setpoint = 0;
        } else if (setpoint > 130 && setpoint < 280) {

            setpoint = 130;
        } else {

        }
        if (!violatesFramePerimiter(setpoint, getElbowAngle())){
        shoulderSetpoint = setpoint;

        }else {System.out.println("hit limit");}
    }

    public void setElbowSetpoint(double setpoint) {
        while (setpoint > 360) {
            setpoint -= 360;
        }
        while (setpoint < 0) {
            setpoint += 360;
        }

        if (setpoint > 15 && setpoint < 180) {

            setpoint = 14;
        } else if (setpoint < 200 && setpoint > 180) {

            setpoint = 201;

        }
        if (!violatesFramePerimiter(getShoulderAngle(), setpoint)){
            elbowSetpoint = setpoint;
            
        }else {System.out.println("hit limit");}
    }

    public double getElbowSetpoint() {
        return elbowSetpoint;
    }

    public double getShoulderSetpoint() {
        return shoulderSetpoint;
    }

    public Translation2d getArmPosition(double shoulder_Angle, double elbow_Angle) {
             //105 is when its 6 inches off
             
        double shoulder_Compliment = 180-shoulder_Angle;
        double elbowX = Math.cos(Math.toRadians(shoulder_Compliment));
        double elbowY = Math.sin(Math.toRadians(shoulder_Compliment));
        elbowX *= ARM_SHOULDER_LENGTH;
        elbowY *= ARM_SHOULDER_LENGTH;

        double shluckerX = Math.cos(Math.toRadians(elbow_Angle + shoulder_Compliment));
        double shluckerY = Math.sin(Math.toRadians(elbow_Angle + shoulder_Compliment));
        shluckerX *= ARM_ELBOW_LENGTH;
        shluckerY *= ARM_ELBOW_LENGTH;

        Translation2d output = new Translation2d(elbowX + shluckerX, elbowY + shluckerY);
        return output;
    }
    public boolean violatesFramePerimiter(double shoulder_Angle, double elbow_Angle) {
        return shoulder_Angle > 105 && getArmPosition(shoulder_Angle, elbow_Angle).getX()+5>46;



    }

    @Override
    public void periodic() {
        if (DEBUG) {
            SmartDashboard.putNumber("shoulder angle", getShoulderAngle());
            SmartDashboard.putNumber("shoulder setpoint", shoulderSetpoint);
            SmartDashboard.putNumber("shoulder left output", shoulderMotorRight.getAppliedOutput());
            SmartDashboard.putNumber("shoulder right output", shoulderMotorLeft.getAppliedOutput());
         //   SmartDashboard.putNumber("shoulder X", getArmPosition().getX());
           // SmartDashboard.putNumber("shoulder Y", getArmPosition().getY());

            SmartDashboard.putNumber("elbow angle", getElbowAngle());
            SmartDashboard.putNumber("elbow setpoint", elbowSetpoint);
            SmartDashboard.putNumber("elbow output", elbowMotor.getAppliedOutput());
        }

        elbowPID.setReference(elbowSetpoint, ControlType.kPosition);
        shoulderPID.setReference(shoulderSetpoint, ControlType.kPosition);
    }
}
