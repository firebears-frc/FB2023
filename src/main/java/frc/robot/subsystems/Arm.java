package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.util.SparkMotor;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import static frc.robot.Constants.*;

import java.util.function.Function;

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
    private double elbowSetpoint;
    private double shoulderSetpoint;
    private ProfiledPIDController elbowProfiledPID;
    private ProfiledPIDController shoulderProfiledPID;
    private Constraints elbowTrapezoidalConstraint;
    private LinearFilter elbowAccelFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    private double elobwPreviousVelocity;

    public Arm() {
        elbowMotor = new SparkMotor(7, MotorType.kBrushless);
        elbowMotor.restoreFactoryDefaults();
        elbowMotor.setInverted(true);
        elbowMotor.setIdleMode(IdleMode.kBrake);
        elbowMotor.setSmartCurrentLimit(STALL_CURRENT_LIMIT_ELBOW, FREE_CURRENT_LIMIT_ELBOW);
        elbowMotor.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT_ELBOW);

        elbowEncoder = elbowMotor.getAbsoluteEncoder(Type.kDutyCycle);

        elbowEncoder.setPositionConversionFactor(360);
        elbowEncoder.setVelocityConversionFactor(360);
        elbowEncoder.setZeroOffset(ELBOW_ENCODER_OFFSET);
        elbowEncoder.setInverted(true);
        elbowMotor.burnFlash();

        elobwPreviousVelocity = elbowEncoder.getVelocity();
        
        elbowTrapezoidalConstraint = new Constraints(10, 50);
        elbowProfiledPID = new ProfiledPIDController(
            PracticeArmConstants.elbowP, 
            PracticeArmConstants.elbowI, 
            PracticeArmConstants.elbowD, 
            elbowTrapezoidalConstraint
        );
        elbowProfiledPID.enableContinuousInput(0, 360);

        shoulderMotorRight = new SparkMotor(8, MotorType.kBrushless);

        shoulderMotorRight.restoreFactoryDefaults();
        shoulderMotorRight.setInverted(true);
        shoulderMotorRight.setIdleMode(IdleMode.kBrake);
        shoulderMotorRight.setSmartCurrentLimit(STALL_CURRENT_LIMIT_SHOULDER, FREE_CURRENT_LIMIT_SHOULDER);
        shoulderMotorRight.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT_SHOULDER);

        shoulderEncoder = shoulderMotorRight.getAbsoluteEncoder(Type.kDutyCycle);
        shoulderEncoder.setInverted(true);
        shoulderEncoder.setPositionConversionFactor(360);
        shoulderEncoder.setZeroOffset(SHOULDER_ENCODER_OFFSET);
        shoulderMotorRight.burnFlash();

        shoulderMotorLeft = new SparkMotor(9, MotorType.kBrushless);

        shoulderMotorLeft.restoreFactoryDefaults();
        shoulderMotorLeft.setInverted(false);
        shoulderMotorLeft.setIdleMode(IdleMode.kBrake);
        shoulderMotorLeft.setSmartCurrentLimit(STALL_CURRENT_LIMIT_SHOULDER, FREE_CURRENT_LIMIT_SHOULDER);
        shoulderMotorLeft.setSecondaryCurrentLimit(SECONDARY_CURRENT_LIMIT_SHOULDER);
        shoulderMotorLeft.follow(shoulderMotorRight, true);
        shoulderMotorLeft.burnFlash();

        shoulderProfiledPID = new ProfiledPIDController(
            PracticeArmConstants.shoulderP, 
            PracticeArmConstants.shoulderI, 
            PracticeArmConstants.shoulderD, 
            elbowTrapezoidalConstraint
        );

        
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
        if (!violatesFramePerimeter(setpoint, getElbowAngle())) {
            shoulderSetpoint = setpoint;

        } else {
            System.out.println("hit limit");
        }
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
        if (!violatesFramePerimeter(getShoulderAngle(), setpoint)) {
            elbowSetpoint = setpoint;

        } else {
            System.out.println("hit limit");
        }
    }

    public double getElbowSetpoint() {
        return elbowSetpoint;
    }

    public double getShoulderSetpoint() {
        return shoulderSetpoint;
    }

    public Translation2d getArmPosition(double shoulder_Angle, double elbow_Angle) {
        // 105 is when its 6 inches off

        double shoulderComplement = 180 - shoulder_Angle;
        double elbowX = Math.cos(Math.toRadians(shoulderComplement));
        double elbowY = Math.sin(Math.toRadians(shoulderComplement));
        elbowX *= ARM_SHOULDER_LENGTH;
        elbowY *= ARM_SHOULDER_LENGTH;

        double shluckerX = Math.cos(Math.toRadians(elbow_Angle + shoulderComplement));
        double shluckerY = Math.sin(Math.toRadians(elbow_Angle + shoulderComplement));
        shluckerX *= ARM_ELBOW_LENGTH;
        shluckerY *= ARM_ELBOW_LENGTH;

        Translation2d output = new Translation2d(elbowX + shluckerX, elbowY + shluckerY);
        return output;
    }

    public boolean violatesFramePerimeter(double shoulder_Angle, double elbow_Angle) {
        double currentExtension = getArmPosition(getShoulderAngle(), getElbowAngle()).getX();
        double desiredExtension = getArmPosition(shoulder_Angle, elbow_Angle).getX();
        return shoulder_Angle > 105 && !(desiredExtension < currentExtension || desiredExtension < 45);

    }

    @Override
    public void periodic() {
        if (DEBUG) {

            double acceleration = elbowAccelFilter.calculate(elbowEncoder.getVelocity() - elobwPreviousVelocity);
            elobwPreviousVelocity = elbowEncoder.getVelocity();

            SmartDashboard.putNumber("shoulder angle", getShoulderAngle());
            SmartDashboard.putNumber("shoulder setpoint", shoulderSetpoint);
            SmartDashboard.putNumber("shoulder left output", shoulderMotorRight.getAppliedOutput());
            SmartDashboard.putNumber("shoulder right output", shoulderMotorLeft.getAppliedOutput());
            SmartDashboard.putNumber("shoulder X", getArmPosition(getShoulderAngle(), getElbowAngle()).getX());
            SmartDashboard.putNumber("shoulder Y", getArmPosition(getShoulderAngle(), getElbowAngle()).getY());

            SmartDashboard.putNumber("elbow angle", getElbowAngle());
            SmartDashboard.putNumber("elbow setpoint", elbowSetpoint);
            SmartDashboard.putNumber("elbow outut", elbowMotor.getAppliedOutput());

            SmartDashboard.putNumber("Elbow velocity", elbowEncoder.getVelocity());
            SmartDashboard.putNumber("Shoulder velocity", shoulderEncoder.getVelocity());
            SmartDashboard.putNumber("Elbow Acceleration", acceleration);
        }

        elbowMotor.setVoltage(
            elbowProfiledPID.calculate(getElbowAngle(), elbowSetpoint));
        shoulderMotorRight.setVoltage(
            shoulderProfiledPID.calculate(getShoulderAngle(), shoulderSetpoint));
    }
}
