package frc.robot.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;

public class Elbow extends Ligament {
    private static final class Constants {
        public static final int PORT = 7;

        public static final int STALL_CURRENT_LIMIT = 40;
        public static final int FREE_CURRENT_LIMIT = 20;
        public static final double SECONDARY_CURRENT_LIMIT = 45.0;

        public static final double P = 0.01;
        public static final double I = 0.0;
        public static final double D = 0.005;
    }

    private final CANSparkMax motor;
    private final SparkMaxAbsoluteEncoder encoder;
    private final SparkMaxPIDController pid;

    public Elbow() {
        motor = new CANSparkMax(Constants.PORT, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setInverted(true);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setSmartCurrentLimit(Constants.STALL_CURRENT_LIMIT, Constants.FREE_CURRENT_LIMIT);
        motor.setSecondaryCurrentLimit(Constants.SECONDARY_CURRENT_LIMIT);
        encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
        encoder.setPositionConversionFactor(360); // degrees\
        encoder.setInverted(true);
        pid = motor.getPIDController();
        pid.setFeedbackDevice(encoder);
        pid.setPositionPIDWrappingEnabled(true);
        pid.setPositionPIDWrappingMinInput(0);
        pid.setPositionPIDWrappingMaxInput(360);
        pid.setP(Constants.P, 0);
        pid.setI(Constants.I, 0);
        pid.setD(Constants.D, 0);

        motor.burnFlash();

        // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
        //motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

        name = "Elbow";
    }

    public void periodic() {
        position = Rotation2d.fromDegrees(encoder.getPosition());
        pid.setReference(setpoint.getDegrees(), ControlType.kPosition);
    }
}
