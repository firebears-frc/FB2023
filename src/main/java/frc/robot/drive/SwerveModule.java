package frc.robot.drive;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private static final class Constants {
        public static final double NEO_FREE_SPEED = 5676.0 / 60; // rotations per second

        private static class Driving {
            private static final double WHEEL_DIAMETER = 0.0762; // meters
            private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; // meters
            // 12T, 13T, or 14T gearing
            private static final int PINION_TEETH = 14;
            private static final double GEAR_RATIO = 45 * 22 / (PINION_TEETH * 15);
            private static final double FREE_SPEED = NEO_FREE_SPEED * WHEEL_CIRCUMFERENCE / GEAR_RATIO; // meters per
                                                                                                        // second
            public static final double POSITION_FACTOR = WHEEL_CIRCUMFERENCE / GEAR_RATIO; // meters
            public static final double VELOCITY_FACTOR = POSITION_FACTOR / 60.0; // meters per second

            public static final double P = 0.04;
            public static final double I = 0.0;
            public static final double D = 0.0;
            public static final double FF = 1.0 / FREE_SPEED;
            public static final double MIN = -1.0;
            public static final double MAX = 1.0;

            public static final IdleMode IDLE_MODE = IdleMode.kBrake;

            public static final int STALL_CURRENT_LIMIT = 50;
            public static final int FREE_CURRENT_LIMIT = 20;
            public static final double SECONDARY_CURRENT_LIMIT = 60.0;
        }

        private static class Turning {
            public static final boolean ENCODER_INVERTED = true;

            public static final double POSITION_FACTOR = 2 * Math.PI; // radians
            public static final double VELOCITY_FACTOR = POSITION_FACTOR / 60.0; // radians per second
            public static final double WRAPPING_MIN = 0; // radians
            public static final double WRAPPING_MAX = POSITION_FACTOR; // radians

            public static final double P = 1.0;
            public static final double I = 0.0;
            public static final double D = 0.0;
            public static final double FF = 0.0;
            public static final double MIN = -1.0;
            public static final double MAX = 1.0;

            public static final IdleMode IDLE_MODE = IdleMode.kBrake;

            public static final int STALL_CURRENT_LIMIT = 20;
            public static final int FREE_CURRENT_LIMIT = 10;
            public static final double SECONDARY_CURRENT_LIMIT = 30.0;
        }
    }

    private final CANSparkMax drivingMotor;
    private final RelativeEncoder drivingEncoder;
    private final SparkMaxPIDController drivingController;

    private final CANSparkMax turningMotor;
    private final AbsoluteEncoder turningEncoder;
    private final SparkMaxPIDController turningController;

    private final double angleOffset;
    private final String name;

    @AutoLogOutput(key = "Drive/Modules/{name}/Target")
    private SwerveModuleState desiredState;

    public SwerveModule(SwerveModuleConfiguration configuration) {
        drivingMotor = new CANSparkMax(configuration.drivingID, MotorType.kBrushless);
        drivingMotor.restoreFactoryDefaults();
        drivingMotor.setIdleMode(Constants.Driving.IDLE_MODE);
        drivingMotor.setSmartCurrentLimit(Constants.Driving.STALL_CURRENT_LIMIT, Constants.Driving.FREE_CURRENT_LIMIT);
        drivingMotor.setSecondaryCurrentLimit(Constants.Driving.SECONDARY_CURRENT_LIMIT);
        drivingEncoder = drivingMotor.getEncoder();
        drivingEncoder.setPositionConversionFactor(Constants.Driving.POSITION_FACTOR);
        drivingEncoder.setVelocityConversionFactor(Constants.Driving.VELOCITY_FACTOR);
        drivingEncoder.setPosition(0);
        drivingController = drivingMotor.getPIDController();
        drivingController.setFeedbackDevice(drivingEncoder);
        drivingController.setP(Constants.Driving.P);
        drivingController.setI(Constants.Driving.I);
        drivingController.setD(Constants.Driving.D);
        drivingController.setFF(Constants.Driving.FF);
        drivingController.setOutputRange(Constants.Driving.MIN, Constants.Driving.MAX);

        turningMotor = new CANSparkMax(configuration.turningID, MotorType.kBrushless);
        turningMotor.restoreFactoryDefaults();
        turningMotor.setIdleMode(Constants.Turning.IDLE_MODE);
        turningMotor.setSmartCurrentLimit(Constants.Turning.STALL_CURRENT_LIMIT, Constants.Turning.FREE_CURRENT_LIMIT);
        turningMotor.setSecondaryCurrentLimit(Constants.Turning.SECONDARY_CURRENT_LIMIT);
        turningEncoder = turningMotor.getAbsoluteEncoder(Type.kDutyCycle);
        turningEncoder.setPositionConversionFactor(Constants.Turning.POSITION_FACTOR);
        turningEncoder.setVelocityConversionFactor(Constants.Turning.VELOCITY_FACTOR);
        turningEncoder.setInverted(Constants.Turning.ENCODER_INVERTED);
        turningController = turningMotor.getPIDController();
        turningController.setFeedbackDevice(turningEncoder);
        turningController.setPositionPIDWrappingEnabled(true);
        turningController.setPositionPIDWrappingMinInput(Constants.Turning.WRAPPING_MIN);
        turningController.setPositionPIDWrappingMaxInput(Constants.Turning.WRAPPING_MAX);
        turningController.setP(Constants.Turning.P);
        turningController.setI(Constants.Turning.I);
        turningController.setD(Constants.Turning.D);
        turningController.setFF(Constants.Turning.FF);
        turningController.setOutputRange(Constants.Turning.MIN, Constants.Turning.MAX);

        angleOffset = configuration.angleOffset;
        name = configuration.name;
        desiredState = new SwerveModuleState(0.0, new Rotation2d(turningEncoder.getPosition()));

        drivingMotor.burnFlash();
        turningMotor.burnFlash();

        // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
        drivingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        drivingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        drivingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        drivingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        drivingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
        drivingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
        drivingMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);
        turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
        turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
        turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);
    }

    public void setDesiredState(SwerveModuleState state) {
        state.angle = state.angle.plus(Rotation2d.fromRadians(angleOffset));
        state = SwerveModuleState.optimize(state, new Rotation2d(turningEncoder.getPosition()));

        drivingController.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        turningController.setReference(state.angle.getRadians(), CANSparkMax.ControlType.kPosition);

        desiredState = state;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                drivingEncoder.getPosition(),
                new Rotation2d(turningEncoder.getPosition() - angleOffset));
    }

    @AutoLogOutput(key = "Drive/Modules/{name}/Actual")
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                drivingEncoder.getVelocity(),
                new Rotation2d(turningEncoder.getPosition() - angleOffset));
    }
}
