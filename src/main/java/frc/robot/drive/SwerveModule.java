package frc.robot.drive;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.sparkmax.ComplexCurrentLimitConfiguration;
import frc.robot.util.sparkmax.SparkMaxConfiguration;
import frc.robot.util.sparkmax.StatusFrameConfiguration;

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

            public static final SparkMaxConfiguration CONFIG = new SparkMaxConfiguration(
                false,
                IdleMode.kBrake,
                new ComplexCurrentLimitConfiguration(50, 20, 10, 60.0),
                StatusFrameConfiguration.normal());
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

            public static final SparkMaxConfiguration CONFIG = new SparkMaxConfiguration(
                false,
                IdleMode.kBrake,
                new ComplexCurrentLimitConfiguration(20, 10, 10, 30.0),
                StatusFrameConfiguration.absoluteEncoder());
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
        Constants.Driving.CONFIG.apply(drivingMotor);
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
        Constants.Turning.CONFIG.apply(turningMotor);
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
    }

    public void setDesiredState(SwerveModuleState state) {
        state.angle = state.angle.plus(Rotation2d.fromRadians(angleOffset));
        state = SwerveModuleState.optimize(state, new Rotation2d(turningEncoder.getPosition()));

        drivingController.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        turningController.setReference(state.angle.getRadians(), CANSparkMax.ControlType.kPosition);

        desiredState = state;
    }

    @AutoLogOutput(key = "Drive/Modules/{name}/Position")
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
