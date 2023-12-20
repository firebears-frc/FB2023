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
import frc.robot.util.sparkmax.ClosedLoopConfiguration;
import frc.robot.util.sparkmax.CurrentLimitConfiguration;
import frc.robot.util.sparkmax.FeedbackConfiguration;
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

            public static final SparkMaxConfiguration CONFIG = new SparkMaxConfiguration(
                    false,
                    IdleMode.kBrake,
                    CurrentLimitConfiguration.complex(50, 20, 10, 60.0),
                    StatusFrameConfiguration.normal(),
                    ClosedLoopConfiguration.simple(0.04, 0.0, 0.0, 1.0 / FREE_SPEED),
                    FeedbackConfiguration.builtInEncoder(false, POSITION_FACTOR));
        }

        private static class Turning {
            public static final SparkMaxConfiguration CONFIG = new SparkMaxConfiguration(
                    false,
                    IdleMode.kBrake,
                    CurrentLimitConfiguration.complex(20, 10, 10, 30.0),
                    StatusFrameConfiguration.absoluteEncoder(),
                    ClosedLoopConfiguration.wrapping(2.5, 0.0, 0.0, 0.0, 0, 2 * Math.PI),
                    FeedbackConfiguration.absoluteEncoder(true, 2 * Math.PI));
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
        drivingEncoder = drivingMotor.getEncoder();
        drivingController = drivingMotor.getPIDController();

        turningMotor = new CANSparkMax(configuration.turningID, MotorType.kBrushless);
        turningEncoder = turningMotor.getAbsoluteEncoder(Type.kDutyCycle);
        turningController = turningMotor.getPIDController();

        Constants.Driving.CONFIG.apply(drivingMotor);
        Constants.Turning.CONFIG.apply(turningMotor);

        drivingEncoder.setPosition(0);
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
