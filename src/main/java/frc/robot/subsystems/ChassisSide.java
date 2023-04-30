package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class ChassisSide implements Sendable {
    public static class ChassisSideConstants {
        public static final int STALL_CURRENT_LIMIT = 30;
        public static final int FREE_CURRENT_LIMIT = 20;
        public static final double SECONDARY_CURRENT_LIMIT = 60.0;

        public static final double GEAR_RATIO = (52.0 / 10.0) * (68.0 / 30.0);
        public static final double WHEEL_DIAMETER = 0.2032; // 8 inches
        public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        public static final double METERS_PER_MOTOR_ROTATION = WHEEL_CIRCUMFERENCE / GEAR_RATIO;
        public static final double VELOCITY_CONVERSION_FACTOR = METERS_PER_MOTOR_ROTATION / 60.0; // The raw units are
                                                                                                  // RPM

        // Values spit out of sysid
        public static final double P = 0.011179;
        public static final double I = 0.0;
        public static final double D = 0.0;
    }

    private CANSparkMax frontMotor;
    private CANSparkMax backMotor;
    private RelativeEncoder encoder;
    private PIDController pid;
    private SimpleMotorFeedforward feedforward;

    public ChassisSide(int frontID, int backID, SimpleMotorFeedforward feedforward) {
        frontMotor = new CANSparkMax(frontID, MotorType.kBrushless);
        frontMotor.restoreFactoryDefaults();
        frontMotor.setInverted(true);
        frontMotor.setIdleMode(IdleMode.kCoast);
        frontMotor.setSmartCurrentLimit(ChassisSideConstants.STALL_CURRENT_LIMIT,
                ChassisSideConstants.FREE_CURRENT_LIMIT);
        frontMotor.setSecondaryCurrentLimit(ChassisSideConstants.SECONDARY_CURRENT_LIMIT);
        encoder = frontMotor.getEncoder();
        encoder.setPositionConversionFactor(ChassisSideConstants.METERS_PER_MOTOR_ROTATION);
        encoder.setVelocityConversionFactor(ChassisSideConstants.VELOCITY_CONVERSION_FACTOR);
        frontMotor.burnFlash();

        backMotor = new CANSparkMax(backID, MotorType.kBrushless);
        backMotor.restoreFactoryDefaults();
        backMotor.setInverted(true);
        backMotor.setIdleMode(IdleMode.kCoast);
        backMotor.setSmartCurrentLimit(ChassisSideConstants.STALL_CURRENT_LIMIT,
                ChassisSideConstants.FREE_CURRENT_LIMIT);
        backMotor.setSecondaryCurrentLimit(ChassisSideConstants.SECONDARY_CURRENT_LIMIT);
        backMotor.follow(frontMotor);
        backMotor.burnFlash();

        pid = new PIDController(ChassisSideConstants.P, ChassisSideConstants.I, ChassisSideConstants.D);

        setSetpoint(0.0);
        resetDistance();
    }

    /**
     * Set the target velocity of this side
     * 
     * @param speed in meters per second
     */
    public void setSetpoint(double speed) {
        pid.setSetpoint(speed);
    }

    /**
     * Get the target velocity of this side
     * 
     * @return meters per second
     */
    public double getSetpoint() {
        return pid.getSetpoint();
    }

    /**
     * Reset the distance to zero
     */
    public void resetDistance() {
        encoder.setPosition(0);
    }

    /**
     * Get the distance this side has traveled
     * 
     * @return meters
     */
    public double getDistance() {
        return encoder.getPosition();
    }

    /**
     * Get the velocity of this side
     * 
     * @return meters per second
     */
    public double getVelocity() {
        return encoder.getVelocity();
    }

    /**
     * Set the brake mode of this side
     * 
     * @param brakeMode true for brake mode, false for coast
     */
    public void setBrakeMode(boolean brakeMode) {
        IdleMode mode = brakeMode ? IdleMode.kBrake : IdleMode.kCoast;
        frontMotor.setIdleMode(mode);
        backMotor.setIdleMode(mode);
    }

    public double update() {
        double currentDistance = getDistance();
        double currentVelocity = getVelocity();

        double feedforwardVoltage = feedforward.calculate(pid.getSetpoint());
        double feedbackVoltage = pid.calculate(currentVelocity);
        double voltage = feedbackVoltage + feedforwardVoltage;

        frontMotor.setVoltage(voltage);

        return currentDistance;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ChassisSide");
        builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
        builder.addDoubleProperty("distance", this::getDistance, null);
        builder.addDoubleProperty("velocity", this::getVelocity, null);
    }
}
