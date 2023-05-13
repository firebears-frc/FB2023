package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;

public class ChassisSide {
    public static class Constants {
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

    private final CANSparkMax frontMotor;
    private final CANSparkMax backMotor;
    private final RelativeEncoder encoder;
    private final PIDController pid;
    private final SimpleMotorFeedforward feedforward;
    private final DoubleLogEntry setpointLog;
    private final DoubleLogEntry distanceLog;
    private final DoubleLogEntry velocityLog;
    private final DoubleLogEntry feedforwardVoltageLog;
    private final DoubleLogEntry feedbackVoltageLog;
    private final DoubleLogEntry voltageLog;

    public ChassisSide(int frontID, int backID, boolean inverted, SimpleMotorFeedforward feedforward, DataLog log, String name) {
        frontMotor = new CANSparkMax(frontID, MotorType.kBrushless);
        frontMotor.restoreFactoryDefaults();
        frontMotor.setInverted(inverted);
        frontMotor.setIdleMode(IdleMode.kCoast);
        frontMotor.setSmartCurrentLimit(Constants.STALL_CURRENT_LIMIT,
                Constants.FREE_CURRENT_LIMIT);
        frontMotor.setSecondaryCurrentLimit(Constants.SECONDARY_CURRENT_LIMIT);
        encoder = frontMotor.getEncoder();
        encoder.setPositionConversionFactor(Constants.METERS_PER_MOTOR_ROTATION);
        encoder.setVelocityConversionFactor(Constants.VELOCITY_CONVERSION_FACTOR);

        backMotor = new CANSparkMax(backID, MotorType.kBrushless);
        backMotor.restoreFactoryDefaults();
        backMotor.setInverted(inverted);
        backMotor.setIdleMode(IdleMode.kCoast);
        backMotor.setSmartCurrentLimit(Constants.STALL_CURRENT_LIMIT,
                Constants.FREE_CURRENT_LIMIT);
        backMotor.setSecondaryCurrentLimit(Constants.SECONDARY_CURRENT_LIMIT);
        backMotor.follow(frontMotor);

        pid = new PIDController(Constants.P, Constants.I, Constants.D);

        this.feedforward = feedforward;
        setSetpoint(0.0);

        setpointLog = new DoubleLogEntry(log, "Chassis/" + name + "/Setpoint");
        distanceLog = new DoubleLogEntry(log, "Chassis/" + name + "/Position");
        velocityLog = new DoubleLogEntry(log, "Chassis/" + name + "/Velocity");
        feedforwardVoltageLog = new DoubleLogEntry(log, "Chassis/" + name + "/FeedforwardVoltage");
        feedbackVoltageLog = new DoubleLogEntry(log, "Chassis/" + name + "/FeedbackVoltage");
        voltageLog = new DoubleLogEntry(log, "Chassis/" + name + "/Voltage");

        frontMotor.burnFlash();
        backMotor.burnFlash();

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

        double setpoint = pid.getSetpoint();
        double feedforwardVoltage = feedforward.calculate(setpoint);
        double feedbackVoltage = pid.calculate(currentVelocity);
        double voltage = feedbackVoltage + feedforwardVoltage;

        frontMotor.setVoltage(voltage);
        backMotor.setVoltage(voltage);

        setpointLog.append(setpoint);
        distanceLog.append(currentDistance);
        velocityLog.append(currentVelocity);
        feedforwardVoltageLog.append(feedforwardVoltage);
        feedbackVoltageLog.append(feedbackVoltage);
        voltageLog.append(voltage);

        return currentDistance;
    }
}
