package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.util.Constants.ChassisConstants;

public class ChassisSide implements Sendable {
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
        frontMotor.setSmartCurrentLimit(ChassisConstants.STALL_CURRENT_LIMIT, ChassisConstants.FREE_CURRENT_LIMIT);
        frontMotor.setSecondaryCurrentLimit(ChassisConstants.SECONDARY_CURRENT_LIMIT);
        encoder = frontMotor.getEncoder();
        encoder.setPositionConversionFactor(ChassisConstants.METERS_PER_MOTOR_ROTATION);
        encoder.setVelocityConversionFactor(ChassisConstants.VELOCITY_CONVERSION_FACTOR);
        frontMotor.burnFlash();
        backMotor = new CANSparkMax(backID, MotorType.kBrushless);
        backMotor.restoreFactoryDefaults();
        backMotor.setInverted(true);
        backMotor.setIdleMode(IdleMode.kCoast);
        backMotor.setSmartCurrentLimit(ChassisConstants.STALL_CURRENT_LIMIT, ChassisConstants.FREE_CURRENT_LIMIT);
        backMotor.setSecondaryCurrentLimit(ChassisConstants.SECONDARY_CURRENT_LIMIT);
        backMotor.follow(frontMotor);
        backMotor.burnFlash();
    
        pid = new PIDController(ChassisConstants.P, ChassisConstants.I, ChassisConstants.D);

        setSetpoint(0.0);
        resetDistance();
    }

    /**
     * Set the target velocity of this side
     * @param speed in meters per second
     */
    public void setSetpoint(double speed) {
        pid.setSetpoint(speed);
    }

    /**
     * Get the target velocity of this side
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
     * @return meters
     */
    public double getDistance() {
        return encoder.getPosition();
    }

    /**
     * Get the velocity of this side
     * @return meters per second
     */
    public double getVelocity() {
        return encoder.getVelocity();
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
