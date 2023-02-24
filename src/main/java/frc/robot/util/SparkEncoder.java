package frc.robot.util;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Wraps {@link RelativeEncoder} but also implements {@link Sendable}.
 * This allows this encoder to be displayed on Shuffleboard's
 * LiveWindow.
 */
public class SparkEncoder implements RelativeEncoder, Sendable {

    private final RelativeEncoder baseEncoder;
    private double encoderOffset = 0.0;

    public SparkEncoder(RelativeEncoder baseEncoder) {
        this.baseEncoder = baseEncoder;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Encoder");
        builder.addDoubleProperty("Velocity", this::getVelocity, null);
        builder.addDoubleProperty("Position", this::getPosition, null);
    }

    public void resetEncoder() {
		encoderOffset = baseEncoder.getPosition();
	}

    /** @return current offset in ticks. */
    public double getOffset() {
        return encoderOffset;
    }

    @Override
    /** @return current relative position, measured in ticks. */
    public double getPosition() {
        return baseEncoder.getPosition() - encoderOffset;
    }

    @Override
    public double getVelocity() {
        return baseEncoder.getVelocity();
    }

    @Override
    public REVLibError setPosition(double position) {
        return baseEncoder.setPosition(position);
    }

    @Override
    public REVLibError setPositionConversionFactor(double factor) {
        return baseEncoder.setPositionConversionFactor(factor);
    }

    @Override
    public REVLibError setVelocityConversionFactor(double factor) {
        return baseEncoder.setVelocityConversionFactor(factor);
    }

    @Override
    public double getPositionConversionFactor() {
        return baseEncoder.getPositionConversionFactor();
    }

    @Override
    public double getVelocityConversionFactor() {
        return baseEncoder.getVelocityConversionFactor();
    }

    @Override
    public REVLibError setAverageDepth(int depth) {
        return baseEncoder.setAverageDepth(depth);
    }

    @Override
    public int getAverageDepth() {
        return baseEncoder.getAverageDepth();
    }

    @Override
    public REVLibError setMeasurementPeriod(int period_ms) {
        return baseEncoder.setMeasurementPeriod(period_ms);
    }

    @Override
    public int getMeasurementPeriod() {
        return baseEncoder.getMeasurementPeriod();
    }

    @Override
    public int getCountsPerRevolution() {
        return baseEncoder.getCountsPerRevolution();
    }

    @Override
    public REVLibError setInverted(boolean inverted) {
        return baseEncoder.setInverted(inverted);
    }

    @Override
    public boolean getInverted() {
        return baseEncoder.getInverted();
    }

    public double getRawEncoderPosition() {
        return baseEncoder.getPosition();
    }
}
