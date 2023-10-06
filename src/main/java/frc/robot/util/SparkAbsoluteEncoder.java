package frc.robot.util;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Wraps {@link SparkMaxAbsoluteEncoder} but also implements {@link Sendable}.
 * This allows this encoder to be displayed on Shuffleboard's
 * LiveWindow.
 */
public class SparkAbsoluteEncoder implements AbsoluteEncoder, Sendable {

    private final AbsoluteEncoder baseEncoder;
    private double encoderOffset = 0.0;

    public SparkAbsoluteEncoder(AbsoluteEncoder baseEncoder) {
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

    @Override
    public REVLibError setZeroOffset(double offset) {
        return baseEncoder.setZeroOffset(offset);
    }

    @Override
    public double getZeroOffset() {
        return baseEncoder.getZeroOffset();
    }
}
