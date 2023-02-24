package frc.robot.util;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Extends {@link CANSparkMax} but also implements {@link Sendable}.
 * This allows this motor controller to be displayed on Shuffleboard's
 * LiveWindow.
 */
public class SparkMotor extends CANSparkMax implements Sendable {

    public SparkMotor(int deviceId, MotorType type) {
        super(deviceId, type);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motor Controller");
        builder.setActuator(true);
        builder.setSafeState(this::stopMotor);
        builder.addDoubleProperty("Value", this::get, this::set);
    }

}
