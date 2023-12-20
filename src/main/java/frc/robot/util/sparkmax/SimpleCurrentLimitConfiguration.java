package frc.robot.util.sparkmax;

import com.revrobotics.CANSparkMax;

public class SimpleCurrentLimitConfiguration implements CurrentLimitConfiguration {
    private final int smartLimit;
    private final double secondaryLimit;

    public SimpleCurrentLimitConfiguration(int smartLimit, double secondaryLimit) {
        this.smartLimit = smartLimit;
        this.secondaryLimit = secondaryLimit;
    }

    public void apply(CANSparkMax motor) {
        Util.configure(motor::setSmartCurrentLimit, smartLimit, "smartCurrentLimit");
        Util.configure(motor::setSecondaryCurrentLimit, secondaryLimit, "secondaryCurrentLimit");
    }
}
