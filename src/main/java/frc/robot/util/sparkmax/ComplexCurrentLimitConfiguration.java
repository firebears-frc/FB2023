package frc.robot.util.sparkmax;

import com.revrobotics.CANSparkMax;

public class ComplexCurrentLimitConfiguration implements CurrentLimitConfiguration {
    private final int stallLimit, freeLimit, rpmCutoff;
    private final double secondaryLimit;

    public ComplexCurrentLimitConfiguration(int stallLimit, int freeLimit, int rpmCutoff, double secondaryLimit) {
        this.stallLimit = stallLimit;
        this.freeLimit = freeLimit;
        this.rpmCutoff = rpmCutoff;
        this.secondaryLimit = secondaryLimit;
    }

    public void apply(CANSparkMax motor) {
        Util.configure(config -> motor.setSmartCurrentLimit(config.stallLimit, config.freeLimit, config.rpmCutoff),
                this, "smartCurrentLimit");
        Util.configure(motor::setSecondaryCurrentLimit, secondaryLimit, "secondaryCurrentLimit");
    }
}
