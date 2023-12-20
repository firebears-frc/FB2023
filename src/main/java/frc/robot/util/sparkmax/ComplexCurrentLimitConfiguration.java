package frc.robot.util.sparkmax;

import com.revrobotics.CANSparkMax;

public class ComplexCurrentLimitConfiguration {
    private final int smartLimitMin, smartLimitMax, smartLimitCutoff;
    private final double secondaryLimit;

    public ComplexCurrentLimitConfiguration(int smartLimitMin, int smartLimitMax, int smartLimitCutoff,
            double secondaryLimit) {
        this.smartLimitMin = smartLimitMin;
        this.smartLimitMax = smartLimitMax;
        this.smartLimitCutoff = smartLimitCutoff;
        this.secondaryLimit = secondaryLimit;
    }

    void apply(CANSparkMax motor) {
        Util.configure(config -> motor.setSmartCurrentLimit(config.smartLimitMin, config.smartLimitMax,
                config.smartLimitCutoff), this, "smartCurrentLimit");
        Util.configure(motor::setSecondaryCurrentLimit, secondaryLimit, "secondaryCurrentLimit");
    }
}
