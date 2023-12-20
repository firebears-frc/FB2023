package frc.robot.util.sparkmax;

import com.revrobotics.CANSparkMax;

public interface CurrentLimitConfiguration {
    public void apply(CANSparkMax motor);

    public static CurrentLimitConfiguration simple(int smartLimit, double secondaryLimit) {
        return new CurrentLimitConfiguration() {
            @Override
            public void apply(CANSparkMax motor) {
                Util.configure(motor::setSmartCurrentLimit, smartLimit, "smartCurrentLimit");
                Util.configure(motor::setSecondaryCurrentLimit, secondaryLimit, "secondaryCurrentLimit");
            }
        };
    }

    public static CurrentLimitConfiguration complex(int stallLimit, int freeLimit, int rpmCutoff,
            double secondaryLimit) {
        return new CurrentLimitConfiguration() {
            @Override
            public void apply(CANSparkMax motor) {
                Util.configure(ignored -> motor.setSmartCurrentLimit(stallLimit, freeLimit, rpmCutoff), this,
                        "smartCurrentLimit");
                Util.configure(motor::setSecondaryCurrentLimit, secondaryLimit, "secondaryCurrentLimit");
            }
        };
    }
}
