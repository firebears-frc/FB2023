package frc.robot.util.sparkmax;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

public class SparkMaxClosedLoopConfiguration {
    private final ClosedLoopGainConfiguration gains;

    public SparkMaxClosedLoopConfiguration(ClosedLoopGainConfiguration gains) {
        this.gains = gains;
    }

    public SparkMaxPIDController apply(CANSparkMax motor) {
        SparkMaxPIDController pid = motor.getPIDController();
        gains.apply(pid);
        return pid;
    }
}
