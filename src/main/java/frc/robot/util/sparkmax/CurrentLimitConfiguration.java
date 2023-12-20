package frc.robot.util.sparkmax;

import com.revrobotics.CANSparkMax;

public interface CurrentLimitConfiguration {
    void apply(CANSparkMax motor);
}
