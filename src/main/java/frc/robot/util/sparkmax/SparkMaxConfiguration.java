package frc.robot.util.sparkmax;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

public class SparkMaxConfiguration {
    private final boolean inverted;
    private final IdleMode idleMode;
    private final CurrentLimitConfiguration currentLimits;
    private final StatusFrameConfiguration statusFrames;
    // TODO: PID, encoder, etc

    public SparkMaxConfiguration(boolean inverted, IdleMode idleMode, CurrentLimitConfiguration currentLimits, StatusFrameConfiguration statusFrames) {
        this.inverted = inverted;
        this.idleMode = idleMode;
        this.currentLimits = currentLimits;
        this.statusFrames = statusFrames;
    }

    public void apply(CANSparkMax motor) {
        Util.configure(motor::restoreFactoryDefaults, false, "restoreFactoryDefaults");
        Util.configure(motor::setInverted, motor::getInverted, inverted, "inverted");
        Util.configure(motor::setIdleMode, motor::getIdleMode, idleMode, "idleMode");
        currentLimits.apply(motor);
        statusFrames.apply(motor);
        Util.burnFlash(motor);
    }
}
