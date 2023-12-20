package frc.robot.util.sparkmax;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

public class SparkMaxConfiguration {
    private final boolean inverted;
    private final IdleMode idleMode;
    private final CurrentLimitConfiguration currentLimits;
    private final StatusFrameConfiguration statusFrames;
    private final ClosedLoopConfiguration closedLoop;

    public SparkMaxConfiguration(boolean inverted, IdleMode idleMode, CurrentLimitConfiguration currentLimits,
            StatusFrameConfiguration statusFrames) {
        this.inverted = inverted;
        this.idleMode = idleMode;
        this.currentLimits = currentLimits;
        this.statusFrames = statusFrames;
        this.closedLoop = null;
    }

    public SparkMaxConfiguration(boolean inverted, IdleMode idleMode, CurrentLimitConfiguration currentLimits,
            StatusFrameConfiguration statusFrames, ClosedLoopConfiguration closedLoop) {
        this.inverted = inverted;
        this.idleMode = idleMode;
        this.currentLimits = currentLimits;
        this.statusFrames = statusFrames;
        this.closedLoop = closedLoop;
    }

    public void apply(CANSparkMax motor) {
        Util.configure(motor::restoreFactoryDefaults, false, "restoreFactoryDefaults");
        Util.configureAndVerify(motor::setInverted, motor::getInverted, inverted, "inverted");
        Util.configureCheckAndVerify(motor::setIdleMode, motor::getIdleMode, idleMode, "idleMode");
        currentLimits.apply(motor);
        statusFrames.apply(motor);
        if (closedLoop != null)
            closedLoop.apply(motor.getPIDController());
        Util.burnFlash(motor);
    }
}
