package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmLigament {
    protected Rotation2d setpoint;
    protected Rotation2d position;

    public void setAngle(Rotation2d angle) {
        setpoint = angle;
    }

    public Rotation2d getTargetAngle() {
        return setpoint;
    }

    public Rotation2d getAngle() {
        return position;
    }

    public Rotation2d getError() {
        return getAngle().minus(getTargetAngle());
    }
}
