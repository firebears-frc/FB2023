package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmLigament {
    protected String name;
    protected Rotation2d setpoint;
    protected Rotation2d position;

    public void setAngle(Rotation2d angle) {
        setpoint = angle;
    }

    @AutoLogOutput(key = "Arm/{name}/Setpoint")
    public Rotation2d getTargetAngle() {
        return setpoint;
    }

    @AutoLogOutput(key = "Arm/{name}/Position")
    public Rotation2d getAngle() {
        return position;
    }

    @AutoLogOutput(key = "Arm/{name}/Error")
    public Rotation2d getError() {
        return getAngle().minus(getTargetAngle());
    }
}
