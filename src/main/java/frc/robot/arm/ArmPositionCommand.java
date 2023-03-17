package frc.robot.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmPositionCommand extends CommandBase {
    private final Arm arm;
    private final double elbowSetpoint;
    private final double shoulderSetpoint;

    public ArmPositionCommand(Arm arm, double elbowSetpoint, double shoulderSetpoint) {
        this.arm = arm;
        this.elbowSetpoint = elbowSetpoint;
        this.shoulderSetpoint = shoulderSetpoint;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setAngles(elbowSetpoint, shoulderSetpoint);
    }

    @Override
    public boolean isFinished() {
        return arm.onTarget();
    }
}
