package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

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
    public void initialize() {
        arm.setAngles(elbowSetpoint, shoulderSetpoint);
    }

    @Override
    public void 
}
