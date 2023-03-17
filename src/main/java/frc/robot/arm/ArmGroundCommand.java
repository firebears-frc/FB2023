package frc.robot.arm;

import frc.robot.arm.Arm.ArmConstants;

public class ArmGroundCommand extends ArmPositionCommand {
    public ArmGroundCommand(Arm arm) {
        super(arm, ArmConstants.ELBOW_GROUND, ArmConstants.SHOULDER_GROUND);
    }
}
