package frc.robot.arm;

import frc.robot.arm.Arm.ArmConstants;

public class ArmMidCommand extends ArmPositionCommand {
    public ArmMidCommand(Arm arm) {
        super(arm, ArmConstants.ELBOW_MID, ArmConstants.SHOULDER_MID);
    }
}
