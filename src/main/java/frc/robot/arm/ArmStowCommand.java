package frc.robot.arm;

import frc.robot.arm.Arm.ArmConstants;

public class ArmStowCommand extends ArmPositionCommand {
    public ArmStowCommand(Arm arm) {
        super(arm, ArmConstants.ELBOW_STOW, ArmConstants.SHOULDER_STOW);
    }
}
