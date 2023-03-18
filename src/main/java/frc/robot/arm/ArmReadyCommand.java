package frc.robot.arm;

import frc.robot.arm.Arm.ArmConstants;

public class ArmReadyCommand extends ArmPositionCommand {
    public ArmReadyCommand(Arm arm) {
        super(arm, ArmConstants.ELBOW_READY, ArmConstants.SHOULDER_READY);
    }
}
