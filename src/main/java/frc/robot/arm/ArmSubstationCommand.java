package frc.robot.arm;

import frc.robot.arm.Arm.ArmConstants;

public class ArmSubstationCommand extends ArmPositionCommand {
    public ArmSubstationCommand(Arm arm) {
        super(arm, ArmConstants.ELBOW_SUBSTATION, ArmConstants.SHOULDER_SUBSTATION);
    }
}
