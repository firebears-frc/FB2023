package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.util.Constants.ArmConstants;

public class ArmMidCommand extends ArmPositionCommand {
    public ArmMidCommand(Arm arm) {
        super(arm, ArmConstants.ELBOW_MID, ArmConstants.SHOULDER_MID);
    }
}
