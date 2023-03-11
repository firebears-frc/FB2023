package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.util.Constants.ArmConstants;

public class ArmSubstationCommand extends ArmPositionCommand {
    public ArmSubstationCommand(Arm arm) {
        super(arm, ArmConstants.ELBOW_SUBSTATION, ArmConstants.SHOULDER_SUBSTATION);
    }
}
