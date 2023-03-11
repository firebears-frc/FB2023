package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.util.Constants.ArmConstants;

public class ArmStowCommand extends ArmPositionCommand {
    public ArmStowCommand(Arm arm) {
        super(arm, ArmConstants.ELBOW_STOW, ArmConstants.SHOULDER_STOW);
    }
}
