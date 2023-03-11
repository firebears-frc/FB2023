package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.util.Constants.ArmConstants;

public class ArmLowCommand extends ArmPositionCommand {
    public ArmLowCommand(Arm arm) {
        super(arm, ArmConstants.ELBOW_LOW, ArmConstants.SHOULDER_LOW);
    }
}
