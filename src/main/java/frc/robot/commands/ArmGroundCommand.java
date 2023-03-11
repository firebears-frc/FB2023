package frc.robot.commands;

import frc.robot.subsystems.Arm;
import frc.robot.util.Constants.ArmConstants;

public class ArmGroundCommand extends ArmPositionCommand {
    public ArmGroundCommand(Arm arm) {
        super(arm, ArmConstants.ELBOW_GROUND, ArmConstants.SHOULDER_GROUND);
    }
}
