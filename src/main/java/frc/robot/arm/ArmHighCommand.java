package frc.robot.arm;

import frc.robot.arm.Arm.ArmConstants;

public class ArmHighCommand extends ArmPositionCommand {
    public ArmHighCommand(Arm arm) {
        super(arm, ArmConstants.ELBOW_HIGH, ArmConstants.SHOULDER_HIGH);
    }
}
