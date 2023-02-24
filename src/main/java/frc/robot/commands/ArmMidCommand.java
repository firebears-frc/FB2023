package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.util.Constants.ArmConstants;

public class ArmMidCommand extends InstantCommand {
    public ArmMidCommand(Arm arm) {
        super(() -> arm.setAngles(ArmConstants.ELBOW_MID,
                ArmConstants.SHOULDER_MID), arm);
    }
}
