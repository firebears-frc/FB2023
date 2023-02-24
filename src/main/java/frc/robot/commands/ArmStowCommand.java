package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.util.Constants.ArmConstants;

public class ArmStowCommand extends InstantCommand {
    public ArmStowCommand(Arm arm) {
        super(() -> arm.setAngles(ArmConstants.ELBOW_STOW,
                ArmConstants.SHOULDER_STOW), arm);
    }
}
