package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.util.Constants.ArmConstants;

public class ArmSubstationCommand extends InstantCommand {
    public ArmSubstationCommand(Arm arm) {
        super(() -> arm.setAngles(ArmConstants.ELBOW_SUBSTATION,
                ArmConstants.SHOULDER_SUBSTATION), arm);
    }
}
