package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.util.Constants.ArmConstants;

public class ArmLowCommand extends InstantCommand {
    public ArmLowCommand(Arm arm) {
        super(() -> arm.setAngles(ArmConstants.ELBOW_LOW,
                ArmConstants.SHOULDER_LOW), arm);
    }
}
