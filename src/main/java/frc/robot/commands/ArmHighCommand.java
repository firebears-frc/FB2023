package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;
import frc.robot.util.Constants.ArmConstants;

public class ArmHighCommand extends InstantCommand {
    public ArmHighCommand(Arm arm) {
        super(() -> arm.setAngles(ArmConstants.ELBOW_HIGH,
                ArmConstants.SHOULDER_HIGH), arm);
    }
}
