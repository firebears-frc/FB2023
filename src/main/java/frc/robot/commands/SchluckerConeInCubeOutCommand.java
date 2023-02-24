package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Schlucker;
import frc.robot.util.Constants.SchluckerConstants;

public class SchluckerConeInCubeOutCommand extends InstantCommand {
    public SchluckerConeInCubeOutCommand(Schlucker schlucker) {
        super(() -> schlucker.setSpeed(SchluckerConstants.CONE_IN_CUBE_OUT_SPEED), schlucker);
    }
}
