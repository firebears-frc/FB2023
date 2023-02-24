package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Schlucker;
import frc.robot.util.Constants.SchluckerConstants;

public class SchluckerConeOutCubeInCommand extends InstantCommand {
    public SchluckerConeOutCubeInCommand(Schlucker schlucker) {
        super(() -> schlucker.setSpeed(SchluckerConstants.CONE_OUT_CUBE_IN_SPEED), schlucker);
    }
}
