package frc.robot.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class AutoBalanceCommand extends CommandBase {
    private static final double BALANCE_SPEED = 0.375; // meters per second

    private final Drivetrain drivetrain;

    public AutoBalanceCommand(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        drivetrain.drive(new ChassisSpeeds(BALANCE_SPEED, 0, 0));
    }

    @Override
    public void execute() {
        if (!drivetrain.isNotPitching()) {
            // Charge station is moving, stop!
            drivetrain.drive(new ChassisSpeeds());
            return;
        }

        // Depending on what way the charge station is tipped, go to middle
        if (drivetrain.getPitchDegrees() > 0) {
            drivetrain.drive(new ChassisSpeeds(BALANCE_SPEED, 0, 0));
        } else {
            drivetrain.drive(new ChassisSpeeds(-1.0 * BALANCE_SPEED, 0, 0));
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds());
        if (!interrupted) {
            drivetrain.setX();
        }
    }

    @Override
    public boolean isFinished() {
        return drivetrain.isNotPitching() && drivetrain.isLevel();
    }
}
