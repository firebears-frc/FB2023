package frc.robot.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chassis;

public class AutoBalanceCommand extends Command {
    private static final double BALANCE_SPEED = 0.375; // meters per second

    private final Chassis chassis;

    public AutoBalanceCommand(Chassis chassis) {
        this.chassis = chassis;
    }

    @Override
    public void initialize() {
        chassis.drive(new ChassisSpeeds(BALANCE_SPEED, 0, 0), false);
    }

    @Override
    public void execute() {
        if (!chassis.isNotPitching()) {
            // Charge station is moving, stop!
            chassis.setX();
            return;
        }

        // Depending on what way the charge station is tipped, go to middle
        if (chassis.getPitchDegrees() > 0) {
            chassis.drive(new ChassisSpeeds(BALANCE_SPEED, 0, 0), false);
        } else {
            chassis.drive(new ChassisSpeeds(-1.0 * BALANCE_SPEED, 0, 0), false);
        }
    }

    @Override
    public boolean isFinished() {
        return chassis.isNotPitching() && chassis.isLevel();
    }
}
