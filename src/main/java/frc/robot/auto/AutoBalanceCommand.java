package frc.robot.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.chassis.Chassis;

public class AutoBalanceCommand extends CommandBase {
    public static class BalanceConstants {
        public static final double PITCH_TOLERANCE = 2.0; // degrees
        public static final double PITCH_VELOCITY_MAX = 0.2; // degrees per cycle

        public static final double BALANCE_SPEED = 0.375; // meters per second
    }

    private final Chassis chassis;

    public AutoBalanceCommand(Chassis chassis) {
        this.chassis = chassis;
    }

    @Override
    public void initialize() {
        chassis.drive(new ChassisSpeeds(BalanceConstants.BALANCE_SPEED, 0, 0));
    }

    @Override
    public void execute() {
        if (Math.abs(chassis.getPitchVelocity()) >= BalanceConstants.PITCH_VELOCITY_MAX) {
            // Charge station is moving, stop!
            chassis.drive(new ChassisSpeeds());
            return;
        }

        // Depending on what way the charge station is tipped, go to middle
        if (chassis.getPitch() > 0) {
            chassis.drive(new ChassisSpeeds(BalanceConstants.BALANCE_SPEED, 0, 0));
        } else {
            chassis.drive(new ChassisSpeeds(-1.0 * BalanceConstants.BALANCE_SPEED, 0, 0));
        }
    }

    @Override
    public void end(boolean interrupted) {
        chassis.drive(new ChassisSpeeds());
        if (!interrupted) {
            chassis.setBrakeMode(true);
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(chassis.getPitch()) < BalanceConstants.PITCH_TOLERANCE
                && Math.abs(chassis.getPitchVelocity()) < BalanceConstants.PITCH_VELOCITY_MAX;
                // TODO: Move to chassis for use in teleop
    }
}
