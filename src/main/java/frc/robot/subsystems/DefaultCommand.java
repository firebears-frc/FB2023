package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public class DefaultCommand extends Command {
    private static final class Constants {
        public static final double SLOW_TELE_VELOCITY = 1.0; // meters per second
        public static final double MAX_TELE_ANGULAR_VELOCITY = 1.5 * Math.PI; // radians per second
        public static final double SLOW_TELE_ANGULAR_VELOCITY = Math.PI / 2; // radians per second
    }

    private final Chassis chassis;
    private final Supplier<ChassisSpeeds> commandSupplier;
    private final boolean slowMode;
    private final boolean fieldRelative;
    private final boolean rateLimit;

    private final RateLimiter rateLimiter;

    public DefaultCommand(Chassis chassis, Supplier<ChassisSpeeds> commandSupplier,
            boolean slowMode, boolean fieldRelative, boolean rateLimit) {
        this.chassis = chassis;
        this.commandSupplier = commandSupplier;
        this.slowMode = slowMode;
        this.fieldRelative = fieldRelative;
        this.rateLimit = rateLimit;

        rateLimiter = new RateLimiter();

        addRequirements(chassis);
    }

    @Override
    public void execute() {
        ChassisSpeeds command = commandSupplier.get();

        if (rateLimit) {
            command = rateLimiter.calculate(command);
        }

        if (slowMode) {
            command.vxMetersPerSecond *= Constants.SLOW_TELE_VELOCITY;
            command.vyMetersPerSecond *= Constants.SLOW_TELE_VELOCITY;
            command.omegaRadiansPerSecond *= Constants.SLOW_TELE_ANGULAR_VELOCITY;
        } else {
            command.vxMetersPerSecond *= Chassis.MAX_VELOCITY;
            command.vyMetersPerSecond *= Chassis.MAX_VELOCITY;
            command.omegaRadiansPerSecond *= Constants.MAX_TELE_ANGULAR_VELOCITY;
        }

        chassis.drive(command, fieldRelative);
    }
}
