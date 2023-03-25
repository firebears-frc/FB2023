package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis;

public class ChassisDriveCommand extends CommandBase {

    private final Chassis m_chassis;

    public ChassisDriveCommand(Chassis subsystem) {

        m_chassis = subsystem;
        addRequirements(m_chassis);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        Joystick joystick = RobotContainer.getInstance().getJoystick();
        double speed = joystick.getY();
        if (Math.abs(speed) < 0.12)
            speed = 0;
        if (m_chassis.getSlowMode())
            speed *= 0.2;

        double rotation = joystick.getTwist();
        double rotMultiplier = ((-joystick.getThrottle() / 2 + 0.5f) * 0.70f) + 0.1f;
        m_chassis.arcadeDrive(-speed,(-rotation) * rotMultiplier);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
