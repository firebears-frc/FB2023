package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmShoulderSetpointCommand extends InstantCommand {
    Arm m_arm;
    double setPoint;

    public ArmShoulderSetpointCommand(double s, Arm arm) {
        m_arm = arm;
        addRequirements(arm);
        setPoint = s;
    }

    @Override
    public void initialize() {
        m_arm.setShoulderSetpoint(setPoint);
    }
}
