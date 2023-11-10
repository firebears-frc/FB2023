package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmManualCommand extends CommandBase {
    /** Creates a new ManualArmCommand. */

    private final Arm m_arm;
    private double shoulderSetPoint = 0;
    private double elbowSetPoint = 0;
    XboxController controller;

    public ArmManualCommand(Arm subsystem, XboxController x) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_arm = subsystem;
        controller = x;
        addRequirements(m_arm);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shoulderSetPoint = m_arm.getShoulderAngle();
    elbowSetPoint = m_arm.getElbowAngle();
  }

    @Override
    public void execute() {
        shoulderSetPoint = m_arm.getShoulderSetpoint();
        if (Math.abs(controller.getRightY()) > 0.1) {
            shoulderSetPoint -= controller.getRightY();
        }
        m_arm.setShoulderSetpoint(shoulderSetPoint);

        elbowSetPoint = m_arm.getElbowSetpoint();
        if (Math.abs(controller.getLeftY()) > 0.1) {
            elbowSetPoint -= controller.getLeftY();
        }
        m_arm.setElbowSetpoint(elbowSetPoint);

    }

    @Override
    public boolean isFinished() {
        return false;

        // return Math.abs(elbowSetPoint - m_arm.getElbowAngle()) < 5;
        // return m_arm.getShoulderAngle() <= shoulderSetPoint + 5 &
        // m_arm.getShoulderAngle() >= shoulderSetPoint - 5 || m_arm.getElbowAngle() <=
        // elbowSetPoint + 5 & m_arm.getElbowAngle() >= elbowSetPoint - 5;
    }
}
