// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmManualCommand extends CommandBase {
  /** Creates a new ManualArmCommand. */

  private final Arm m_arm;
  private double shoulderSetPoint = 0;
  private double elbowSetPoint = 0;
  
  public ArmManualCommand(Arm subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = subsystem;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //XboxController xboxController=RobotContainer.getInstance().getxbox();
    shoulderSetPoint = 0;
    elbowSetPoint = 330;
    SmartDashboard.putBoolean("Runing Arm", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //shoulderSetPoint = xboxController.getRightY();
    //elbowSetPoint = xboxController.getLeftY();
    shoulderSetPoint = 0;
    elbowSetPoint = 270;
    m_arm.setElbowSetpoint(elbowSetPoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shoulderSetPoint = 0;
    elbowSetPoint = 330;
    SmartDashboard.putBoolean("Runing Arm", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(elbowSetPoint - m_arm.getElbowAngle()) < 5;

    //return m_arm.getShoulderAngle() <= shoulderSetPoint + 5 & m_arm.getShoulderAngle() >= shoulderSetPoint - 5 || m_arm.getElbowAngle() <= elbowSetPoint + 5 & m_arm.getElbowAngle() >= elbowSetPoint - 5;
  }
}
