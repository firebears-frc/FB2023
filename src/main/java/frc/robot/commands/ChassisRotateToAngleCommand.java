// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

public class ChassisRotateToAngleCommand extends CommandBase {
  /** Creates a new DriveToPositionCommand. */
  private double rotation;
  private Chassis m_chassis;
  private double target;

  public ChassisRotateToAngleCommand(double r, Chassis c) {
    rotation = r;
    m_chassis = c;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = m_chassis.getYaw() + rotation;
    while(target > 180) {
      target -= 360;
    }
    while(target < -180) {
      target += 360;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_chassis.drive(new ChassisSpeeds(
      0,
      0,
      Math.copySign(Math.PI/2, rotation))
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_chassis.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(target - m_chassis.getYaw()) <= 5;
  }
}