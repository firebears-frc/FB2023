// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ChassisDriveToDistanceCommand extends CommandBase {
  /** Creates a new DriveToPositionCommand. */
  private double distance;
  private DriveSubsystem m_chassis;
  private double speed = 0.6;

  public ChassisDriveToDistanceCommand(double d, DriveSubsystem c) {
    distance = d;
    m_chassis = c;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_chassis);
  }

  public ChassisDriveToDistanceCommand(double d, double s, DriveSubsystem c) {
    distance = d;
    m_chassis = c;
    speed = s;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_chassis.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (distance > 0) {
      m_chassis.arcadeDrive(speed, 0);
    } else {
      m_chassis.arcadeDrive(-1 * speed, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_chassis.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
//return Math.abs(distance - m_chassis.getEncoderDistance()) < 0.1;

    if (distance > 0 && m_chassis.getEncoderDistance() > distance) {
      return true;
    } else if (distance < 0 && m_chassis.getEncoderDistance() < distance) {
      return true;
    }
    return false;
  }
}