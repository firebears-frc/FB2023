// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class ChassisRotateToAngleCommand extends CommandBase {
  /** Creates a new DriveToPositionCommand. */
  private double rotation;
  private DriveSubsystem m_chassis;
  private double initialangle;
  private double testangle;

  public ChassisRotateToAngleCommand(double r, DriveSubsystem c) {
    rotation = r;
    m_chassis = c;
    testangle = m_chassis.getAngle();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialangle = m_chassis.getAngle();
    if (Constants.DEBUG) {
      SmartDashboard.putNumber("INIT DIFF", initialangle - testangle);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (rotation > 0) {
      m_chassis.arcadeDrive(0, 0.35);
    } else {
      m_chassis.arcadeDrive(0, -0.35);
    }
    if (Constants.DEBUG) {
      SmartDashboard.putNumber("currentangle", m_chassis.getAngle());
      SmartDashboard.putNumber("delta ang", rotation);
      SmartDashboard.putNumber("dersired angle", initialangle + rotation);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_chassis.arcadeDrive(0, 0);
    SmartDashboard.putNumber("DIFF", m_chassis.getAngle() - initialangle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (rotation > 0 && m_chassis.getAngle() > rotation + initialangle) {
      return true;
    } else if (rotation < 0 && m_chassis.getAngle() < rotation + initialangle) {
      return true;
    }
    return false;
  }
}