// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class BalanceTake2Command extends CommandBase {

  Chassis m_chassis;
  double speed;
  double lastPitch;

  /** Creates a new BalanceTake2Command. */
  public BalanceTake2Command(Chassis chassis) {
    m_chassis = chassis;
    speed = 0.07;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_chassis);
  }
  public BalanceTake2Command(double s, Chassis chassis) {
    m_chassis = chassis;
    speed = s;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastPitch = m_chassis.getPitch();
    m_chassis.setBrakemode(true);
    m_chassis.arcadeDrive(speed, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(m_chassis.getpitchVelocity()) > 0.15) {
      m_chassis.arcadeDrive(0, 0);
    } else {
      if (m_chassis.getPitch() > 0) {
        m_chassis.arcadeDrive(speed, 0);
      } else {
        m_chassis.arcadeDrive(-speed, 0);
      }
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
    return Math.abs(m_chassis.getPitch()) < 3 && Math.abs(m_chassis.getpitchVelocity()) < 0.15;
    //m_chassis.getPitch() < 5 || m_chassis.getpitchVelocity() < -0.3;
    //m_chassis.getpitchVelocity() <= -0.5;
  }
}
