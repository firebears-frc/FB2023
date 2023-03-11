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
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speed = 0.35;
    lastPitch = m_chassis.getPitch();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitchVelocity = m_chassis.getpitchVelocity();
    double pitchSpeed = Math.abs(pitchVelocity);

    if (pitchVelocity > -0.5 ){
      m_chassis.arcadeDrive(speed, 0);
    }else{
      m_chassis.arcadeDrive(0, 0);
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
    return m_chassis.getpitchVelocity() <= -0.5;
  }
}
