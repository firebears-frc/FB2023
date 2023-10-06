// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Chassis;

public class AutonomousBalanceCommand extends CommandBase {
  Chassis m_chassis;
  Timer timer;
  double lastPitch;
  double speed;
  Timer timer2;
  boolean moveForward;
  /** Creates a new ChassisAutoBalanceCommand. */
  public AutonomousBalanceCommand(Chassis chassis) {
    m_chassis = chassis;
    timer = new Timer();
    timer2 = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastPitch = m_chassis.getPitch();
    timer2.reset();
    timer2.start();
    speed = 0.07;
    moveForward = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pitchVelocity = 0; //m_chassis.getpitchVelocity();
    double pitchSpeed = Math.abs(pitchVelocity);

    SmartDashboard.putNumber("Speed", speed);
    SmartDashboard.putNumber("Timer2", timer2.get());
    SmartDashboard.putNumber(
      "CounterRot", 
      pitchVelocity * Math.signum(m_chassis.getPitch())
    );
    if(pitchSpeed >= 0.3 && timer2.hasElapsed(1)) {
        speed = -0.05;
      moveForward = false;
    }

    if(m_chassis.getPitch() >= 2) {
      m_chassis.arcadeDrive(speed, 0);
      timer.reset();
    } else if(m_chassis.getPitch() <= -2) {
      m_chassis.arcadeDrive(-speed, 0);
      timer.reset();
    } else {
      timer.start();
      if (moveForward == false){
        speed = 0;
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
    return timer.hasElapsed(2);
  }
}