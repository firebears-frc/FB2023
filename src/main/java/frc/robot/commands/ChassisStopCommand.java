// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ChassisStopCommand extends CommandBase {
  /** Creates a new ChassisStopCommand. */
  private DriveSubsystem m_chassis;
  private Timer timer;
  private double seconds;

  /**
   * Sets the breaks then waits 
   * @param seconds time in seconds.
   * @param chassis chassis subsystem.
   */
  public ChassisStopCommand(double seconds, DriveSubsystem chassis) {
    m_chassis = chassis;
    timer = new Timer();
    this.seconds = seconds;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_chassis.setBrakemode(true);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_chassis.arcadeDrive(0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(seconds);
  }
}
