// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChassisSetBrakeMode extends InstantCommand {
  private DriveSubsystem m_chassis;
  private boolean brake;
  public ChassisSetBrakeMode(boolean b, DriveSubsystem c) {
    m_chassis = c;
    brake = b;
    addRequirements(m_chassis);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_chassis.setBrakemode(brake);
  }
}
