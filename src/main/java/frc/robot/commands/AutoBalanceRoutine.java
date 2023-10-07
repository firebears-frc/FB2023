// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;


public class AutoBalanceRoutine extends SequentialCommandGroup {
  /** Creates a new AutoBalanceRoutine. */
  private DriveSubsystem m_chassis;
  public AutoBalanceRoutine(DriveSubsystem c) {
    m_chassis = c;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ChassisSetBrakeMode(true, m_chassis),
      // new ChassisDriveToPitch(10, 0.2, m_chassis),
      new ChassisDriveToPitch(10, 0.2, m_chassis), new PrintCommand("PITCH"),
      m_chassis.getDriveCommand(new Pose2d(), List.of(), new Pose2d(0.65, 0, new Rotation2d())), new PrintCommand("DRIVEN"),
      new BalanceTake2Command(0.07, m_chassis),  new PrintCommand("BALANCED"),
    
     
      new ChassisStopCommand(3, m_chassis)
      //new ChassisDriveToDistanceCommand(-0.05, 0.15, m_chassis),
      //new WaitCommand(5)
    );
  }
}
