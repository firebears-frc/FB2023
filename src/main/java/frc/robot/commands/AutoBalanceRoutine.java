// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Chassis;


public class AutoBalanceRoutine extends SequentialCommandGroup {
  /** Creates a new AutoBalanceRoutine. */
  private Chassis m_chassis;
  public AutoBalanceRoutine(Chassis c) {
    m_chassis = c;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ChassisSetBrakeMode(true, m_chassis),
      new ChassisDriveToPitch(10, 0.2, m_chassis),
      new ChassisDriveToPitch(10, 0.2, m_chassis),
      new ChassisDriveToDistanceCommand(0.75, 0.2, m_chassis),
      new BalanceTake2Command(0.06, m_chassis),
      new ChassisDriveToDistanceCommand(-0.05, 0.15, m_chassis),
      new WaitCommand(5)
    );
  }
}
