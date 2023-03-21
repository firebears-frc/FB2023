// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Schlucker;

public class AutoCubeAndBalanceCommand extends SequentialCommandGroup {
  Chassis m_chassis;
  Schlucker m_schlucker;
  Arm m_arm;

  public AutoCubeAndBalanceCommand(Chassis chassis, Schlucker schlucker, Arm arm) {
    m_chassis = chassis;
    m_schlucker = schlucker;
    m_arm = arm;
    addCommands(
        (new InstantCommand(m_schlucker::intakeCube, m_schlucker)),
        (new InstantCommand(m_schlucker::hold, m_schlucker)),
        (new WaitCommand(1)),
        (new ArmShoulderSetpointCommand(122, m_arm)),
        (new ArmElbowSetpointCommand(360, m_arm)),
        (new WaitCommand(2)),
        (new ArmElbowSetpointCommand(338, m_arm)),
        (new WaitCommand(1)),
        (new InstantCommand(m_schlucker::eject, m_schlucker)),
        (new ArmElbowSetpointCommand(350, m_arm)),
        (new WaitCommand(0.5)),
        (new InstantCommand(m_schlucker::stop, m_schlucker)),
        (new ChassisDriveToDistanceCommand(-0.5, 0.4, m_chassis)),
        (new ArmShoulderSetpointCommand(20, m_arm)),
        (new ArmElbowSetpointCommand(220, m_arm)),
        (new WaitCommand(1)),
        (new ChassisDriveToDistanceCommand(-3.5, 0.3, m_chassis)),
        (new AutoBalanceRoutine(m_chassis)))
      
        

    ;

  }
}