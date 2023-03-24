// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Schlucker;

public class Pushups extends SequentialCommandGroup {
  Chassis m_chassis;
  Schlucker m_schlucker;
  Arm m_arm;

  public Pushups(Chassis chassis, Schlucker schlucker, Arm arm) {
    m_chassis = chassis;
    m_schlucker = schlucker;
    m_arm = arm;
    addCommands(
      (new ArmShoulderSetpointCommand(130, m_arm)),
      (new ArmElbowSetpointCommand(288, m_arm)),
      (new WaitCommand(4)),
      (new ArmElbowSetpointCommand(270, m_arm)),
      (new ArmShoulderSetpointCommand(143, m_arm)),
      (new WaitCommand(5)),
      (new ArmShoulderSetpointCommand(185, m_arm)),
      (new WaitCommand(2)),
      (new ArmShoulderSetpointCommand(143, m_arm)),

      (new WaitCommand(2)),
      (new ArmShoulderSetpointCommand(185, m_arm)),
      (new WaitCommand(2)),
      (new ArmShoulderSetpointCommand(143, m_arm)),

      (new WaitCommand(2)),
      (new ArmShoulderSetpointCommand(185, m_arm)),
      (new WaitCommand(2)),
      (new ArmShoulderSetpointCommand(143, m_arm)),

      (new WaitCommand(2)),
      (new ArmShoulderSetpointCommand(185, m_arm)),
      (new WaitCommand(2)),
      (new ArmShoulderSetpointCommand(143, m_arm)),

      (new WaitCommand(2)),
      (new ArmShoulderSetpointCommand(185, m_arm)),
      (new WaitCommand(2)),
      (new ArmShoulderSetpointCommand(143, m_arm)),

      (new WaitCommand(2)),
      (new ArmShoulderSetpointCommand(185, m_arm)),
      (new WaitCommand(2)),
      (new ArmShoulderSetpointCommand(143, m_arm))
      

      

  );











  }
  }