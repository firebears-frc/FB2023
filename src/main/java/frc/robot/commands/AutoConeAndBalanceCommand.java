// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Schlucker;

public class AutoConeAndBalanceCommand extends SequentialCommandGroup {
  DriveSubsystem m_chassis;
  Schlucker m_schlucker;
  Arm m_arm;

  public AutoConeAndBalanceCommand(DriveSubsystem chassis, Schlucker schlucker, Arm arm) {
    m_chassis = chassis;
    m_schlucker = schlucker;
    m_arm = arm;
    addCommands(
      (new InstantCommand(m_schlucker::intakeCone, m_schlucker)),
      (new InstantCommand(m_schlucker::hold, m_schlucker)),
      (new WaitCommand(0.25)),
      (new ArmShoulderSetpointCommand(112, m_arm)),
      (new ArmElbowSetpointCommand(355, m_arm)),
      (new WaitCommand(1)),
      (new ArmElbowSetpointCommand(329, m_arm)),
      (new WaitCommand(0.5)),
      (new InstantCommand(m_schlucker::eject, m_schlucker)),
      (new ArmElbowSetpointCommand(350, m_arm)),
      (new WaitCommand(0.5)),
      (new InstantCommand(m_schlucker::stop, m_schlucker)),
      m_chassis.getDriveCommand(new Pose2d(), List.of(), new Pose2d(-0.5, 0, new Rotation2d())),
      (new ArmShoulderSetpointCommand(20, m_arm)),
      (new ArmElbowSetpointCommand(220, m_arm)),
      (new WaitCommand(0.5)),
      m_chassis.getDriveCommand(new Pose2d(), List.of(), new Pose2d(-3.5, 0, new Rotation2d())),
      (new AutoBalanceRoutine(m_chassis)))
    
      

  ;

}
}