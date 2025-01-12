// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.KitBot;

import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command kitBotAuto(CommandSwerveDrivetrain swerveSubsystem, KitBot kitBotSubsystem, FieldCentric fieldCentric) {
    return Commands.sequence(new DriveForward(1,1,swerveSubsystem, fieldCentric), new KitBotShoot(12, 3, kitBotSubsystem));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
