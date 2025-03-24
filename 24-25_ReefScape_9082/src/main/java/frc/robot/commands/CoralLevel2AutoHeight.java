// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElementLift;
import frc.robot.subsystems.UtilitySensors;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralLevel2AutoHeight extends Command {
  private final ElementLift elementLift;
  private final UtilitySensors utilitySensors;
  /** Creates a new CoralLevel2AutoHeight. */
  public CoralLevel2AutoHeight(ElementLift elementLift, UtilitySensors utilitySensors) {
    this.elementLift = elementLift;
    this.utilitySensors = utilitySensors;
    addRequirements(elementLift);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elementLift.setVoltage(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elementLift.setVoltage(6);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elementLift.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return utilitySensors.Level2HeightDetected();
  }
}
