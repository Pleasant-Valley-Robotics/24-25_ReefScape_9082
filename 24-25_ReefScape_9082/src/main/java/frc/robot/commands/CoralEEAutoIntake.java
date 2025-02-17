// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEndEffector;
import frc.robot.subsystems.UtilitySensors;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralEEAutoIntake extends Command {
  CoralEndEffector coralEE;
  UtilitySensors sensors;
  /** Creates a new CoralEEAutoIntake. */
  public CoralEEAutoIntake(UtilitySensors sensors, CoralEndEffector coralEE) {
    this.sensors = sensors;
    this.coralEE = coralEE;

    addRequirements(sensors);
    addRequirements(coralEE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralEE.setVoltage(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralEE.setVoltage(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralEE.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(sensors.coralDetected()){
      return false;
    }
    else{
      return true;
    }
  }
}
