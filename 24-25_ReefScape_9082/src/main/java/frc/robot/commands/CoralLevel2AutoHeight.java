// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEndEffector;
import frc.robot.subsystems.ElementLift;
import frc.robot.subsystems.UtilitySensors;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralLevel2AutoHeight extends Command {
  private final ElementLift elementLift;
  private final UtilitySensors utilitySensors;
  private final CoralEndEffector coralEE;
  private Timer timer;
  private boolean coralOuttakeStarted = false;
  private boolean coralOuttakeInProgress = false;

  /** Creates a new CoralLevel2AutoHeight. */
  public CoralLevel2AutoHeight(ElementLift elementLift, UtilitySensors utilitySensors, CoralEndEffector coralEE) {
    this.elementLift = elementLift;
    this.utilitySensors = utilitySensors;
    this.coralEE = coralEE;
    this.timer = new Timer();
    addRequirements(elementLift, utilitySensors,coralEE);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elementLift.setVoltage(0);
    coralEE.setVoltage(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!utilitySensors.Level2HeightDetected()){
      elementLift.setVoltage(5);
    }
    else{
      elementLift.setVoltage(0);
      coralOuttakeStarted = true;
    }
    if (coralOuttakeStarted && !coralOuttakeInProgress){
      coralEE.setVoltage(3);
      coralOuttakeInProgress = true;
      timer.reset();
      timer.start();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elementLift.setVoltage(0);
    coralEE.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(3);
  }
}
