// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElementLift;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElementLiftAutoHeight extends Command {
  private final double height;
  private final ElementLift elementLift;
  
  /**
   * Creates a command that will lift to the specified height by calling the lift subsystems goToHeight() method. 
   * Once it's done or interrupted motors are sent no more power. 
   * @param elementLift the lift subsystem 
   * @param height how high up to lift
   */
  public ElementLiftAutoHeight(ElementLift elementLift, double height) {
    this.elementLift = elementLift;
    this.height = height;
    
    addRequirements(elementLift);
  }

  // Called when the command is initially scheduled.
  /**
   * Stop lift from moving. 
   */
  @Override
  public void initialize() {
    elementLift.setVoltage(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * Drive lift to height every time the goToHeight() method's called. 
   */
  @Override
  public void execute() {
    elementLift.goToHeight(height);
  }

  // Called once the command ends or is interrupted.
  /**
   * Make lift not move. 
   */
  @Override
  public void end(boolean interrupted) {
    elementLift.setVoltage(0);
  }

  // Returns true when the command should end.
  /**
   * When the lift is at the height it's been commanded to go to end the command/return true. 
   */
  @Override
  public boolean isFinished() {
    return elementLift.atSetPoint();
  }
}
