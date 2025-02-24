// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEndEffector;
import edu.wpi.first.wpilibj.Timer;

public class CoralEEAutoOuttake extends Command {
  private final double voltage;
  private final double timeToStop;
  private final CoralEndEffector coralEE;
  private Timer timer;
  
  /**
   * This command will automatically drive the coral out of the end effector
   * and into the reef scoring options. This just drives a wheel at a provided
   * voltage until a provided timeout value.
   * @param coralEE the CoralEndEffector subsystem
   * @param voltage the voltage to drive at
   * @param timeToStop how long to run this command
   */
  public CoralEEAutoOuttake(CoralEndEffector coralEE, double voltage, double timeToStop) {
    this.coralEE = coralEE;
    this.voltage = voltage;
    this.timeToStop = timeToStop;
    this.timer = new Timer();
    addRequirements(coralEE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralEE.setVoltage(0);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralEE.setVoltage(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralEE.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(timeToStop);
  }
}
