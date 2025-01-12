// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.KitBot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class KitBotShoot extends Command {
  private final double voltage;
  private final double timeToStop;
  private final KitBot kitBot;
  private Timer timer;
  /** Creates a new KitBotShoot. */
  public KitBotShoot(double voltage, double timeToStop, KitBot kitBotSubsystem) {
    this.voltage = voltage;
    this.timeToStop = timeToStop;
    this.kitBot = kitBotSubsystem;
    this.timer = new Timer();
    
    addRequirements(kitBot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kitBot.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    kitBot.shootVoltage(voltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    kitBot.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(timeToStop);
  }
}
