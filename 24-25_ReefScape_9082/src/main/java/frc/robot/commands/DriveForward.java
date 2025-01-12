// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import static edu.wpi.first.units.Units.*;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveForward extends Command {
  private CommandSwerveDrivetrain swerve;
  private double distance;
  private double speed;
  private Timer timer = new Timer();
  private SwerveRequest.FieldCentric drive;
  private Pose2d startPosition;
  private double goal;

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /** Creates a new DriveForward. */
  public DriveForward(double meters, double speed, CommandSwerveDrivetrain swerveSubsystem, SwerveRequest.FieldCentric fieldCentric) {
    this.swerve = swerveSubsystem;
    this.speed = speed;
    this.distance = meters;
    this.drive = fieldCentric;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.applyRequest(() ->
      drive.withVelocityX(0 * MaxSpeed) // Drive forward with negative Y (forward)
      .withVelocityY(0 * MaxSpeed) // Drive left with negative X (left)
      .withRotationalRate(0 * MaxAngularRate)); // Drive counterclockwise with negative X (left)
      startPosition = swerve.getState().Pose;
      goal = startPosition.getX() + distance;

      timer.reset();
      timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.applyRequest(() ->
      drive.withVelocityX(speed) // Drive forward with negative Y (forward)
      .withVelocityY(0) // Drive left with negative X (left)
      .withRotationalRate(0)); // Drive counterclockwise with negative X (left)
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.applyRequest(() ->
      drive.withVelocityX(0) // Drive forward with negative Y (forward)
      .withVelocityY(0) // Drive left with negative X (left)
      .withRotationalRate(0)); // Drive counterclockwise with negative X (left)
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.hasElapsed(5))
    {
      return true;
    }
    else if (swerve.getState().Pose.getX() >= goal)
    {
      return true;
    }
    return false;
  }
}
