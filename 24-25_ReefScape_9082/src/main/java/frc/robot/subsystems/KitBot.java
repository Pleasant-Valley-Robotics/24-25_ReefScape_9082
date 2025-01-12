// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.*;
import com.revrobotics.config.*;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.config.SparkBaseConfig.*;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.kitBotConstants;

public class KitBot extends SubsystemBase {
  private final SparkMax shooter = new SparkMax(kitBotConstants.kitBotShooterCAN, MotorType.kBrushed);
  SparkMaxConfig shooterConfig = new SparkMaxConfig();
  

  /** Creates a new KitBot. */
  public KitBot() {
    shooterConfig
      .idleMode(IdleMode.kCoast);
    shooter.configure(shooterConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void shoot() {
    shooter.set(1);
  }
  public void stop() {
    shooter.set(0);
  }
}
