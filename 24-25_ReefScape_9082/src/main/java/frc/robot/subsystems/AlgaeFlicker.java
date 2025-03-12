// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeFlickerConstants;

public class AlgaeFlicker extends SubsystemBase {
  private final SparkMax algaeFlicker = new SparkMax(AlgaeFlickerConstants.AlgaeFlickerCAN, MotorType.kBrushed);
  private SparkMaxConfig algaeFlickerConfig = new SparkMaxConfig();

  /** Creates a new AlgaeFlicker. */
  public AlgaeFlicker() {
      algaeFlickerConfig
      .inverted(true)
      .idleMode(IdleMode.kCoast);
    algaeFlicker.configure(algaeFlickerConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("algaeFlicker Velocity", algaeFlicker.getEncoder().getVelocity());
    SmartDashboard.putNumber("algaeFlicker Applied Output", algaeFlicker.getAppliedOutput());
  }
}
