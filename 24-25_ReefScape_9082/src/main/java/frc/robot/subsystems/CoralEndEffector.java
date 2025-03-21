// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.coralEEConstants;


public class CoralEndEffector extends SubsystemBase {
  private final SparkFlex coralEE = new SparkFlex(coralEEConstants.coralEECAN, MotorType.kBrushless);
  private SparkFlexConfig coralEEConfig = new SparkFlexConfig();

  /** Creates a new CoralEndEffector for intaking coral, depositing coral to score, logging coral velocity, and logging applied output 
   * to coral(the voltage the motor's running at). 
  */
  public CoralEndEffector() {
        coralEEConfig
    .inverted(false)
    .idleMode(IdleMode.kCoast);
  coralEE.configure(coralEEConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Periodically put intake's velocity and applied output into SmartDashboard.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run.
    SmartDashboard.putNumber("CoralEE Velocity", coralEE.getEncoder().getVelocity());
    SmartDashboard.putNumber("CoralEE Applied Output", coralEE.getAppliedOutput());
  }
  
  /**
   * Sets voltage/power supplied to robot. 
  */
  public void setVoltage(double voltage) {
    coralEE.setVoltage(voltage);
  }
}
