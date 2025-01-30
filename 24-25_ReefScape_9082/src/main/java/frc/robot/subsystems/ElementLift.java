// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.config.SparkBaseConfig.*;

import frc.robot.Constants.elementLiftConstants;

public class ElementLift extends SubsystemBase {

  private final SparkMax elementLift = new SparkMax(elementLiftConstants.elementLiftCAN, MotorType.kBrushed);
  SparkMaxConfig elementLiftConfig = new SparkMaxConfig();
  
  /** Creates a new ElementLift. */
  public ElementLift() {
    elementLiftConfig
    .idleMode(IdleMode.kBrake);
  elementLift.configure(elementLiftConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void lift(double speed){
    elementLift.set(speed);
  }
  public void liftVoltage(double voltage){
    elementLift.setVoltage(voltage);
  }
  public double getSetSpeed(){
    return elementLift.get();
  }
  public double getAppliedOutput(){
    return elementLift.getAppliedOutput();
  }
  public double getEncoderVelocity(){
    return elementLift.getEncoder().getVelocity();
  }
}
