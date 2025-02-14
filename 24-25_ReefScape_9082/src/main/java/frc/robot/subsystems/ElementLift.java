// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.config.SparkBaseConfig.*;

import frc.robot.Constants.elementLiftConstants;

public class ElementLift extends SubsystemBase {

  private final SparkMax elementLift = new SparkMax(elementLiftConstants.elementLiftCAN, MotorType.kBrushless);
  private SparkMaxConfig elementLiftConfig = new SparkMaxConfig();
  private PIDController elementLiftController = new PIDController(.5,0,0);
  
  /** Creates a new ElementLift. */
  public ElementLift() {
    elementLiftConfig
    .inverted(true)
    .idleMode(IdleMode.kBrake);
  elementLift.configure(elementLiftConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Lift Encoder Value", elementLift.getEncoder().getPosition());
    SmartDashboard.putNumber("Lift Height", (elementLift.getEncoder().getPosition()*elementLiftConstants.encoderToInches)+elementLiftConstants.liftOffset);
  }
  public void setSpeed(double speed){
    elementLift.set(speed);
  }
  public void setVoltage(double voltage){
    elementLift.setVoltage(voltage);
  }
  public void setPIDGains(double P, double I, double D){
    elementLiftController.setPID(P,I,D);
  }
  public void goToHeight(double height){
    height = height - elementLiftConstants.liftOffset;
    double voltage = elementLiftController.calculate(elementLift.getEncoder().getPosition()*elementLiftConstants.encoderToInches,height);
    voltage = Math.min(elementLiftConstants.liftMaxVoltage, voltage);
    elementLift.setVoltage(voltage);
  }
  public double getSpeed(){
    return elementLift.get();
  }
  public double getAppliedOutput(){
    return elementLift.getAppliedOutput();
  }
  public double getEncoderVelocity(){
    return elementLift.getEncoder().getVelocity();
  }
  public void resetEncoder(){
    elementLift.getEncoder().setPosition(0);
  }

}
