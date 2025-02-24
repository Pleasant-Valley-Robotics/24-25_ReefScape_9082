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
  private PIDController elementLiftController = new PIDController(elementLiftConstants.liftP,elementLiftConstants.liftI,elementLiftConstants.liftD);
  
  /**Makes a new lift object that can go to a set height with logic to keep it between 2 values(a min and a max voltage stated in Constants file), automatically and constantly keep the string tightened to avoid it slipping 
   * off the pulley and/or getting wrapped up in moving parts, manual controls if it's determined that the joystick's being moved 
   * enough to be a driver moving it, logging where the encoders are, what height the lift is at, grabbing encoder values, grabbing speed, grabbing current volts being sent to the motor, setting power, etc. 
  */
  public ElementLift() {
    elementLiftConfig
    .inverted(true)
    .idleMode(IdleMode.kBrake);
  elementLift.configure(elementLiftConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  elementLiftController.setTolerance(0.25);
  this.setPIDGains(elementLiftConstants.liftP,elementLiftConstants.liftI,elementLiftConstants.liftD);
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
    height = height - elementLiftConstants.liftOffset;  //Compensate for the height between the lift start position and the floor
    double voltage = elementLiftController.calculate(elementLift.getEncoder().getPosition()*elementLiftConstants.encoderToInches, height);
  
    //This logic is used for clamping
    if(height > 60){
      if(Math.abs(voltage) < elementLiftConstants.liftMinVoltage+1.375){
      voltage = (voltage/Math.abs(voltage))*(elementLiftConstants.liftMinVoltage+1.375);
      }
    }  
    else if (Math.abs(voltage) < elementLiftConstants.liftMinVoltage){
      voltage = (voltage/Math.abs(voltage))*elementLiftConstants.liftMinVoltage;
    }

    if (Math.abs(voltage) > elementLiftConstants.liftMaxVoltage){
      voltage = (voltage/Math.abs(voltage))*elementLiftConstants.liftMaxVoltage;
    }
    SmartDashboard.putNumber("Lift PID Voltage", voltage);
    SmartDashboard.putNumber("Lift Accumulated Error", elementLiftController.getAccumulatedError());
    elementLift.setVoltage(voltage);  //Actually drive the lift with set voltage
  }
  public boolean atSetPoint(){
    return elementLiftController.atSetpoint();
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
  public double getEncoderPosition(){
    return elementLift.getEncoder().getPosition();
  }
  public void resetEncoder(){
    elementLift.getEncoder().setPosition(0);
  }

}
