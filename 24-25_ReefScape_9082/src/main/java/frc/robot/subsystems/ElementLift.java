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
    SmartDashboard.putNumber("Lift Voltage", elementLift.getAppliedOutput());
  }

  /**
   * Set percentage of robot's battery that lift can use to power it.
   * @param speed percentage of robot battery to use. 
  */
  public void setSpeed(double speed){
    elementLift.set(speed);
  }

  /**
   * Send volts to lift to raise it. 
   * @param voltage
   */
  public void setVoltage(double voltage){
    elementLift.setVoltage(voltage);
  }

  /**
   * Set up PID controller.
   * @param P 
   * @param I
   * @param D 
   */
  public void setPIDGains(double P, double I, double D){
    elementLiftController.setPID(P,I,D);
  }

  /**
   * Calculate how much the lift should drive using the PID controller to allow for slowing down as getting closer to target. 
   * Keep the string tightened by staying between a min and max value. If a person's trying to move the joystick and not just the 
   * joystick getting shaken then move the lift.  
   * @param height desired height of robot not including the amount that it's already off the ground. 
   */
  public void goToHeight(double height){
    height = height - elementLiftConstants.liftOffset;  //Compensate for the height between the lift start position and the floor
    double voltage = elementLiftController.calculate(elementLift.getEncoder().getPosition()*elementLiftConstants.encoderToInches, height);
  
    //This logic is used for clamping
    if(height > 60){
      if(Math.abs(voltage) < elementLiftConstants.liftMinVoltage+1.375){
      voltage = (voltage/Math.abs(voltage))*(elementLiftConstants.liftMinVoltage+1.375);
      }
    }  
    //else if the voltage is below the min required to hold the string tight apply a fraction of max voltage allowed to turn the motor that tightens the string. 
    else if (Math.abs(voltage) < elementLiftConstants.liftMinVoltage){
      voltage = (voltage/Math.abs(voltage))*elementLiftConstants.liftMinVoltage;
    }
    //
    if (Math.abs(voltage) > elementLiftConstants.liftMaxVoltage){
      voltage = (voltage/Math.abs(voltage))*elementLiftConstants.liftMaxVoltage;
    }
    SmartDashboard.putNumber("Lift PID Voltage", voltage);
    SmartDashboard.putNumber("Lift Accumulated Error", elementLiftController.getAccumulatedError());
    elementLift.setVoltage(voltage);  //Actually drive the lift with set voltage
  }

  /**
   * Return whether the robot is at the height the PID loop calculated. 
   * @return whether lift's at the height calculated via PID loop. 
   */
  public boolean atSetPoint(){
    return elementLiftController.atSetpoint();
  }

  /**
   * Grab the percentage of battery being used by lift. 
   * @return percentage of battery used by lift. 
   */
  public double getSpeed(){
    return elementLift.get();
  }

  /**
   * Grab the volts of power being used by the lift motor. 
   * @return the volts of power being used by the lift motor. 
   */
  public double getAppliedOutput(){
    return elementLift.getAppliedOutput();
  }

  /**
   * Grab the encoder velocity. 
   * @return encoder velocity for lift encoder. 
   */
  public double getEncoderVelocity(){
    return elementLift.getEncoder().getVelocity();
  }

  /**
   * Grab the encoder postition for the lift/where it thinks it is. 
   * @return
   */
  public double getEncoderPosition(){
    return elementLift.getEncoder().getPosition();
  }

  /**
   * Reset the encoder position for the lift to 0.
   */
  public void resetEncoder(){
    elementLift.getEncoder().setPosition(0);
  }

  public PIDController getLiftPID() {
    return elementLiftController; 
  }
}
