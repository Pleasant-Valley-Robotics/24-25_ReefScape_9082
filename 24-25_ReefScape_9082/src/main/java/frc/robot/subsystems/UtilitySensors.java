// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class UtilitySensors extends SubsystemBase {
  private AnalogInput coralDetector; 
  private HttpCamera limelightfeed; 
  private DigitalInput heightLimitSwitch;

  /** Creates a new UtilitySensors subsystem that sets up the camera tab and starts the feed so drivers can see the camera output, creates the sensor object, 
   * can return the value of a REV modern optical sensor to see if a coral is in the intake , and logs the distance returned from the sensor.
  */
  public UtilitySensors() {
    coralDetector = new AnalogInput(0);
    heightLimitSwitch = new DigitalInput(0);
    limelightfeed = new HttpCamera("limelight", "http://10.90.82.11:5801");
    ShuffleboardTab dashboardTab = Shuffleboard.getTab("Dash"); 
    dashboardTab.addCamera("limelight", "limelight", "http://10.90.82.11:5801");
  }

  /**
   * Periodically log the modern distance sensor to SmartDashboard.  
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Coral Detector Output", coralDetector.getValue());
    SmartDashboard.putBoolean("HeightLimitSwitch", heightLimitSwitch.get());
  }

  /**
   * Grab the value of the modern distance sensor. 
   * @return the value of the modern distance sensor. 
   */
  public double coralDetectorValue(){
    return coralDetector.getValue();
  }

  /**
   * Check and see if the sensor's seen the coral yet. 
   * @return whether the sensor's seen the coral. 
   */
  public boolean coralDetected(){
    if(coralDetector.getValue() > 250){
      return true;
    }
    else{
    return false;
    }
  }

  /**
   * Detect if the lift has reached the maximum value it's allowed to go to in code.
   * @return if liftHeight has been reached. 
   */
  public Boolean liftHeightReached(){
    return heightLimitSwitch.get();
  }
}
