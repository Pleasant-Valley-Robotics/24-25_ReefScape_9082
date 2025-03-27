// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDs.ShowPattern;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;


public class UtilitySensors extends SubsystemBase {
  private AnalogInput coralDetector; 
  private AnalogInput level2HeightDetector; 
  private ShowPattern lastPattern = ShowPattern.liftProgress;

  /** Creates a new UtilitySensors subsystem that sets up the camera tab and starts the feed so drivers can see the camera output, creates the sensor object, 
   * can return the value of a REV modern optical sensor to see if a coral is in the intake , and logs the distance returned from the sensor.
  */
  public UtilitySensors() {
    coralDetector = new AnalogInput(0);
    level2HeightDetector = new AnalogInput(1);
  }

  /**
   * Periodically log the modern distance sensor to SmartDashboard.  
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Coral Detector Output", coralDetector.getValue());
    SmartDashboard.putBoolean("Coral Detected", coralDetected());
    SmartDashboard.putNumber("level2HeightDetector Output", level2HeightDetector.getValue());
    SmartDashboard.putBoolean("Level2 detected", Level2HeightDetected());
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
      if (RobotContainer.LEDs.showPattern != ShowPattern.solidRed)
      {
        lastPattern = RobotContainer.LEDs.showPattern;
        RobotContainer.LEDs.showPattern = ShowPattern.solidRed;
      }
      return true;
    }
    else{
      if(RobotContainer.LEDs.showPattern == ShowPattern.solidRed){
        RobotContainer.LEDs.showPattern = lastPattern;
      }
    return false;
    }
  }

  public boolean Level2HeightDetected(){
   if (level2HeightDetector.getValue() > 300) {
    return true;
   }
   else {
    return false; 
   }
  }
}
