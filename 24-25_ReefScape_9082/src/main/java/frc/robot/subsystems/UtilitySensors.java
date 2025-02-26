// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class UtilitySensors extends SubsystemBase {
  private AnalogInput coralDetector; 
  private HttpCamera limelightfeed; 

  /** Creates a new UtilitySensors subsystem that sets up the camera tab and starts the feed so drivers can see the camera output, creates the sensor object, 
   * can return the value of a REV modern optical sensor to see if a coral is in the intake , and logs the distance returned from the sensor.
  */
  public UtilitySensors() {
    coralDetector = new AnalogInput(0);
    limelightfeed = new HttpCamera("limelight", "http://10.90.82.11:5801");
    ShuffleboardTab dashboardTab = Shuffleboard.getTab("Dash"); 
    dashboardTab.addCamera("limelight", "limelight", "http://10.90.82.11:5801");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Coral Detector Output", coralDetector.getValue());
  }
  public double coralDetectorValue(){
    return coralDetector.getValue();
  }
  public boolean coralDetected(){
    if(coralDetector.getValue() > 250){
      return true;
    }
    else{
    return false;
    }
  }
}
