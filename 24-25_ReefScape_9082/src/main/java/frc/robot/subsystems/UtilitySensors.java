// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Counter;

public class UtilitySensors extends SubsystemBase {
  private AnalogInput coralDetector; 

  /** Creates a new UtilitySensors. */
  public UtilitySensors() {
    coralDetector = new AnalogInput(0);
  }

  @Override
  public void periodic() {
    
  }
}
