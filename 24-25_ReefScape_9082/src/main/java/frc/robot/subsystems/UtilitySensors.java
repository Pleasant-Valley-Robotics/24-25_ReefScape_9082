// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.wpilibj.Counter;



public class UtilitySensors extends SubsystemBase {
  /**
   * NAVX Integration
   */
  AHRS navx;
  double accelerationX;
  double accelerationY;
  double accelerationZ;
  double barometricPressure;
  double refreshRate;

  /**
  * LIDAR-lite example
  * This gets the pulse width of the LIDAR sensor and converts it to a distance 
  * Inspired by https://github.com/GirlsOfSteelRobotics/Docs/wiki/LIDAR-Lite-Distance-Sensor
  */
  private Counter LIDAR;
  final double offsetLIDAR  = 10; //offset for sensor. test with tape measure <-- note from Brandon: I don't know how to do this. This came from sample code, I hope it's right
  double distanceLIDAR = 0;


  /** Creates a new UtilitySensors. */
  public UtilitySensors() {
    navx = new AHRS(NavXComType.kMXP_UART);
    LIDAR = new Counter(0); //plug the lidar into PWM 0
    LIDAR.setMaxPeriod(1.00); //set the max period that can be measured
    LIDAR.setSemiPeriodMode(true); //Set the counter to period measurement
    LIDAR.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //NAVX Periodic
    accelerationX = navx.getRawAccelX();
    accelerationY = navx.getRawAccelY();
    accelerationZ = navx.getRawAccelZ();
    barometricPressure = navx.getBarometricPressure();
    refreshRate = navx.getActualUpdateRate();
    
    SmartDashboard.putNumber("NavX2-MXP X Acceleration", accelerationX);
    SmartDashboard.putNumber("NavX2-MXP Y Acceleration", accelerationY);
    SmartDashboard.putNumber("NavX2-MXP Z Acceleration", accelerationZ);
    SmartDashboard.putNumber("NavX2-MXP Barometric Pressure", barometricPressure);
    SmartDashboard.putNumber("NavX2-MXP Refresh Rate", refreshRate);

    //LIDAR Periodic
    if(LIDAR.get() < 1){
      distanceLIDAR = 0;
    }
    else{
      distanceLIDAR = (LIDAR.getPeriod()*1000000.0/10.0) - offsetLIDAR; //convert to distance. sensor is high 10 us for every centimeter. 
    }
    SmartDashboard.putNumber("Distance", distanceLIDAR); //put the distance on the dashboard

  }
}
