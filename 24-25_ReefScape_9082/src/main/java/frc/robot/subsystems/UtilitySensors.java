// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

public class UtilitySensors extends SubsystemBase {
  AHRS navx;
  double accelerationX;
  double accelerationY;
  double accelerationZ;
  double barometricPressure;
  double refreshRate;
  /** Creates a new UtilitySensors. */
  public UtilitySensors() {
    navx = new AHRS(NavXComType.kMXP_UART);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
  }
}
