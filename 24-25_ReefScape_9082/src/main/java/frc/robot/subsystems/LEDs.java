// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
  private static final int port = 0;
  private static final int length = 300;
  
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  /** Creates a new LEDs. */
  public LEDs() {
    led = new AddressableLED(Constants.LEDConstants.port);
    buffer = new AddressableLEDBuffer(Constants.LEDConstants.bufferLenght);
    led.setLength(Constants.LEDConstants.bufferLenght);
    led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    led.setData(buffer);
  }

  /**
   * Shows one color with no special effects.
   * @param color color you want the pattern to be. 
   */
  public void solidPattern(Color color) {
    LEDPattern desiredColor = LEDPattern.solid(color);
    desiredColor.applyTo(buffer);
    led.setData(buffer);
  }

  /**
   * Shows continous gradient of 2 colors. Pattern wraps around.
   * @param startAndEndColor color pattern will start and end with.
   * @param middleColor middle color.
   */
  public void continuousGradient(Color startAndEndColor, Color middleColor){
    LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, startAndEndColor, middleColor);
  }

  /**
   * Shows discontinuous gradient. Doesn't wrap around.
   */
  public void discontinuousGradient() {
    
  }
}
