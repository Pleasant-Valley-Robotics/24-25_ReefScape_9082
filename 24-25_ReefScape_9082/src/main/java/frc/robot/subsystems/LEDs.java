// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//LED docs link https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;
 
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
  private static final int port = 0;
  private static final int length = 300;

  Color blueColor = new Color(0,0,255);
  Color yellowColor = new Color(255, 255, 0);

  LEDPattern scrollingRainbow = LEDPattern.rainbow(255,5).scrollAtRelativeSpeed(Percent.per(Second).of(25)); 
  
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
    
    if(DriverStation.getAlliance().equals(Alliance.Blue)){
      showBreatheBlue();
    }
    else{
      showBreatheRed();
    }
    //showBreathe(blueColor, yellowColor);
    //scrollingRainbow.applyTo(buffer);
    // This method will be called once per scheduler run
    led.setData(buffer);
  }

  //To create patterns.
  //-----------------------------------------------------------------

  /**
   * Shows one color with no special effects.
   * @param color color you want the pattern to be. 
   */
  public void showSolidPattern(Color color) {
    LEDPattern desiredColor = LEDPattern.solid(color);
    desiredColor.applyTo(buffer);
    led.setData(buffer);
  }

  /**
   * Shows continous gradient of 2 colors. Pattern wraps around.
   * @param startAndEndColor color pattern will start and end with.
   * @param middleColor middle color.
   */
  public void showContinuousGradient(Color blueColor, Color yellowColor){
    LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, blueColor, yellowColor).atBrightness(Percent.of(50));
    gradient.applyTo(buffer); 
    led.setData(buffer);
  }

  /**
   * Shows discontinuous gradient. Doesn't wrap around.
   * @param firstColor first color to be shown on pattern.
   * @param lastColor second color to be shown on pattern.
   */
  public void showDiscontinuousGradient(Color firstColor, Color lastColor) {
    LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, firstColor, lastColor);
    gradient.applyTo(buffer);
    led.setData(buffer);
  }

  /**
   * Shows a step pattern with half one color and half a different color.
   * @param startingColor color for first half of step.
   * @param endingColor color for last half of the step.
   */
  public void showSteps(Color startingColor, Color endingColor){
    LEDPattern steps = LEDPattern.steps(Map.of(0, startingColor, 0.5, endingColor));
    steps.applyTo(buffer);
    led.setData(buffer);
  }

 /**
  * Shows percent of progress via leds using how far you are in getting to the goal and the goal(in numbers). Uses black and white.
  * @param currentProgress what robot's at now(current position or a current sensor value).
  * @param goal value the robot should go to(a target distance or a target sensor reading).
  */
  public void showDefaultProgressMask(double currentProgress, double goal) {
    LEDPattern pattern = LEDPattern.progressMaskLayer(() -> currentProgress / goal);
    pattern.applyTo(buffer);
    led.setData(buffer);
  }

  public void showBreatheBlue(){
    LEDPattern base = LEDPattern.solid(blueColor).atBrightness(Percent.of(100));
    LEDPattern pattern = base.breathe(Seconds.of(2));

    // Apply the LED pattern to the data buffer
    pattern.applyTo(buffer);

    // Write the data to the LED strip
    led.setData(buffer);
  }

  public void showBreatheRed() {
    LEDPattern base = LEDPattern.solid(blueColor).atBrightness(Percent.of(100));
    LEDPattern pattern = base.breathe(Seconds.of(2));

    // Apply the LED pattern to the data buffer
    pattern.applyTo(buffer);

    // Write the data to the LED strip
    led.setData(buffer);
  }

  //To create  patterns
  //-----------------------------------------------------------------
}
