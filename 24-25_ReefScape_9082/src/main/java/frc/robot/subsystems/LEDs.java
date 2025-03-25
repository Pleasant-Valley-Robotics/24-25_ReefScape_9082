// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//LED docs link https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LEDs extends SubsystemBase {
  public enum ShowPattern {
    solidRed,
    liftProgress,
    solidWhite,
    solidGreen
  }

  public ShowPattern showPattern;

  Color blueColor = new Color(25, 0, 255); 
  Color yellowColor = new Color(45, 255, 0); 
  Color redColor = new Color(0, 255, 0);
  Color greenColor = new Color(255,0,0);
  Color whiteColor = new Color(255,255,255);

  LEDPattern solidRedPattern = LEDPattern.solid(redColor).atBrightness(Percent.of(25));
  LEDPattern solidWhitePattern = LEDPattern.solid(whiteColor).atBrightness(Percent.of(25));
  LEDPattern solidGreenPattern = LEDPattern.solid(greenColor).atBrightness(Percent.of(25));

  LEDPattern blueLiftMaskPattern = LEDPattern.solid(blueColor).atBrightness(Percent.of(10)).mask(LEDPattern.progressMaskLayer(() -> RobotContainer.elementLift.getEncoderPosition() / Constants.elementLiftConstants.liftMaxEncoder));
  LEDPattern yellowLiftMaskPattern = LEDPattern.solid(yellowColor).atBrightness(Percent.of(10)).mask(LEDPattern.progressMaskLayer(() -> RobotContainer.elementLift.getEncoderPosition() / Constants.elementLiftConstants.liftMaxEncoder));

  LEDPattern stepFunnelPattern = LEDPattern.steps(Map.of(0.00, blueColor, 0.1111, whiteColor, 0.2222, yellowColor, .3333, blueColor, .4444 , whiteColor, .5555, yellowColor, .6666, blueColor, .7777, whiteColor, .8888, yellowColor )).scrollAtRelativeSpeed(Percent.per(Second).of(12.5)).atBrightness(Percent.of(20));//.breathe(Seconds.of(2.0)); 
  LEDPattern offPattern = LEDPattern.kOff;
  
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  AddressableLEDBufferView leftLiftBufferView;
  AddressableLEDBufferView middleLiftBufferView;
  AddressableLEDBufferView rightLiftBufferView;
  AddressableLEDBufferView betweenLiftAndFunnelBufferView;
  AddressableLEDBufferView leftFunnelBufferView;
  AddressableLEDBufferView middleFunnelBufferView;
  AddressableLEDBufferView rightFunnelBufferView;
  AddressableLEDBufferView notLiftBufferView;

  /** Creates a new LEDs. */
  public LEDs() {
    this.showPattern = ShowPattern.liftProgress;
    led = new AddressableLED(0);
    buffer = new AddressableLEDBuffer(100);
    leftLiftBufferView = buffer.createView(0,16);
    middleLiftBufferView = buffer.createView(17,33);
    rightLiftBufferView = buffer.createView(34, 49);
    betweenLiftAndFunnelBufferView = buffer.createView(50, 58);
    leftFunnelBufferView = buffer.createView(59, 77);
    middleFunnelBufferView = buffer.createView(78, 84);
    rightFunnelBufferView = buffer.createView(85, 99);
    notLiftBufferView = buffer.createView(45,99);

    led.setLength(100);
    led.start();
  }

  @Override
  public void periodic() {
    switch (showPattern) {
      case solidRed:
        solidRedPattern.applyTo(leftLiftBufferView);
        solidRedPattern.applyTo(middleLiftBufferView);
        solidRedPattern.applyTo(rightLiftBufferView);
        solidRedPattern.applyTo(betweenLiftAndFunnelBufferView); 
        solidRedPattern.applyTo(leftFunnelBufferView);
        solidRedPattern.applyTo(middleFunnelBufferView);
        solidRedPattern.applyTo(rightFunnelBufferView);
        break;
      case liftProgress:
      if (RobotContainer.elementLift.getEncoderPosition() > 253){
            solidGreenPattern.applyTo(middleLiftBufferView);
        }
        else{
          offPattern.applyTo(middleLiftBufferView);
        }
        blueLiftMaskPattern.applyTo(leftLiftBufferView);
        yellowLiftMaskPattern.reversed().applyTo(rightLiftBufferView);
        stepFunnelPattern.applyTo(notLiftBufferView);
        break;
      case solidWhite: 
        solidWhitePattern.applyTo(leftLiftBufferView);
        solidWhitePattern.applyTo(middleLiftBufferView);
        solidWhitePattern.applyTo(rightLiftBufferView);
        solidWhitePattern.applyTo(betweenLiftAndFunnelBufferView);
        solidWhitePattern.applyTo(leftFunnelBufferView);
        solidWhitePattern.applyTo(middleFunnelBufferView);
        solidWhitePattern.applyTo(rightFunnelBufferView); 
      break;
      case solidGreen:
        solidGreenPattern.applyTo(buffer);
      break;
    }  
    led.setData(buffer);
  }
}
