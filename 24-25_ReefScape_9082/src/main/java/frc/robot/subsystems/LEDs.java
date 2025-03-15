// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//LED docs link https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;
import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.elementLiftConstants;
import frc.robot.RobotContainer;

public class LEDs extends SubsystemBase {
  private static final int port = 0;
  private static final int liftBatterySidelength = 0;

  Color yellowColor = new Color(45, 255, 0); 
  Color blueColor = new Color(25, 0, 255); 
  Color redColor = new Color(255, 0, 0);
  Color greenColor = new Color(0,255,0);
  Color whiteColor = new Color(255,255,255);

  LEDPattern scrollingRainbow = LEDPattern.rainbow(255,5).scrollAtRelativeSpeed(Percent.per(Second).of(25));

  LEDPattern solidRedPattern = LEDPattern.solid(Color.kRed).atBrightness(Percent.of(25));
  LEDPattern solidYellowPattern = LEDPattern.solid(yellowColor).atBrightness(Percent.of(25));
  LEDPattern solidBluePattern = LEDPattern.solid(blueColor).atBrightness(Percent.of(25));
  LEDPattern solidWhitePattern = LEDPattern.solid(whiteColor).atBrightness(Percent.of(25));

  LEDPattern yellowMaskPattern = LEDPattern.solid(yellowColor).atBrightness(Percent.of(10)).mask(LEDPattern.progressMaskLayer(() -> Constants.elementLiftConstants.coralL4Height / Constants.elementLiftConstants.liftMaxEncoder));
  LEDPattern blueMaskPattern = LEDPattern.solid(Color.kBlue).atBrightness(Percent.of(10)).mask(LEDPattern.progressMaskLayer(() -> Constants.elementLiftConstants.coralL4Height / Constants.elementLiftConstants.liftMaxEncoder));
  LEDPattern redMaskPattern = LEDPattern.solid(Color.kRed).atBrightness(Percent.of(10)).mask(LEDPattern.progressMaskLayer(() -> Constants.elementLiftConstants.coralL4Height / Constants.elementLiftConstants.liftMaxEncoder));


  LEDPattern blueMaskPatternReversed = LEDPattern.solid(Color.kBlue).mask(LEDPattern.progressMaskLayer(() -> Constants.elementLiftConstants.coralL4Height / Constants.elementLiftConstants.liftMaxEncoder)).reversed();
  
  LEDPattern defaultProgressPattern = LEDPattern.progressMaskLayer(() -> (20.0 / 100.0)).atBrightness(Percent.of(25)); 

  LEDPattern yellowBreathePattern = LEDPattern.solid(Color.kYellow).breathe(Seconds.of(2/50)); 
  LEDPattern blueBreathePattern = LEDPattern.solid(Color.kBlue).breathe(Seconds.of(2/50));
  LEDPattern redBreathePattern = LEDPattern.solid(Color.kRed).breathe(Seconds.of(2/50));

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  AddressableLEDBufferView leftLiftBufferView;
  AddressableLEDBufferView middleLiftBufferView;
  AddressableLEDBufferView rightLiftBufferView;
  AddressableLEDBufferView betweenLiftAndFunnelBufferView;
  AddressableLEDBufferView leftFunnelBufferView;
  AddressableLEDBufferView middleFunnelBufferView;
  AddressableLEDBufferView rightFunnelBufferView;

  /** Creates a new LEDs. */
  public LEDs() {
    led = new AddressableLED(0);
    //Entire LED strip had 3600 LEDS. 
    buffer = new AddressableLEDBuffer(100);

    //This number will be 0, length being used/LEDSPerMeter. Gives the total number of LEDs on the strip. 
   // leftBufferView = buffer.createView(0, 719); //First meter of LED strip
   // rightBufferView = buffer.createView(720, 1439); //Second meter of LED strip. 
    //unusedBufferView = buffer.createView(1440, 3599); //Unused section of the LED strip. 

    leftLiftBufferView = buffer.createView(0,16);
    middleLiftBufferView = buffer.createView(17,24);
    rightLiftBufferView = buffer.createView(25, 30);
    betweenLiftAndFunnelBufferView = buffer.createView(31, 42);
    leftFunnelBufferView = buffer.createView(43, 50);
    middleFunnelBufferView = buffer.createView(51, 61);
    rightFunnelBufferView = buffer.createView(62, 99);

    led.setLength(100);
    led.start();
  }

  @Override
  public void periodic() {
    solidBluePattern.applyTo(leftLiftBufferView);
    solidYellowPattern.applyTo(middleLiftBufferView);
    solidBluePattern.applyTo(rightLiftBufferView);
    solidYellowPattern.applyTo(betweenLiftAndFunnelBufferView);
    solidBluePattern.applyTo(leftFunnelBufferView);
    solidYellowPattern.applyTo(middleFunnelBufferView);
    solidBluePattern.applyTo(rightFunnelBufferView);
    
    led.setData(buffer);
  }
}
