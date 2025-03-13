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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.elementLiftConstants; 
import frc.robot.Constants.LEDConstants;
import frc.robot.RobotContainer;

public class LEDs extends SubsystemBase {
  public enum ShowPattern {
    SOLID_BLUE,
    SOLID_RED,
    SOLID_YELLOW,
    BREATHE_BLUE,
    BREATHE_RED,
    BREATHE_YELLOW
  }

  private ShowPattern showPattern; 

  Color blueColor = new Color(0,0,255);
  Color yellowColor = new Color(255, 255, 0);
  Color redColor = new Color(0, 255, 0);

  LEDPattern scrollingRainbow = LEDPattern.rainbow(255,5).scrollAtRelativeSpeed(Percent.per(Second).of(25));

  LEDPattern solidBluePattern = LEDPattern.solid(blueColor);
  LEDPattern solidRedPattern = LEDPattern.solid(redColor);
  LEDPattern solidYellowPattern = LEDPattern.solid(yellowColor);

  LEDPattern breatheBluePattern = LEDPattern.solid(blueColor).atBrightness(Percent.of(100)).breathe(Seconds.of(2));
  LEDPattern breatheRedPattern = LEDPattern.solid(redColor).atBrightness(Percent.of(100)).breathe(Seconds.of(2));
  LEDPattern breatheYellowPattern = LEDPattern.solid(yellowColor).atBrightness(Percent.of(100)).breathe(Seconds.of(2));

  LEDPattern progressMaskPattern = LEDPattern.progressMaskLayer(() -> RobotContainer.elementLift.getEncoderPosition() / elementLiftConstants.liftMaxEncoder);

  private final AddressableLED liftLED;
  private final AddressableLED funnelLED;

  private final AddressableLEDBuffer liftBuffer;
  private final AddressableLEDBuffer funnelBuffer;

  Optional<Alliance> alliance;

  /** Creates a new LEDs. */
  public LEDs() {
    this.showPattern = ShowPattern.SOLID_BLUE;

    liftLED = new AddressableLED(Constants.LEDConstants.liftLEDPort);
    funnelLED = new AddressableLED(Constants.LEDConstants.funnelLEDPort);

    liftBuffer = new AddressableLEDBuffer(Constants.LEDConstants.liftBufferLength);
    funnelBuffer = new AddressableLEDBuffer(Constants.LEDConstants.funnelBufferLength);

    liftLED.setLength(Constants.LEDConstants.liftBufferLength);
    funnelLED.setLength(Constants.LEDConstants.funnelBufferLength);

    liftLED.start();
    funnelLED.start();

     alliance = DriverStation.getAlliance();
  }

  @Override
  public void periodic() {
    switch (showPattern){
      case SOLID_BLUE:
        solidBluePattern.applyTo(liftBuffer);
        solidBluePattern.applyTo(funnelBuffer);
        break;
      case SOLID_RED:
        solidRedPattern.applyTo(liftBuffer);
        solidRedPattern.applyTo(funnelBuffer);
        break;
      case SOLID_YELLOW:
        solidYellowPattern.applyTo(liftBuffer);
        solidYellowPattern.applyTo(funnelBuffer);
        break;
      case BREATHE_BLUE:
        breatheBluePattern.applyTo(liftBuffer);
        breatheBluePattern.applyTo(funnelBuffer);
        break;
      case BREATHE_RED:
        breatheRedPattern.applyTo(liftBuffer);
        breatheRedPattern.applyTo(funnelBuffer);
        break;
      case BREATHE_YELLOW:
        breatheYellowPattern.applyTo(liftBuffer);
        breatheYellowPattern.applyTo(funnelBuffer);
        break;
    }

    liftLED.setData(liftBuffer);
    funnelLED.setData(funnelBuffer);
  }

  public void setLEDPattern(ShowPattern pattern){
    showPattern = pattern;
  }

  public ShowPattern getLEDPattern(){
    return showPattern;
  }
}
