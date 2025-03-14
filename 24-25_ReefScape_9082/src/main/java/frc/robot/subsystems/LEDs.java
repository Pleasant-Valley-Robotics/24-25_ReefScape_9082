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
import frc.robot.RobotContainer;

public class LEDs extends SubsystemBase {
 private static final int port = 0;
 private static final int length = 720;
 LEDPattern scrollingRainbow = LEDPattern.rainbow(255,5).scrollAtRelativeSpeed(Percent.per(Second).of(25));
 private final AddressableLED led;
 private final AddressableLEDBuffer buffer;


  /** Creates a new LEDs. */
  public LEDs() {
    led = new AddressableLED(0);
    buffer = new AddressableLEDBuffer(720);
    led.setLength(length);
    led.start();
  }

  @Override
  public void periodic() {
    scrollingRainbow.applyTo(buffer); 
    led.setData(buffer);
  }
}
