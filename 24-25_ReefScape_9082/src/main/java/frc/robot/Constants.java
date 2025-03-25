// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class operatorConstants {
    public static final int driverControllerPort = 0;
  }
  public static class coralAlignmentConstants {

    public static final double translativeTolerance = .1;
    public static final double rotationalTolerance = 5.0;

    public static final double coralAX = 3.141;
    public static final double coralAY = 4.175;
    public static final double coralARot = 0.0;

    public static final double coralBX = 3.141;
    public static final double coralBY = 3.875;
    public static final double coralBRot = 0.0;

    public static final double coralCX = 3.644;
    public static final double coralCY = 2.940;
    public static final double coralCRot = 60.0;

    public static final double coralDX = 3.920;
    public static final double coralDY = 2.728;
    public static final double coralDRot = 60.0;

    public static final double coralEX = 5.011;
    public static final double coralEY = 2.772;
    public static final double coralERot = 120.0;

    public static final double coralFX = 5.323;
    public static final double coralFY = 2.928;
    public static final double coralFRot = 120.0;

    public static final double coralGX = 5.870;
    public static final double coralGY = 3.859;
    public static final double coralGRot = 180.0;
    
    public static final double coralHX = 5.860;
    public static final double coralHY = 4.161;
    public static final double coralHRot = 180.0;

    public static final double coralIX = 5.304;
    public static final double coralIY = 5.117;
    public static final double coralIRot = -120.0;
    
    public static final double coralJX = 5.011;
    public static final double coralJY = 5.283;
    public static final double coralJRot = -120.0;

    public static final double coralKX = 3.949;
    public static final double coralKY = 5.263;
    public static final double coralKRot = -60.0;

    public static final double coralLX = 3.695;
    public static final double coralLY = 5.117;
    public static final double coralLRot = -60.0;
  }
  public static class elementLiftConstants {
    public static final int elementLiftCAN = 52;

    /*
     * To calculate the encodersToInches, 
     * 1. Measure the encoders of the lift in SmartDashboard
     * 2. Start at 0 encoders
     * 3. Drive the lift up to the maximum height
     * 4. Write this encoder number down from SmartDashboard
     * 5. Measure the distance from the ground to the current end-effector location on the lift
     * 6. Subtract the distance from the ground to the initial height of the end-effector
     * 7. Take the resulting distance and divide it by the measured encoders - this is the encoderToInches value
     */

     /*
      * This is a two-point calibration method
      */
    public static final double measuredDistance = 42.5;
    public static final double measuredEncoders = 47.0;
    public static final double liftOffset = 18.125;
    public static final double encoderToInches = (measuredDistance-liftOffset)/measuredEncoders;
    /*
     * Math for Proportional Gain:
     * P = Desired Voltage / Desired Error
     * i.e. we want to be going at 2V when we are 12 inches away, we would have 2V/12in
     * We use 1V / 2 inches of error, from experimenting
     */
    public static final double liftP = 2.0;
    /*
     * an Integral Gain value would account for gravity pulling the lift down beyond what the motor is supplying voltage for
     * i.e. steady state error
     */
    public static final double liftI = 0.0;

    /*
     * IDK
     */
    public static final double liftD = 0.0;

    //The maximum voltage that the element lift can have during a PID Controlled system
    public static final double liftMaxVoltage = 12.0;

    //The minimum voltage that the element lift can have during a PID Controlled system
    public static final double liftMinVoltage = 1.5;
    
    public static final double liftMaxEncoder = 285.0;

    public static final double liftMinEncoder = -10.0;
    
    public static final double humanPlayerStationHeight = liftOffset; //Height for human player station. 

    //Coral level heights. 
    public static final double coralL1Height = 18.0;
    public static final double coralL2Height = 32.0; 
    public static final double coralL3Height = 48.0;
    public static final double coralL4Height = 77.0;
  } 
  /*
   * Coral End Effector Constants
   */
  public static class coralEEConstants{
    public static final int coralEECAN = 53;
  }
  public static class LEDConstants{
    public static final int liftLEDPort = 0;
    public static final int liftBufferLength = 3600;

    public static final int LEDSPerMeter = 720;  //720 LEDS per meter.
    public static final int totalLEDLengthUsed = 720;
  }
}
