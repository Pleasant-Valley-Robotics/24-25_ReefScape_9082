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
    public static final double measuredDistance = 60.0;
    public static final double measuredEncoders = 74.5;
    public static final double liftOffset = 19.875;
    public static final double encoderToInches = (measuredDistance)/measuredEncoders;
    /*
     * Math for Proportional Gain:
     * P = Desired Voltage / Desired Error
     * i.e. we want to be going at 2V when we are 12 inches away, we would have 2V/12in
     * We use 1V / 2 inches of error, from experimenting
     */
    public static final double liftP = 1.0/2.0;
    /*
     * an Integral Gain value would account for gravity pulling the lift down beyond what the motor is supplying voltage for
     * i.e. steady state error
     */
    public static final double liftI = 0.0;

    /*
     * IDK
     */
    public static final double liftD = 0;

    //The maximum voltage that the element lift can have during a PID Controlled system
    public static final double liftMaxVoltage = 6.0;

    //The minimum voltage that the element lift can have during a PID Controlled system
    public static final double liftMinVoltage = 1.5;
    
    public static final double liftMaxEncoder = 78.5;

    public static final double liftMinEncoder = -10.0;
    
    public static final double humanPlayerStationHeight = liftOffset; //Height for human player station. 

    //Coral level heights. 
    public static final double coralL1Height = 18.0;
    public static final double coralL2Height = 32.0; 
    public static final double coralL3Height = 48.0;
    public static final double coralL4Height = 75.0;
  }
  /*
   * Coral End Effector Constants
   */
  public static class coralEEConstants{
    public static final int coralEECAN = 53;
  }
  public static class LEDConstants{
    public static final int port = 0;
    public static final int bufferLenght = 720;
  }
}
