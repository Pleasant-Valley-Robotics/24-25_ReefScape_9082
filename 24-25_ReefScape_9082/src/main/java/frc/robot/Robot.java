// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final boolean kUseLimelight = true;
  private Matrix<N3,N1> visionStdDefault = VecBuilder.fill(0.3,0.3,999999);
  private Matrix<N3,N1> visionStd = VecBuilder.fill(0.3,0.3,999999);
  public Robot() {
    m_robotContainer = new RobotContainer();
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */
    if (kUseLimelight) {
      var driveState = RobotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees(); //WE NEED TO SEE IF THIS CHANGES BASED ON OUR AUTO START LOCATION
      //We can test the above line in the stands by running blue auto on the cart and verifying whether or not this value changes

      //double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
      double yawRate = Units.radiansToDegrees(driveState.Speeds.omegaRadiansPerSecond); //We need to test if this changes CW+ or CCW+

      /*
       * Field Space (FRC) (Used by map generator)
      3d Cartesian Coordinate System with (0,0,0) located at the center of the field
      X+ → Points along the long side of the field
      Y+ → Points up the short side of the field
      Z+ → Points towards the sky
      Right-handed. Positive theta results in counterclockwise rotation from positive outside perspective
       */
      SmartDashboard.putNumber("Limelight Degrees Input", headingDeg);
      SmartDashboard.putNumber("Limelight YawRate Input", yawRate);
      

      LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if (llMeasurement != null && llMeasurement.tagCount > 0 && llMeasurement.avgTagDist < 5 /*&& ((llMeasurement.pose.getX()-RobotContainer.drivetrain.getState().Pose.getX()) < .1)&& ((llMeasurement.pose.getY()-RobotContainer.drivetrain.getState().Pose.getY()) < .1)*/) {
        visionStd = visionStdDefault.times(llMeasurement.avgTagDist);
        SmartDashboard.putNumber("visionStdX", visionStd.get(0,0));
        SmartDashboard.putNumber("visionStdY", visionStd.get(1,0));
        SmartDashboard.putNumber("visionStdHeading", visionStd.get(2,0));
        SmartDashboard.putNumber("llMeasurement avg tag dist", llMeasurement.avgTagDist);
        
        
        //We would like to show which april tag IDs onto SmartDashboard here

        RobotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, Utils.fpgaToCurrentTime(llMeasurement.timestampSeconds), visionStd);
      }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}