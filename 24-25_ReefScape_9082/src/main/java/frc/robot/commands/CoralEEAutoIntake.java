// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEndEffector;
import frc.robot.subsystems.ElementLift;
import frc.robot.subsystems.UtilitySensors;


public class CoralEEAutoIntake extends Command {
  
  //Class variable definitions
  private CoralEndEffector coralEE;
  private UtilitySensors sensors;
  private ElementLift elementLift;
  /**
 * This command will automatically drive the coral into the end-effector until
 * the coral is no longer sticking out the back of our lift, ensuring a safe
 * lifting process without risking any collisions. We want this to be handled
 * autonomously to eliminate human error and prevent accidentally sending the
 * coral out the front of our end-effector, as this is a simple through-put
 * system. We do this by using a Modern Robotics Core Optical Distance Sensor
 * placed at the entry-point of our end-effector, then we drive the coral into 
 * the end-effector until it can no longer be seen by the sensor. At this point
 * we know that the coral is fully staged in our system and we can move on without
 * causing harm to our physical systems.
 * @param sensors - The Utility Sensors subsystem
 * @param coralEE - The Coral End Effector subsystem
 */
  public CoralEEAutoIntake(UtilitySensors sensors, CoralEndEffector coralEE, ElementLift elementLift) {
    /*
     * We feed in and use a UtilitySensors and CoralEndEffector class, which
     * control the sensors within our robot, and the subsystem that manipulates
     * the coral game element.
     */
    this.sensors = sensors;
    this.coralEE = coralEE;

    addRequirements(sensors);
    addRequirements(coralEE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*
     * Here we want to reset our voltage to 0 to ensure that nothing is moving
     * through our end-effector
     */
    elementLift.setVoltage(0);
    coralEE.setVoltage(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
     * Here we set our voltage to 6 to drive our end-effector at half-speed
     * This is enough to reasonably quickly send a coral out in a controlled
     * manner
     */
    //elementLift.goToHeight()
    coralEE.setVoltage(6);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /*
     * If our command is interrupted or finishes, we should stop moving the
     * end effector. This requires resetting the voltage back to 0.
     */
    coralEE.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
     * We end our command if the coral detection sensor placed at the entrance
     * of our end-effector no longer detects the coral. This indicates that the
     * coral is placed fully within the end-effector, and is no longer in the
     * transition stage between the funnel and the end-effector.
     * 
     * In other words, we stop intaking when the coral is 'fully staged' in our
     * end-effector.
     */
    if(sensors.coralDetected()){
      return false;
    }
    else{
      return true;
    }
  }
}
