// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElementLift;

import frc.robot.subsystems.UtilitySensors;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final Joystick joystick2 = new Joystick(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final ElementLift elementLift = new ElementLift();
    public final UtilitySensors utilitySensors = new UtilitySensors();

    // Path Follower
    private final SendableChooser<Command> autoChooser;


    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Britney Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);
        drivetrain.registerTelemetry(logger::telemeterize);
        configureBindings();
    }

    private void configureBindings() {
        
        //Subsystem Default Commands        
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        
        elementLift.setDefaultCommand(new RunCommand(()->{
            elementLift.setSpeed(-joystick2.getY());
        }, elementLift));
        utilitySensors.setDefaultCommand(new RunCommand(() -> {}, utilitySensors));


        //Joystick 1 button bindings:
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        //For shooting on coral. Ref video for coral heights, https://www.youtube.com/watch?v=U-Hhq4fZT4U&list=PLZT9pIgNOV6aMzMboyCYP9MdSFgfckvl9&index=3.  
        joystick.button(7).whileTrue(new RunCommand(() ->{
            elementLift.goToHeight(18.0); //To score on L1(trough). Inches.
        }, elementLift)); 
        joystick.button(8).whileTrue(new RunCommand(() -> {
            elementLift.goToHeight(32.0); //To score on L2. Inches.
        }, elementLift)); 
        joystick.button(9).whileTrue(new RunCommand(() -> {
            elementLift.goToHeight(48.0); //To score on L3. Inches.
        }, elementLift)); 
        joystick.button(10).whileTrue(new RunCommand(() -> {
            elementLift.goToHeight(72.0); //To score on L4. Inches.
        }, elementLift)); 


        
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));


        
        
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();

    }
}
