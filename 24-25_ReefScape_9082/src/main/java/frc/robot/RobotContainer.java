// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Queue;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.elementLiftConstants;
import frc.robot.commands.CoralEEAutoOuttake;
import frc.robot.commands.ElementLiftAutoHeight;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralEndEffector;
import frc.robot.subsystems.ElementLift;
import frc.robot.subsystems.UtilitySensors;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;



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
    //public final UtilitySensors utilitySensors = new UtilitySensors();
    public final CoralEndEffector coralEE = new CoralEndEffector();
    public final PathPlannerAuto L4CoralLTeleOpAutomation = new PathPlannerAuto("L4CoralLTeleOpAutomation");

    // Path Follower
    private final SendableChooser<Command> autoChooser;


    public RobotContainer() {
        NamedCommands.registerCommand("CoralEEAutoOuttake", new CoralEEAutoOuttake(coralEE, 2.0, 3.0));
        NamedCommands.registerCommand("ElementLiftAutoHeightHumanPlayer", new ElementLiftAutoHeight(elementLift, 11.0));
        NamedCommands.registerCommand("ElementLiftAutoHeightL1", new ElementLiftAutoHeight(elementLift, 18.0));
        NamedCommands.registerCommand("ElementLiftAutoHeightL2", new ElementLiftAutoHeight(elementLift, 32.0));
        NamedCommands.registerCommand("ElementLiftAutoHeightL3", new ElementLiftAutoHeight(elementLift, 48.0));
        NamedCommands.registerCommand("ElementLiftAutoHeightL4", new ElementLiftAutoHeight(elementLift, 72.0));
        autoChooser = AutoBuilder.buildAutoChooser("L4CoralJAuto");
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
        
        //Element Lift handling
        elementLift.setDefaultCommand(new RunCommand(()->{
            if(Math.abs(joystick2.getY()) > 0.065){  //Joystick deadzone of 0.05
                if(elementLift.getEncoderPosition() < elementLiftConstants.liftMinEncoder){
                    elementLift.setVoltage(1);
                }
//                else if(elementLift.getEncoderPosition() > elementLiftConstants.liftMaxEncoder){
//                    elementLift.setVoltage(-1);
//               }
                else{
                elementLift.setVoltage(-joystick2.getY()*12);    //Set power directly to the lift via the joystick y axis
                }
            }
            else{
                elementLift.setVoltage(0);
            }
        }, elementLift));
        //utilitySensors.setDefaultCommand(new RunCommand(() -> {}, utilitySensors));

        coralEE.setDefaultCommand(new RunCommand(() -> {
            if (joystick2.getRawButton(1)){
                coralEE.setVoltage(2.0);
            }
        }, coralEE));

        //Joystick 1 button bindings:
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
        new JoystickButton(joystick2, 7).whileTrue(new ElementLiftAutoHeight(elementLift, 18.0).repeatedly());
        new JoystickButton(joystick2, 8).whileTrue(new ElementLiftAutoHeight(elementLift, 32.0).repeatedly());
        new JoystickButton(joystick2, 9).whileTrue(new ElementLiftAutoHeight(elementLift, 48.0).repeatedly());
        new JoystickButton(joystick2, 10).whileTrue(new ElementLiftAutoHeight(elementLift, 72.0).repeatedly());
        new JoystickButton(joystick2, 11).whileTrue(L4CoralLTeleOpAutomation);
        //new JoystickButton(joystick2, 12).whileTrue(new RunCommand(()-> {elementLift.resetEncoder();}));

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
