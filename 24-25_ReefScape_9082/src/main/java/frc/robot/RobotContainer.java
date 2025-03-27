// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.elementLiftConstants;
import frc.robot.commands.CoralEEAutoIntake;
import frc.robot.commands.CoralEEAutoOuttake;
import frc.robot.commands.ElementLiftAutoHeight;
import frc.robot.commands.CoralLevel2AutoHeight;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralEndEffector;
import frc.robot.subsystems.ElementLift;
import frc.robot.subsystems.UtilitySensors;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final Joystick joystick2 = new Joystick(1);

    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public static final ElementLift elementLift = new ElementLift();
    public static final UtilitySensors utilitySensors = new UtilitySensors();
    public static final CoralEndEffector coralEE = new CoralEndEffector();
    public static final frc.robot.subsystems.LEDs LEDs = new frc.robot.subsystems.LEDs();
        
    // Path Follower
    private final SendableChooser<Command> autoChooser;

    Command coralIntake = new CoralEEAutoIntake(utilitySensors, coralEE,elementLift);
    Command coralIntakeHeight = new ElementLiftAutoHeight(elementLift, Constants.elementLiftConstants.humanPlayerStationHeight);
    SequentialCommandGroup coralIntakeButtonCommand = new SequentialCommandGroup(coralIntakeHeight, coralIntake);
    Command autoHeight = new CoralLevel2AutoHeight(elementLift, utilitySensors, coralEE);

    public RobotContainer() {
        NamedCommands.registerCommand("CoralEEAutoIntake", new CoralEEAutoIntake(utilitySensors, coralEE, elementLift));
        NamedCommands.registerCommand("CoralEEAutoOuttake", new CoralEEAutoOuttake(coralEE, 1.5, 3.0));
        NamedCommands.registerCommand("ElementLiftAutoHeightHumanPlayer", new ElementLiftAutoHeight(elementLift, Constants.elementLiftConstants.humanPlayerStationHeight));
        NamedCommands.registerCommand("ElementLiftAutoHeightL1", new ElementLiftAutoHeight(elementLift, 26.0));
        //NamedCommands.registerCommand("ElementLiftAutoHeightL2", new ElementLiftAutoHeight(elementLift, 34.0));
        NamedCommands.registerCommand("ElementLiftAutoHeightL3", new ElementLiftAutoHeight(elementLift, 53.0));
        NamedCommands.registerCommand("ElementLiftAutoHeightL4", new ElementLiftAutoHeight(elementLift, 77.0));
        
        NamedCommands.registerCommand("CoralLevel2AutoHeight", new CoralLevel2AutoHeight(elementLift, utilitySensors, coralEE));
        
        // For convenience a programmer could change this when going to competition.
        boolean isCompetition = true;

        // Build an auto chooser. This will use Commands.none() as the default option.
        // As an example, this will only show autos that start with "comp" while at
        // competition as defined by the programmer
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
            (stream) -> isCompetition
            ? stream.filter(auto -> auto.getName().startsWith("Auto"))
            : stream
        );  

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
            
            if(Math.abs(joystick2.getY()) > 0.065){  //Joystick deadzone of 0.065
                if(elementLift.getEncoderPosition() < elementLiftConstants.liftMinEncoder){
                    elementLift.setVoltage(1.0);
                }
                else if(elementLift.getEncoderPosition() > elementLiftConstants.liftMaxEncoder){
                    elementLift.setVoltage(-1.0);
                }
                else{
                elementLift.setVoltage(-joystick2.getY()*12.0);    //Set power directly to the lift via the joystick y axis
                }
            }
            else{
                elementLift.setVoltage(0.0);
            }
            
        }, elementLift));

        utilitySensors.setDefaultCommand(new RunCommand(() -> {}, utilitySensors));
        coralEE.setDefaultCommand(new RunCommand(() -> {}, coralEE));
        LEDs.setDefaultCommand(new RunCommand(() -> {}, LEDs));

        //Joystick 1 button bindings:
        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));
        joystick.rightBumper().whileTrue(
            drivetrain.applyRequest(() ->
            drive.withVelocityX(-joystick.getLeftY() * 0.5 * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-joystick.getLeftX() * 0.5 * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-joystick.getRightX() * 0.5 * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

        //Operator Joysticks
        new JoystickButton(joystick2, 1).whileTrue(new CoralEEAutoOuttake(coralEE, 1.35, 2));
        new JoystickButton(joystick2, 7).whileTrue(new CoralEEAutoOuttake(coralEE, -1.35, 2));
        new JoystickButton(joystick2, 2).whileTrue(coralIntakeButtonCommand);
        new JoystickButton(joystick2, 12).whileTrue(new RunCommand(()-> {elementLift.resetEncoder();}));
        new JoystickButton(joystick2, 3).whileTrue(new ElementLiftAutoHeight(elementLift, 26.0).repeatedly());
        new JoystickButton(joystick2, 4).whileTrue(new ElementLiftAutoHeight(elementLift, 38.5).repeatedly());
        new JoystickButton(joystick2, 5).whileTrue(new ElementLiftAutoHeight(elementLift, 53.0).repeatedly());
        new JoystickButton(joystick2, 6).whileTrue(new ElementLiftAutoHeight(elementLift, 77.0).repeatedly());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
