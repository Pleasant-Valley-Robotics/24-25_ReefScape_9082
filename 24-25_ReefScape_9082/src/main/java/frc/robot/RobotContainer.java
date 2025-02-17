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
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    private final Joystick joystick3 = new Joystick(2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ElementLift elementLift = new ElementLift();
    //public final UtilitySensors utilitySensors = new UtilitySensors();
    public final CoralEndEffector coralEE = new CoralEndEffector();
    

    //Buttons on Joystick 2 (Arm Joystick 1)
    JoystickButton J2Button1 = new JoystickButton(joystick2, 1);
    JoystickButton J2Button2 = new JoystickButton(joystick2, 2);
    JoystickButton J2Button3 = new JoystickButton(joystick2, 3);
    JoystickButton J2Button4 = new JoystickButton(joystick2, 4);
    JoystickButton J2Button5 = new JoystickButton(joystick2, 5);
    JoystickButton J2Button6 = new JoystickButton(joystick2, 6);
    JoystickButton J2Button7 = new JoystickButton(joystick2, 7);
    JoystickButton J2Button8 = new JoystickButton(joystick2, 8);
    JoystickButton J2Button9 = new JoystickButton(joystick2, 9);
    JoystickButton J2Button10 = new JoystickButton(joystick2, 10);
    JoystickButton J2Button11 = new JoystickButton(joystick2, 11);
    JoystickButton J2Button12 = new JoystickButton(joystick2, 12);

    //Buttons on Joystick 3 (Arm Joystick 2)
    JoystickButton J3Button1 = new JoystickButton(joystick3, 1);
    JoystickButton J3Button2 = new JoystickButton(joystick3, 2);
    JoystickButton J3Button3 = new JoystickButton(joystick3, 3);
    JoystickButton J3Button4 = new JoystickButton(joystick3, 4);
    JoystickButton J3Button5 = new JoystickButton(joystick3, 5);
    JoystickButton J3Button6 = new JoystickButton(joystick3, 6);
    JoystickButton J3Button7 = new JoystickButton(joystick3, 7);
    JoystickButton J3Button8 = new JoystickButton(joystick3, 8);
    JoystickButton J3Button9 = new JoystickButton(joystick3, 9);
    JoystickButton J3Button10 = new JoystickButton(joystick3, 10);
    JoystickButton J3Button11 = new JoystickButton(joystick3, 11);
    JoystickButton J3Button12 = new JoystickButton(joystick3, 12);
    
    //Coral Automation Triggers
    boolean HumanPlayerStation = true;  //True = A (Left), False = B (Right)
    Trigger CoralL1ReefA = J2Button5.and(J2Button7);
    Trigger CoralL1ReefB = J2Button5.and(J2Button8);
    Trigger CoralL1ReefC = J2Button5.and(J2Button9);
    Trigger CoralL1ReefD = J2Button5.and(J2Button10);
    Trigger CoralL1ReefE = J2Button5.and(J2Button11);
    Trigger CoralL1ReefF = J2Button5.and(J2Button12);
    Trigger CoralL1ReefG = J2Button5.and(J3Button7);
    Trigger CoralL1ReefH = J2Button5.and(J3Button8);
    Trigger CoralL1ReefI = J2Button5.and(J3Button9);
    Trigger CoralL1ReefJ = J2Button5.and(J3Button10);
    Trigger CoralL1ReefK = J2Button5.and(J3Button11);
    Trigger CoralL1ReefL = J2Button5.and(J3Button12);
    Trigger CoralL2ReefA = J2Button6.and(J2Button7);
    Trigger CoralL2ReefB = J2Button6.and(J2Button8);
    Trigger CoralL2ReefC = J2Button6.and(J2Button9);
    Trigger CoralL2ReefD = J2Button6.and(J2Button10);
    Trigger CoralL2ReefE = J2Button6.and(J2Button11);
    Trigger CoralL2ReefF = J2Button6.and(J2Button12);
    Trigger CoralL2ReefG = J2Button6.and(J3Button7);
    Trigger CoralL2ReefH = J2Button6.and(J3Button8);
    Trigger CoralL2ReefI = J2Button6.and(J3Button9);
    Trigger CoralL2ReefJ = J2Button6.and(J3Button10);
    Trigger CoralL2ReefK = J2Button6.and(J3Button11);
    Trigger CoralL2ReefL = J2Button6.and(J3Button12);
    Trigger CoralL3ReefA = J2Button3.and(J2Button7);
    Trigger CoralL3ReefB = J2Button3.and(J2Button8);
    Trigger CoralL3ReefC = J2Button3.and(J2Button9);
    Trigger CoralL3ReefD = J2Button3.and(J2Button10);
    Trigger CoralL3ReefE = J2Button3.and(J2Button11);
    Trigger CoralL3ReefF = J2Button3.and(J2Button12);
    Trigger CoralL3ReefG = J2Button3.and(J3Button7);
    Trigger CoralL3ReefH = J2Button3.and(J3Button8);
    Trigger CoralL3ReefI = J2Button3.and(J3Button9);
    Trigger CoralL3ReefJ = J2Button3.and(J3Button10);
    Trigger CoralL3ReefK = J2Button3.and(J3Button11);
    Trigger CoralL3ReefL = J2Button3.and(J3Button12);
    Trigger CoralL4ReefA = J2Button4.and(J2Button7);
    Trigger CoralL4ReefB = J2Button4.and(J2Button8);
    Trigger CoralL4ReefC = J2Button4.and(J2Button9);
    Trigger CoralL4ReefD = J2Button4.and(J2Button10);
    Trigger CoralL4ReefE = J2Button4.and(J2Button11);
    Trigger CoralL4ReefF = J2Button4.and(J2Button12);
    Trigger CoralL4ReefG = J2Button4.and(J3Button7);
    Trigger CoralL4ReefH = J2Button4.and(J3Button8);
    Trigger CoralL4ReefI = J2Button4.and(J3Button9);
    Trigger CoralL4ReefJ = J2Button4.and(J3Button10);
    Trigger CoralL4ReefK = J2Button4.and(J3Button11);
    Trigger CoralL4ReefL = J2Button4.and(J3Button12);

    //TeleOpAutomation Routines
    //Human Player Station A
    public final PathPlannerAuto L1CoralATeleOpAutomationA = new PathPlannerAuto("L1CoralATeleOpAutomationA");
    public final PathPlannerAuto L1CoralBTeleOpAutomationA = new PathPlannerAuto("L1CoralBTeleOpAutomationA");
    public final PathPlannerAuto L1CoralCTeleOpAutomationA = new PathPlannerAuto("L1CoralCTeleOpAutomationA");
    public final PathPlannerAuto L1CoralDTeleOpAutomationA = new PathPlannerAuto("L1CoralDTeleOpAutomationA");
    public final PathPlannerAuto L1CoralETeleOpAutomationA = new PathPlannerAuto("L1CoralETeleOpAutomationA");
    public final PathPlannerAuto L1CoralFTeleOpAutomationA = new PathPlannerAuto("L1CoralFTeleOpAutomationA");
    public final PathPlannerAuto L1CoralGTeleOpAutomationA = new PathPlannerAuto("L1CoralGTeleOpAutomationA");
    public final PathPlannerAuto L1CoralHTeleOpAutomationA = new PathPlannerAuto("L1CoralHTeleOpAutomationA");
    public final PathPlannerAuto L1CoralITeleOpAutomationA = new PathPlannerAuto("L1CoralITeleOpAutomationA");
    public final PathPlannerAuto L1CoralJTeleOpAutomationA = new PathPlannerAuto("L1CoralJTeleOpAutomationA");
    public final PathPlannerAuto L1CoralKTeleOpAutomationA = new PathPlannerAuto("L1CoralKTeleOpAutomationA");
    public final PathPlannerAuto L1CoralLTeleOpAutomationA = new PathPlannerAuto("L1CoralLTeleOpAutomationA");
    public final PathPlannerAuto L2CoralATeleOpAutomationA = new PathPlannerAuto("L2CoralATeleOpAutomationA");
    public final PathPlannerAuto L2CoralBTeleOpAutomationA = new PathPlannerAuto("L2CoralBTeleOpAutomationA");
    public final PathPlannerAuto L2CoralCTeleOpAutomationA = new PathPlannerAuto("L2CoralCTeleOpAutomationA");
    public final PathPlannerAuto L2CoralDTeleOpAutomationA = new PathPlannerAuto("L2CoralDTeleOpAutomationA");
    public final PathPlannerAuto L2CoralETeleOpAutomationA = new PathPlannerAuto("L2CoralETeleOpAutomationA");
    public final PathPlannerAuto L2CoralFTeleOpAutomationA = new PathPlannerAuto("L2CoralFTeleOpAutomationA");
    public final PathPlannerAuto L2CoralGTeleOpAutomationA = new PathPlannerAuto("L2CoralGTeleOpAutomationA");
    public final PathPlannerAuto L2CoralHTeleOpAutomationA = new PathPlannerAuto("L2CoralHTeleOpAutomationA");
    public final PathPlannerAuto L2CoralITeleOpAutomationA = new PathPlannerAuto("L2CoralITeleOpAutomationA");
    public final PathPlannerAuto L2CoralJTeleOpAutomationA = new PathPlannerAuto("L2CoralJTeleOpAutomationA");
    public final PathPlannerAuto L2CoralKTeleOpAutomationA = new PathPlannerAuto("L2CoralKTeleOpAutomationA");
    public final PathPlannerAuto L2CoralLTeleOpAutomationA = new PathPlannerAuto("L2CoralLTeleOpAutomationA");
    public final PathPlannerAuto L3CoralATeleOpAutomationA = new PathPlannerAuto("L3CoralATeleOpAutomationA");
    public final PathPlannerAuto L3CoralBTeleOpAutomationA = new PathPlannerAuto("L3CoralBTeleOpAutomationA");
    public final PathPlannerAuto L3CoralCTeleOpAutomationA = new PathPlannerAuto("L3CoralCTeleOpAutomationA");
    public final PathPlannerAuto L3CoralDTeleOpAutomationA = new PathPlannerAuto("L3CoralDTeleOpAutomationA");
    public final PathPlannerAuto L3CoralETeleOpAutomationA = new PathPlannerAuto("L3CoralETeleOpAutomationA");
    public final PathPlannerAuto L3CoralFTeleOpAutomationA = new PathPlannerAuto("L3CoralFTeleOpAutomationA");
    public final PathPlannerAuto L3CoralGTeleOpAutomationA = new PathPlannerAuto("L3CoralGTeleOpAutomationA");
    public final PathPlannerAuto L3CoralHTeleOpAutomationA = new PathPlannerAuto("L3CoralHTeleOpAutomationA");
    public final PathPlannerAuto L3CoralITeleOpAutomationA = new PathPlannerAuto("L3CoralITeleOpAutomationA");
    public final PathPlannerAuto L3CoralJTeleOpAutomationA = new PathPlannerAuto("L3CoralJTeleOpAutomationA");
    public final PathPlannerAuto L3CoralKTeleOpAutomationA = new PathPlannerAuto("L3CoralKTeleOpAutomationA");
    public final PathPlannerAuto L3CoralLTeleOpAutomationA = new PathPlannerAuto("L3CoralLTeleOpAutomationA");
    public final PathPlannerAuto L4CoralATeleOpAutomationA = new PathPlannerAuto("L4CoralATeleOpAutomationA");
    public final PathPlannerAuto L4CoralBTeleOpAutomationA = new PathPlannerAuto("L4CoralBTeleOpAutomationA");
    public final PathPlannerAuto L4CoralCTeleOpAutomationA = new PathPlannerAuto("L4CoralCTeleOpAutomationA");
    public final PathPlannerAuto L4CoralDTeleOpAutomationA = new PathPlannerAuto("L4CoralDTeleOpAutomationA");
    public final PathPlannerAuto L4CoralETeleOpAutomationA = new PathPlannerAuto("L4CoralETeleOpAutomationA");
    public final PathPlannerAuto L4CoralFTeleOpAutomationA = new PathPlannerAuto("L4CoralFTeleOpAutomationA");
    public final PathPlannerAuto L4CoralGTeleOpAutomationA = new PathPlannerAuto("L4CoralGTeleOpAutomationA");
    public final PathPlannerAuto L4CoralHTeleOpAutomationA = new PathPlannerAuto("L4CoralHTeleOpAutomationA");
    public final PathPlannerAuto L4CoralITeleOpAutomationA = new PathPlannerAuto("L4CoralITeleOpAutomationA");
    public final PathPlannerAuto L4CoralJTeleOpAutomationA = new PathPlannerAuto("L4CoralJTeleOpAutomationA");
    public final PathPlannerAuto L4CoralKTeleOpAutomationA = new PathPlannerAuto("L4CoralKTeleOpAutomationA");
    public final PathPlannerAuto L4CoralLTeleOpAutomationA = new PathPlannerAuto("L4CoralLTeleOpAutomationA");
    
    //Human Player Station B
    public final PathPlannerAuto L1CoralATeleOpAutomationB = new PathPlannerAuto("L1CoralATeleOpAutomationB");
    public final PathPlannerAuto L1CoralBTeleOpAutomationB = new PathPlannerAuto("L1CoralBTeleOpAutomationB");
    public final PathPlannerAuto L1CoralCTeleOpAutomationB = new PathPlannerAuto("L1CoralCTeleOpAutomationB");
    public final PathPlannerAuto L1CoralDTeleOpAutomationB = new PathPlannerAuto("L1CoralDTeleOpAutomationB");
    public final PathPlannerAuto L1CoralETeleOpAutomationB = new PathPlannerAuto("L1CoralETeleOpAutomationB");
    public final PathPlannerAuto L1CoralFTeleOpAutomationB = new PathPlannerAuto("L1CoralFTeleOpAutomationB");
    public final PathPlannerAuto L1CoralGTeleOpAutomationB = new PathPlannerAuto("L1CoralGTeleOpAutomationB");
    public final PathPlannerAuto L1CoralHTeleOpAutomationB = new PathPlannerAuto("L1CoralHTeleOpAutomationB");
    public final PathPlannerAuto L1CoralITeleOpAutomationB = new PathPlannerAuto("L1CoralITeleOpAutomationB");
    public final PathPlannerAuto L1CoralJTeleOpAutomationB = new PathPlannerAuto("L1CoralJTeleOpAutomationB");
    public final PathPlannerAuto L1CoralKTeleOpAutomationB = new PathPlannerAuto("L1CoralKTeleOpAutomationB");
    public final PathPlannerAuto L1CoralLTeleOpAutomationB = new PathPlannerAuto("L1CoralLTeleOpAutomationB");
    public final PathPlannerAuto L2CoralATeleOpAutomationB = new PathPlannerAuto("L2CoralATeleOpAutomationB");
    public final PathPlannerAuto L2CoralBTeleOpAutomationB = new PathPlannerAuto("L2CoralBTeleOpAutomationB");
    public final PathPlannerAuto L2CoralCTeleOpAutomationB = new PathPlannerAuto("L2CoralCTeleOpAutomationB");
    public final PathPlannerAuto L2CoralDTeleOpAutomationB = new PathPlannerAuto("L2CoralDTeleOpAutomationB");
    public final PathPlannerAuto L2CoralETeleOpAutomationB = new PathPlannerAuto("L2CoralETeleOpAutomationB");
    public final PathPlannerAuto L2CoralFTeleOpAutomationB = new PathPlannerAuto("L2CoralFTeleOpAutomationB");
    public final PathPlannerAuto L2CoralGTeleOpAutomationB = new PathPlannerAuto("L2CoralGTeleOpAutomationB");
    public final PathPlannerAuto L2CoralHTeleOpAutomationB = new PathPlannerAuto("L2CoralHTeleOpAutomationB");
    public final PathPlannerAuto L2CoralITeleOpAutomationB = new PathPlannerAuto("L2CoralITeleOpAutomationB");
    public final PathPlannerAuto L2CoralJTeleOpAutomationB = new PathPlannerAuto("L2CoralJTeleOpAutomationB");
    public final PathPlannerAuto L2CoralKTeleOpAutomationB = new PathPlannerAuto("L2CoralKTeleOpAutomationB");
    public final PathPlannerAuto L2CoralLTeleOpAutomationB = new PathPlannerAuto("L2CoralLTeleOpAutomationB");
    public final PathPlannerAuto L3CoralATeleOpAutomationB = new PathPlannerAuto("L3CoralATeleOpAutomationB");
    public final PathPlannerAuto L3CoralBTeleOpAutomationB = new PathPlannerAuto("L3CoralBTeleOpAutomationB");
    public final PathPlannerAuto L3CoralCTeleOpAutomationB = new PathPlannerAuto("L3CoralCTeleOpAutomationB");
    public final PathPlannerAuto L3CoralDTeleOpAutomationB = new PathPlannerAuto("L3CoralDTeleOpAutomationB");
    public final PathPlannerAuto L3CoralETeleOpAutomationB = new PathPlannerAuto("L3CoralETeleOpAutomationB");
    public final PathPlannerAuto L3CoralFTeleOpAutomationB = new PathPlannerAuto("L3CoralFTeleOpAutomationB");
    public final PathPlannerAuto L3CoralGTeleOpAutomationB = new PathPlannerAuto("L3CoralGTeleOpAutomationB");
    public final PathPlannerAuto L3CoralHTeleOpAutomationB = new PathPlannerAuto("L3CoralHTeleOpAutomationB");
    public final PathPlannerAuto L3CoralITeleOpAutomationB = new PathPlannerAuto("L3CoralITeleOpAutomationB");
    public final PathPlannerAuto L3CoralJTeleOpAutomationB = new PathPlannerAuto("L3CoralJTeleOpAutomationB");
    public final PathPlannerAuto L3CoralKTeleOpAutomationB = new PathPlannerAuto("L3CoralKTeleOpAutomationB");
    public final PathPlannerAuto L3CoralLTeleOpAutomationB = new PathPlannerAuto("L3CoralLTeleOpAutomationB");
    public final PathPlannerAuto L4CoralATeleOpAutomationB = new PathPlannerAuto("L4CoralATeleOpAutomationB");
    public final PathPlannerAuto L4CoralBTeleOpAutomationB = new PathPlannerAuto("L4CoralBTeleOpAutomationB");
    public final PathPlannerAuto L4CoralCTeleOpAutomationB = new PathPlannerAuto("L4CoralCTeleOpAutomationB");
    public final PathPlannerAuto L4CoralDTeleOpAutomationB = new PathPlannerAuto("L4CoralDTeleOpAutomationB");
    public final PathPlannerAuto L4CoralETeleOpAutomationB = new PathPlannerAuto("L4CoralETeleOpAutomationB");
    public final PathPlannerAuto L4CoralFTeleOpAutomationB = new PathPlannerAuto("L4CoralFTeleOpAutomationB");
    public final PathPlannerAuto L4CoralGTeleOpAutomationB = new PathPlannerAuto("L4CoralGTeleOpAutomationB");
    public final PathPlannerAuto L4CoralHTeleOpAutomationB = new PathPlannerAuto("L4CoralHTeleOpAutomationB");
    public final PathPlannerAuto L4CoralITeleOpAutomationB = new PathPlannerAuto("L4CoralITeleOpAutomationB");
    public final PathPlannerAuto L4CoralJTeleOpAutomationB = new PathPlannerAuto("L4CoralJTeleOpAutomationB");
    public final PathPlannerAuto L4CoralKTeleOpAutomationB = new PathPlannerAuto("L4CoralKTeleOpAutomationB");
    public final PathPlannerAuto L4CoralLTeleOpAutomationB = new PathPlannerAuto("L4CoralLTeleOpAutomationB");
    
    // Path Follower
    private final SendableChooser<Command> autoChooser;


    public RobotContainer() {
        NamedCommands.registerCommand("CoralEEAutoOuttake", new CoralEEAutoOuttake(coralEE, 2.0, 3.0));
        NamedCommands.registerCommand("ElementLiftAutoHeightHumanPlayer", new ElementLiftAutoHeight(elementLift, 11.0));
        NamedCommands.registerCommand("ElementLiftAutoHeightL1", new ElementLiftAutoHeight(elementLift, 18.0));
        NamedCommands.registerCommand("ElementLiftAutoHeightL2", new ElementLiftAutoHeight(elementLift, 32.0));
        NamedCommands.registerCommand("ElementLiftAutoHeightL3", new ElementLiftAutoHeight(elementLift, 48.0));
        NamedCommands.registerCommand("ElementLiftAutoHeightL4", new ElementLiftAutoHeight(elementLift, 72.0));
        
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
            if(Math.abs(joystick2.getY()) > 0.065){  //Joystick deadzone of 0.05
                if(elementLift.getEncoderPosition() < elementLiftConstants.liftMinEncoder){
                    elementLift.setVoltage(1);
                }
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
        
        if(joystick3.getRawButton(5) == true)   //This button needs to change based off of what we want on the joystick
        {
            HumanPlayerStation = true;
        }
        if(joystick3.getRawButton(6) == true)   //This button needs to change based off of what we want on the joystick
        {
            HumanPlayerStation = false;
        }

        if(HumanPlayerStation){   //Logic for determining Human Player Station A vs B (Left vs Right) goes here
            CoralL1ReefA.whileTrue(L1CoralATeleOpAutomationA);
            CoralL1ReefB.whileTrue(L1CoralBTeleOpAutomationA);
            CoralL1ReefC.whileTrue(L1CoralCTeleOpAutomationA);
            CoralL1ReefD.whileTrue(L1CoralDTeleOpAutomationA);
            CoralL1ReefE.whileTrue(L1CoralETeleOpAutomationA);
            CoralL1ReefF.whileTrue(L1CoralFTeleOpAutomationA);
            CoralL1ReefG.whileTrue(L1CoralGTeleOpAutomationA);
            CoralL1ReefH.whileTrue(L1CoralHTeleOpAutomationA);
            CoralL1ReefI.whileTrue(L1CoralITeleOpAutomationA);
            CoralL1ReefJ.whileTrue(L1CoralJTeleOpAutomationA);
            CoralL1ReefK.whileTrue(L1CoralKTeleOpAutomationA);
            CoralL1ReefL.whileTrue(L1CoralLTeleOpAutomationA);
            CoralL2ReefA.whileTrue(L2CoralATeleOpAutomationA);
            CoralL2ReefB.whileTrue(L2CoralBTeleOpAutomationA);
            CoralL2ReefC.whileTrue(L2CoralCTeleOpAutomationA);
            CoralL2ReefD.whileTrue(L2CoralDTeleOpAutomationA);
            CoralL2ReefE.whileTrue(L2CoralETeleOpAutomationA);
            CoralL2ReefF.whileTrue(L2CoralFTeleOpAutomationA);
            CoralL2ReefG.whileTrue(L2CoralGTeleOpAutomationA);
            CoralL2ReefH.whileTrue(L2CoralHTeleOpAutomationA);
            CoralL2ReefI.whileTrue(L2CoralITeleOpAutomationA);
            CoralL2ReefJ.whileTrue(L2CoralJTeleOpAutomationA);
            CoralL2ReefK.whileTrue(L2CoralKTeleOpAutomationA);
            CoralL2ReefL.whileTrue(L2CoralLTeleOpAutomationA);
            CoralL3ReefA.whileTrue(L3CoralATeleOpAutomationA);
            CoralL3ReefB.whileTrue(L3CoralBTeleOpAutomationA);
            CoralL3ReefC.whileTrue(L3CoralCTeleOpAutomationA);
            CoralL3ReefD.whileTrue(L3CoralDTeleOpAutomationA);
            CoralL3ReefE.whileTrue(L3CoralETeleOpAutomationA);
            CoralL3ReefF.whileTrue(L3CoralFTeleOpAutomationA);
            CoralL3ReefG.whileTrue(L3CoralGTeleOpAutomationA);
            CoralL3ReefH.whileTrue(L3CoralHTeleOpAutomationA);
            CoralL3ReefI.whileTrue(L3CoralITeleOpAutomationA);
            CoralL3ReefJ.whileTrue(L3CoralJTeleOpAutomationA);
            CoralL3ReefK.whileTrue(L3CoralKTeleOpAutomationA);
            CoralL3ReefL.whileTrue(L3CoralLTeleOpAutomationA);
            CoralL4ReefA.whileTrue(L4CoralATeleOpAutomationA);
            CoralL4ReefB.whileTrue(L4CoralBTeleOpAutomationA);
            CoralL4ReefC.whileTrue(L4CoralCTeleOpAutomationA);
            CoralL4ReefD.whileTrue(L4CoralDTeleOpAutomationA);
            CoralL4ReefE.whileTrue(L4CoralETeleOpAutomationA);
            CoralL4ReefF.whileTrue(L4CoralFTeleOpAutomationA);
            CoralL4ReefG.whileTrue(L4CoralGTeleOpAutomationA);
            CoralL4ReefH.whileTrue(L4CoralHTeleOpAutomationA);
            CoralL4ReefI.whileTrue(L4CoralITeleOpAutomationA);
            CoralL4ReefJ.whileTrue(L4CoralJTeleOpAutomationA);
            CoralL4ReefK.whileTrue(L4CoralKTeleOpAutomationA);
            CoralL4ReefL.whileTrue(L4CoralLTeleOpAutomationA);
        }
        else{
            CoralL1ReefA.whileTrue(L1CoralATeleOpAutomationB);
            CoralL1ReefB.whileTrue(L1CoralBTeleOpAutomationB);
            CoralL1ReefC.whileTrue(L1CoralCTeleOpAutomationB);
            CoralL1ReefD.whileTrue(L1CoralDTeleOpAutomationB);
            CoralL1ReefE.whileTrue(L1CoralETeleOpAutomationB);
            CoralL1ReefF.whileTrue(L1CoralFTeleOpAutomationB);
            CoralL1ReefG.whileTrue(L1CoralGTeleOpAutomationB);
            CoralL1ReefH.whileTrue(L1CoralHTeleOpAutomationB);
            CoralL1ReefJ.whileTrue(L1CoralJTeleOpAutomationB);
            CoralL1ReefK.whileTrue(L1CoralKTeleOpAutomationB);
            CoralL1ReefL.whileTrue(L1CoralLTeleOpAutomationB);
            CoralL2ReefA.whileTrue(L2CoralATeleOpAutomationB);
            CoralL2ReefB.whileTrue(L2CoralBTeleOpAutomationB);
            CoralL2ReefC.whileTrue(L2CoralCTeleOpAutomationB);
            CoralL2ReefD.whileTrue(L2CoralDTeleOpAutomationB);
            CoralL2ReefE.whileTrue(L2CoralETeleOpAutomationB);
            CoralL2ReefG.whileTrue(L2CoralGTeleOpAutomationB);
            CoralL2ReefH.whileTrue(L2CoralHTeleOpAutomationB);
            CoralL2ReefI.whileTrue(L2CoralITeleOpAutomationB);
            CoralL2ReefJ.whileTrue(L2CoralJTeleOpAutomationB);
            CoralL2ReefK.whileTrue(L2CoralKTeleOpAutomationB);
            CoralL2ReefL.whileTrue(L2CoralLTeleOpAutomationB);
            CoralL3ReefB.whileTrue(L3CoralBTeleOpAutomationB);
            CoralL3ReefC.whileTrue(L3CoralCTeleOpAutomationB);
            CoralL3ReefD.whileTrue(L3CoralDTeleOpAutomationB);
            CoralL3ReefE.whileTrue(L3CoralETeleOpAutomationB);
            CoralL3ReefF.whileTrue(L3CoralFTeleOpAutomationB);
            CoralL3ReefG.whileTrue(L3CoralGTeleOpAutomationB);
            CoralL3ReefH.whileTrue(L3CoralHTeleOpAutomationB);
            CoralL3ReefI.whileTrue(L3CoralITeleOpAutomationB);
            CoralL3ReefJ.whileTrue(L3CoralJTeleOpAutomationB);
            CoralL3ReefK.whileTrue(L3CoralKTeleOpAutomationB);
            CoralL3ReefL.whileTrue(L3CoralLTeleOpAutomationB);
            CoralL4ReefA.whileTrue(L4CoralATeleOpAutomationB);
            CoralL4ReefB.whileTrue(L4CoralBTeleOpAutomationB);
            CoralL4ReefC.whileTrue(L4CoralCTeleOpAutomationB);
            CoralL4ReefD.whileTrue(L4CoralDTeleOpAutomationB);
            CoralL4ReefE.whileTrue(L4CoralETeleOpAutomationB);
            CoralL4ReefF.whileTrue(L4CoralFTeleOpAutomationB);
            CoralL4ReefG.whileTrue(L4CoralGTeleOpAutomationB);
            CoralL4ReefH.whileTrue(L4CoralHTeleOpAutomationB);
            CoralL4ReefI.whileTrue(L4CoralITeleOpAutomationB);
            CoralL4ReefJ.whileTrue(L4CoralJTeleOpAutomationB);
            CoralL4ReefK.whileTrue(L4CoralKTeleOpAutomationB);
            CoralL4ReefL.whileTrue(L4CoralLTeleOpAutomationB); 
        }
        
        new JoystickButton(joystick3, 2).whileTrue(new ElementLiftAutoHeight(elementLift, 18.0).repeatedly());
        new JoystickButton(joystick3, 3).whileTrue(new ElementLiftAutoHeight(elementLift, 32.0).repeatedly());
        new JoystickButton(joystick3, 4).whileTrue(new ElementLiftAutoHeight(elementLift, 48.0).repeatedly());
        new JoystickButton(joystick3, 5).whileTrue(new ElementLiftAutoHeight(elementLift, 72.0).repeatedly());
        
        new JoystickButton(joystick3, 1).whileTrue(new RunCommand(()-> {elementLift.resetEncoder();}));

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
