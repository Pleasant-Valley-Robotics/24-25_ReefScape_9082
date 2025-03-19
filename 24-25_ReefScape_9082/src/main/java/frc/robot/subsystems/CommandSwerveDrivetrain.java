package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.LEDs.ShowPattern;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private Field2d field = new Field2d();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation  The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }
    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
        SmartDashboard.putNumber("Pose X", this.getState().Pose.getX());
        SmartDashboard.putNumber("Pose Y", this.getState().Pose.getY());
        SmartDashboard.putNumber("Pose Rot", this.getState().Pose.getRotation().getDegrees());
        //Coral A Alignment
        if((this.getState().Pose.getX() >= Constants.coralAlignmentConstants.coralAX-Constants.coralAlignmentConstants.translativeTolerance)&&(this.getState().Pose.getX() <= Constants.coralAlignmentConstants.coralAX+Constants.coralAlignmentConstants.translativeTolerance)){
            if((this.getState().Pose.getY() >= Constants.coralAlignmentConstants.coralAY-Constants.coralAlignmentConstants.translativeTolerance)&&(this.getState().Pose.getY() <= Constants.coralAlignmentConstants.coralAY+Constants.coralAlignmentConstants.translativeTolerance)){
                if((this.getState().Pose.getRotation().getDegrees() >= Constants.coralAlignmentConstants.coralARot-Constants.coralAlignmentConstants.rotationalTolerance)&&(this.getState().Pose.getRotation().getDegrees() <= Constants.coralAlignmentConstants.coralARot+Constants.coralAlignmentConstants.rotationalTolerance)){
                    if(RobotContainer.LEDs.showPattern != ShowPattern.solidGreen){
                        RobotContainer.LEDs.showPattern = ShowPattern.solidGreen;
                    }
                }
            }
        }
        //Coral B Alignment
        else if((this.getState().Pose.getX() >= Constants.coralAlignmentConstants.coralBX-Constants.coralAlignmentConstants.translativeTolerance)&&(this.getState().Pose.getX() <= Constants.coralAlignmentConstants.coralBX+Constants.coralAlignmentConstants.translativeTolerance)){
            if((this.getState().Pose.getY() >= Constants.coralAlignmentConstants.coralBY-Constants.coralAlignmentConstants.translativeTolerance)&&(this.getState().Pose.getY() <= Constants.coralAlignmentConstants.coralBY+Constants.coralAlignmentConstants.translativeTolerance)){
                if((this.getState().Pose.getRotation().getDegrees() >= Constants.coralAlignmentConstants.coralBRot-Constants.coralAlignmentConstants.rotationalTolerance)&&(this.getState().Pose.getRotation().getDegrees() <= Constants.coralAlignmentConstants.coralBRot+Constants.coralAlignmentConstants.rotationalTolerance)){
                    if(RobotContainer.LEDs.showPattern != ShowPattern.solidGreen){
                        RobotContainer.LEDs.showPattern = ShowPattern.solidGreen;
                    }
                }
            }
        }
        //Coral C Alignment
        else if((this.getState().Pose.getX() >= Constants.coralAlignmentConstants.coralCX-Constants.coralAlignmentConstants.translativeTolerance)&&(this.getState().Pose.getX() <= Constants.coralAlignmentConstants.coralCX+Constants.coralAlignmentConstants.translativeTolerance)){
            if((this.getState().Pose.getY() >= Constants.coralAlignmentConstants.coralCY-Constants.coralAlignmentConstants.translativeTolerance)&&(this.getState().Pose.getY() <= Constants.coralAlignmentConstants.coralCY+Constants.coralAlignmentConstants.translativeTolerance)){
                if((this.getState().Pose.getRotation().getDegrees() >= Constants.coralAlignmentConstants.coralCRot-Constants.coralAlignmentConstants.rotationalTolerance)&&(this.getState().Pose.getRotation().getDegrees() <= Constants.coralAlignmentConstants.coralCRot+Constants.coralAlignmentConstants.rotationalTolerance)){
                    if(RobotContainer.LEDs.showPattern != ShowPattern.solidGreen){
                        RobotContainer.LEDs.showPattern = ShowPattern.solidGreen;
                    }
                }
            }
        }
        //Coral D Alignment
        else if((this.getState().Pose.getX() >= Constants.coralAlignmentConstants.coralDX-Constants.coralAlignmentConstants.translativeTolerance)&&(this.getState().Pose.getX() <= Constants.coralAlignmentConstants.coralDX+Constants.coralAlignmentConstants.translativeTolerance)){
            if((this.getState().Pose.getY() >= Constants.coralAlignmentConstants.coralDY-Constants.coralAlignmentConstants.translativeTolerance)&&(this.getState().Pose.getY() <= Constants.coralAlignmentConstants.coralDY+Constants.coralAlignmentConstants.translativeTolerance)){
                if((this.getState().Pose.getRotation().getDegrees() >= Constants.coralAlignmentConstants.coralDRot-Constants.coralAlignmentConstants.rotationalTolerance)&&(this.getState().Pose.getRotation().getDegrees() <= Constants.coralAlignmentConstants.coralDRot+Constants.coralAlignmentConstants.rotationalTolerance)){
                    if(RobotContainer.LEDs.showPattern != ShowPattern.solidGreen){
                        RobotContainer.LEDs.showPattern = ShowPattern.solidGreen;
                    }
                }
            }
        }
        //Coral E Alignment
        else if((this.getState().Pose.getX() >= Constants.coralAlignmentConstants.coralEX-Constants.coralAlignmentConstants.translativeTolerance)&&(this.getState().Pose.getX() <= Constants.coralAlignmentConstants.coralEX+Constants.coralAlignmentConstants.translativeTolerance)){
            if((this.getState().Pose.getY() >= Constants.coralAlignmentConstants.coralEY-Constants.coralAlignmentConstants.translativeTolerance)&&(this.getState().Pose.getY() <= Constants.coralAlignmentConstants.coralEY+Constants.coralAlignmentConstants.translativeTolerance)){
                if((this.getState().Pose.getRotation().getDegrees() >= Constants.coralAlignmentConstants.coralERot-Constants.coralAlignmentConstants.rotationalTolerance)&&(this.getState().Pose.getRotation().getDegrees() <= Constants.coralAlignmentConstants.coralERot+Constants.coralAlignmentConstants.rotationalTolerance)){
                    if(RobotContainer.LEDs.showPattern != ShowPattern.solidGreen){
                        RobotContainer.LEDs.showPattern = ShowPattern.solidGreen;
                    }
                }
            }
        }
        //Coral F Alignment
        else if((this.getState().Pose.getX() >= Constants.coralAlignmentConstants.coralFX-Constants.coralAlignmentConstants.translativeTolerance)&&(this.getState().Pose.getX() <= Constants.coralAlignmentConstants.coralFX+Constants.coralAlignmentConstants.translativeTolerance)){
            if((this.getState().Pose.getY() >= Constants.coralAlignmentConstants.coralFY-Constants.coralAlignmentConstants.translativeTolerance)&&(this.getState().Pose.getY() <= Constants.coralAlignmentConstants.coralFY+Constants.coralAlignmentConstants.translativeTolerance)){
                if((this.getState().Pose.getRotation().getDegrees() >= Constants.coralAlignmentConstants.coralFRot-Constants.coralAlignmentConstants.rotationalTolerance)&&(this.getState().Pose.getRotation().getDegrees() <= Constants.coralAlignmentConstants.coralFRot+Constants.coralAlignmentConstants.rotationalTolerance)){
                    if(RobotContainer.LEDs.showPattern != ShowPattern.solidGreen){
                        RobotContainer.LEDs.showPattern = ShowPattern.solidGreen;
                    }
                }
            }
        }
        //Coral G Alignment
        else if((this.getState().Pose.getX() >= Constants.coralAlignmentConstants.coralGX-Constants.coralAlignmentConstants.translativeTolerance)&&(this.getState().Pose.getX() <= Constants.coralAlignmentConstants.coralGX+Constants.coralAlignmentConstants.translativeTolerance)){
            if((this.getState().Pose.getY() >= Constants.coralAlignmentConstants.coralGY-Constants.coralAlignmentConstants.translativeTolerance)&&(this.getState().Pose.getY() <= Constants.coralAlignmentConstants.coralGY+Constants.coralAlignmentConstants.translativeTolerance)){
                if((this.getState().Pose.getRotation().getDegrees() >= Constants.coralAlignmentConstants.coralGRot-Constants.coralAlignmentConstants.rotationalTolerance)&&(this.getState().Pose.getRotation().getDegrees() <= Constants.coralAlignmentConstants.coralGRot+Constants.coralAlignmentConstants.rotationalTolerance)){
                    if(RobotContainer.LEDs.showPattern != ShowPattern.solidGreen){
                        RobotContainer.LEDs.showPattern = ShowPattern.solidGreen;
                    }
                }
            }
        }
        //Coral H Alignment
        else if((this.getState().Pose.getX() >= Constants.coralAlignmentConstants.coralHX-Constants.coralAlignmentConstants.translativeTolerance)&&(this.getState().Pose.getX() <= Constants.coralAlignmentConstants.coralHX+Constants.coralAlignmentConstants.translativeTolerance)){
            if((this.getState().Pose.getY() >= Constants.coralAlignmentConstants.coralHY-Constants.coralAlignmentConstants.translativeTolerance)&&(this.getState().Pose.getY() <= Constants.coralAlignmentConstants.coralHY+Constants.coralAlignmentConstants.translativeTolerance)){
                if((this.getState().Pose.getRotation().getDegrees() >= Constants.coralAlignmentConstants.coralHRot-Constants.coralAlignmentConstants.rotationalTolerance)&&(this.getState().Pose.getRotation().getDegrees() <= Constants.coralAlignmentConstants.coralHRot+Constants.coralAlignmentConstants.rotationalTolerance)){
                    if(RobotContainer.LEDs.showPattern != ShowPattern.solidGreen){
                        RobotContainer.LEDs.showPattern = ShowPattern.solidGreen;
                    }
                }
            }
        }
        //Coral I Alignment
        else if((this.getState().Pose.getX() >= Constants.coralAlignmentConstants.coralIX-Constants.coralAlignmentConstants.translativeTolerance)&&(this.getState().Pose.getX() <= Constants.coralAlignmentConstants.coralIX+Constants.coralAlignmentConstants.translativeTolerance)){
            if((this.getState().Pose.getY() >= Constants.coralAlignmentConstants.coralIY-Constants.coralAlignmentConstants.translativeTolerance)&&(this.getState().Pose.getY() <= Constants.coralAlignmentConstants.coralIY+Constants.coralAlignmentConstants.translativeTolerance)){
                if((this.getState().Pose.getRotation().getDegrees() >= Constants.coralAlignmentConstants.coralIRot-Constants.coralAlignmentConstants.rotationalTolerance)&&(this.getState().Pose.getRotation().getDegrees() <= Constants.coralAlignmentConstants.coralIRot+Constants.coralAlignmentConstants.rotationalTolerance)){
                    if(RobotContainer.LEDs.showPattern != ShowPattern.solidGreen){
                        RobotContainer.LEDs.showPattern = ShowPattern.solidGreen;
                    }
                }
            }
        }
        //Coral J Alignment
        else if((this.getState().Pose.getX() >= Constants.coralAlignmentConstants.coralJX-Constants.coralAlignmentConstants.translativeTolerance)&&(this.getState().Pose.getX() <= Constants.coralAlignmentConstants.coralJX+Constants.coralAlignmentConstants.translativeTolerance)){
            if((this.getState().Pose.getY() >= Constants.coralAlignmentConstants.coralJY-Constants.coralAlignmentConstants.translativeTolerance)&&(this.getState().Pose.getY() <= Constants.coralAlignmentConstants.coralJY+Constants.coralAlignmentConstants.translativeTolerance)){
                if((this.getState().Pose.getRotation().getDegrees() >= Constants.coralAlignmentConstants.coralJRot-Constants.coralAlignmentConstants.rotationalTolerance)&&(this.getState().Pose.getRotation().getDegrees() <= Constants.coralAlignmentConstants.coralJRot+Constants.coralAlignmentConstants.rotationalTolerance)){
                    if(RobotContainer.LEDs.showPattern != ShowPattern.solidGreen){
                        RobotContainer.LEDs.showPattern = ShowPattern.solidGreen;
                    }
                }
            }
        }
        //Coral K Alignment
        else if((this.getState().Pose.getX() >= Constants.coralAlignmentConstants.coralKX-Constants.coralAlignmentConstants.translativeTolerance)&&(this.getState().Pose.getX() <= Constants.coralAlignmentConstants.coralKX+Constants.coralAlignmentConstants.translativeTolerance)){
            if((this.getState().Pose.getY() >= Constants.coralAlignmentConstants.coralKY-Constants.coralAlignmentConstants.translativeTolerance)&&(this.getState().Pose.getY() <= Constants.coralAlignmentConstants.coralKY+Constants.coralAlignmentConstants.translativeTolerance)){
                if((this.getState().Pose.getRotation().getDegrees() >= Constants.coralAlignmentConstants.coralKRot-Constants.coralAlignmentConstants.rotationalTolerance)&&(this.getState().Pose.getRotation().getDegrees() <= Constants.coralAlignmentConstants.coralKRot+Constants.coralAlignmentConstants.rotationalTolerance)){
                    if(RobotContainer.LEDs.showPattern != ShowPattern.solidGreen){
                        RobotContainer.LEDs.showPattern = ShowPattern.solidGreen;
                    }
                }
            }
        }
        //Coral L Alignment
        else if((this.getState().Pose.getX() >= Constants.coralAlignmentConstants.coralLX-Constants.coralAlignmentConstants.translativeTolerance)&&(this.getState().Pose.getX() <= Constants.coralAlignmentConstants.coralLX+Constants.coralAlignmentConstants.translativeTolerance)){
            if((this.getState().Pose.getY() >= Constants.coralAlignmentConstants.coralLY-Constants.coralAlignmentConstants.translativeTolerance)&&(this.getState().Pose.getY() <= Constants.coralAlignmentConstants.coralLY+Constants.coralAlignmentConstants.translativeTolerance)){
                if((this.getState().Pose.getRotation().getDegrees() >= Constants.coralAlignmentConstants.coralLRot-Constants.coralAlignmentConstants.rotationalTolerance)&&(this.getState().Pose.getRotation().getDegrees() <= Constants.coralAlignmentConstants.coralLRot+Constants.coralAlignmentConstants.rotationalTolerance)){
                    if(RobotContainer.LEDs.showPattern != ShowPattern.solidGreen){
                        RobotContainer.LEDs.showPattern = ShowPattern.solidGreen;
                    }
                }
            }
        }             
        else{
            if(RobotContainer.LEDs.showPattern == ShowPattern.solidGreen){
                RobotContainer.LEDs.showPattern = ShowPattern.liftProgress;
            }
        }                           
        SmartDashboard.putData("Field", field);
        field.setRobotPose(this.getState().Pose);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}