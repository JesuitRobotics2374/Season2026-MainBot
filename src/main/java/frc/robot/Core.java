// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.align.alignUtils.Target;
import frc.robot.align.driverAssist.FixYawToHub;
import frc.robot.align.preciseAligning.CanAlign;
import frc.robot.align.preciseAligning.ClimbAlign;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.Telemetry;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;



public class Core {
    //Swerve Stuff
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.75; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity
    private Command pathfindingCommand;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //Controllers

    private final CommandXboxController driveController = new CommandXboxController(0);

    //Subsystems

    public final DriveSubsystem drivetrain = TunerConstants.createDrivetrain();

    public final VisionSubsystem vision = new VisionSubsystem();

    public final ShooterSubsystem m_shooter = new ShooterSubsystem();
    public final IntakeSubsystem m_intake = new IntakeSubsystem();

    //Auto

     private final SendableChooser<Command> autoChooser;

    //Driver assist
    
    private final FixYawToHub fixYawToHub = new FixYawToHub(drivetrain, false);

    private final Target testTarget = new Target(31, new Transform3d(1.575, 0.0, 0, new Rotation3d(0, 0, 0)));

    private final SequentialCommandGroup climbAlign = new SequentialCommandGroup(
            new ClimbAlign(drivetrain, vision, testTarget),
            new CanAlign(drivetrain, vision, testTarget.requestFiducialID().get(), true));

    private boolean hubYawAlign = false;

    private static final double TranslationalAccelerationLimit = 10; // meters per second^2
    private static final double RotationalAccelerationLimit = Math.PI * 5.5; // radians per second^2

    private final SlewRateLimiter xRateLimiter = new SlewRateLimiter(TranslationalAccelerationLimit);
    private final SlewRateLimiter yRateLimiter = new SlewRateLimiter(TranslationalAccelerationLimit);
    private final SlewRateLimiter omegaRateLimiter = new SlewRateLimiter(RotationalAccelerationLimit);

    public Core() {
        configureBindings();

        NamedCommands.registerCommand("far target rpm", new InstantCommand(() -> m_shooter.setTargetRpm(4500)));
        NamedCommands.registerCommand("3m target rpm", new InstantCommand(() -> m_shooter.setTargetRpm(4000)));
        NamedCommands.registerCommand("1.5m target rpm", new InstantCommand(() -> m_shooter.setTargetRpm(3550)));
        NamedCommands.registerCommand("Shoot", new InstantCommand(() -> m_shooter.autoShoot())); //WILL WORK WHEN EHTAN'S CODE IS PUSHED IN
        NamedCommands.registerCommand("Stop Shoot", new InstantCommand(() -> m_shooter.stopAll())); //^^
        
        NamedCommands.registerCommand("Start Intake", new InstantCommand(() -> m_intake.intakeFuel(-0.5)));
        NamedCommands.registerCommand("Stop Intake", new InstantCommand(() -> m_intake.stopIntake()));

        autoChooser = AutoBuilder.buildAutoChooser();
        
        configureShuffleBoard();
    }

    public void configureShuffleBoard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Test");
    }

     public Command getAutonomousCommand() {
        return new PathPlannerAuto("Blue Depot Hub");
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                double axisScale = getAxisMovementScale();

                double driverVelocityX = driveController.getLeftY() * MaxSpeed * axisScale;
                double driverVelocityY = driveController.getLeftX() * MaxSpeed * axisScale;
                double driverRotationalRate = -driveController.getRightX() * MaxAngularRate * axisScale;

                // Determine which controller is active
                // boolean driverActive =
                //     Math.abs(driverVelocityX) > 0.05 ||
                //     Math.abs(driverVelocityY) > 0.05 ||
                //     Math.abs(driverRotationalRate) > 0.05;
                boolean driverActive = Math.abs(driveController.getRightX()) > 0.1 || !hubYawAlign;

                double desiredRotationalRate = driverActive ? driverRotationalRate : calculateRotationalRate();

                    return drive
                        .withVelocityX(xRateLimiter.calculate(-driverVelocityX)) // Limit translational acceleration forward/backward
                        .withVelocityY(yRateLimiter.calculate(-driverVelocityY)) // Limit translational acceleration left/right
                        .withRotationalRate(omegaRateLimiter.calculate(desiredRotationalRate));
            })
        );

        // reset the field-centric heading on left bumper press
        driveController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driveController.a().onTrue(new ClimbAlign(drivetrain, vision, testTarget));
        driveController.b().onTrue(new CanAlign(drivetrain, vision, testTarget.requestFiducialID().get(), false));

        driveController.y().onTrue(vision.runOnce(() -> vision.getTagRelativeToBot(26)));

        driveController.x().onTrue(climbAlign);

        driveController.povUp().onTrue(new InstantCommand(() -> {
            fixYawToHub.schedule();
            hubYawAlign = true;}));

        driveController.povDown().onTrue(new InstantCommand(() -> {
            fixYawToHub.cancel(); 
            hubYawAlign = false;}));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public double getAxisMovementScale() {
        return (1 - (driveController.getRightTriggerAxis() * 0.75));
    }

    private double calculateRotationalRate() {
        return fixYawToHub.getRotationalRate();
    }

    public void doPathfind(Pose2d target) {
        PathConstraints constraints = new PathConstraints(
                3, 4, // 3 - 4
                Units.degreesToRadians(540),
                Units.degreesToRadians(720));

        System.out.println(target);

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        pathfindingCommand = AutoBuilder.pathfindToPose(
                target,
                constraints,
                0);

        pathfindingCommand.schedule();

        System.out.println("PATHFIND TO " + target.toString() + " STARTED");
    }

       public Command getPath(String id) {
        try {
            // Load the path you want to follow using its name in the GUI
            PathPlannerPath path = PathPlannerPath.fromPathFile(id);

            // Create a path following command using AutoBuilder. This will also trigger
            // event markers.
            return AutoBuilder.followPath(path);

        } catch (Exception e) {
            DriverStation.reportError("Pathing failed: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }


}