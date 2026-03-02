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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.align.alignUtils.Target;
import frc.robot.align.driverAssist.FixYawToHub;
import frc.robot.align.preciseAligning.ClimbAlign;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PowerManagement;
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
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.7; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.65; // 3/4 of a rotation per second
                                                                                      // max angular velocity
    private Command pathfindingCommand;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //Controllers

    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    //Subsystems

    public final DriveSubsystem drivetrain = TunerConstants.createDrivetrain();

    public final VisionSubsystem vision = new VisionSubsystem();

    public final HopperSubsystem hopper = new HopperSubsystem();

    public final ShooterSubsystem shooter = new ShooterSubsystem(hopper, false, drivetrain);

    public final IntakeSubsystem intake = new IntakeSubsystem();

    public final PowerManagement powerManager = new PowerManagement(drivetrain, hopper, intake, shooter);

    //Auto

     private final SendableChooser<Command> autoChooser;

    //Driver assist
    
    private final FixYawToHub fixYawToHub = new FixYawToHub(drivetrain, false);

    private final Target testTarget = new Target(31, new Transform3d(1.575, 0.0, 0, new Rotation3d(0, 0, 0)));

    private boolean hubYawAlign = false;

    private static final double TranslationalAccelerationLimit = 10; // meters per second^2
    private static final double RotationalAccelerationLimit = Math.PI * 7.5; // radians per second^2

    private final SlewRateLimiter xRateLimiter = new SlewRateLimiter(TranslationalAccelerationLimit);
    private final SlewRateLimiter yRateLimiter = new SlewRateLimiter(TranslationalAccelerationLimit);
    private final SlewRateLimiter omegaRateLimiter = new SlewRateLimiter(RotationalAccelerationLimit);

    public Core() {
        configureBindings();

        configureAutoCommands();
        autoChooser = AutoBuilder.buildAutoChooser();

        configureShuffleBoard();
    }

    public void configureAutoCommands() {
        NamedCommands.registerCommand("Shoot", new InstantCommand(() -> shooter.autoShoot())); //WILL WORK WHEN EHTAN'S CODE IS PUSHED IN
        NamedCommands.registerCommand("Stop Shoot", shooter.stopShooterCommand()); //^^
        
        NamedCommands.registerCommand("Start Intake", intake.intakeCommand());
        NamedCommands.registerCommand("Stop Intake", intake.stopCommand());

        NamedCommands.registerCommand("Deploy Intake", intake.lowerManual());
        NamedCommands.registerCommand("Stop Deploy", intake.stopPivot());
    }

    public void configureShuffleBoard() {
        ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
        ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
        ShuffleboardTab Tab = Shuffleboard.getTab("Tab");

        intakeTab.addDouble("Speed Intake", () -> intake.getSpeedRPM());
        intakeTab.addDouble("Target Speed Intake", () -> intake.getTargetRPM());
        intakeTab.addBoolean("Intaking", () -> intake.isIntaking());

        shooterTab.addDouble("Speed Shooter", () -> shooter.getSpeedRPM());
        shooterTab.addDouble("Target Speed Shooter", () -> shooter.getTargetRPM());
        shooterTab.addDouble("Speed Kicker", () -> shooter.getSpeedRPMKicker());
        shooterTab.addDouble("Target Speed Kicker", () -> shooter.getTargetRPMKicker());
        shooterTab.addBoolean("Shooting", () -> shooter.isRunning());
        shooterTab.addBoolean("Kicking", () -> shooter.isKicking());
    
        Tab.addDouble("Drivetrain X", () -> drivetrain.getEstimator().getX());
        Tab.addDouble("Drivetrain Y", () -> drivetrain.getEstimator().getY());
        Tab.addDouble("Dist To Hub", () -> shooter.getDistToHub());
        Tab.addDouble("Time", () -> DriverStation.getMatchTime());

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                double axisScale = 1;

                if (getBumpAxisMovementScale()) {
                    axisScale = 0.5;
                }

                double triggerScale = getTriggerAxisMovementScale();

                if (triggerScale != 1) {
                    axisScale = triggerScale;
                }

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
                        .withVelocityX(-driverVelocityX) // Limit translational acceleration forward/backward
                        .withVelocityY(-driverVelocityY) // Limit translational acceleration left/right
                        .withRotationalRate(desiredRotationalRate);
            })
        );

        // DRIVER BINDINGS

        // reset the field-centric heading on left bumper press
        driveController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driveController.leftTrigger().onTrue(new InstantCommand(() -> {
            CommandScheduler.getInstance().schedule(fixYawToHub);
            hubYawAlign = true;}));

        driveController.leftTrigger().onFalse(new InstantCommand(() -> {
            CommandScheduler.getInstance().cancel(fixYawToHub);
            hubYawAlign = false;}));

        // OPERATOR BINDINGS

        
        operatorController.a().toggleOnTrue(intake.intakeCommand());
        operatorController.b().onTrue(hopper.changeRPMCommand(100));
        operatorController.x().whileTrue(intake.setRPMCommand(-Math.abs(intake.getTargetRPM())));
        operatorController.x().whileFalse(intake.setRPMCommand(Math.abs(intake.getTargetRPM())));
        operatorController.y().onTrue(new InstantCommand(() -> shooter.autoShoot()));

        
        operatorController.povUp().whileTrue(intake.lowerManual()).onFalse(intake.stopPivot());
        operatorController.povRight().onTrue(intake.changeTargetRPMCommand(100));
        operatorController.povDown().whileTrue(intake.raiseManual()).onFalse(intake.stopPivot());
        operatorController.povLeft().onTrue(intake.changeTargetRPMCommand(-100));

        operatorController.rightBumper().onTrue(new InstantCommand(() -> shooter.changeKickerTargetRPM(100)));
        operatorController.rightTrigger().onTrue(new InstantCommand(() -> shooter.changeTargetRPM(100)));
        operatorController.leftBumper().onTrue(new InstantCommand(() -> shooter.changeKickerTargetRPM(-100)));
        operatorController.leftTrigger().onTrue(new InstantCommand(() -> shooter.changeTargetRPM(-100)));

        operatorController.start().onTrue(hopper.pulseCommand());
        operatorController.back().onTrue(new InstantCommand(() -> shooter.toggleAutoRange()));


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public double getTriggerAxisMovementScale() {
        return (1 - (driveController.getRightTriggerAxis() * 0.75));
    }

    public boolean getBumpAxisMovementScale() {
        return driveController.rightBumper().getAsBoolean();
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