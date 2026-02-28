// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.align.alignUtils.Target;
import frc.robot.align.driverAssist.FixYawToHub;
import frc.robot.align.preciseAligning.CanAlign;
import frc.robot.align.preciseAligning.ClimbAlign;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.Telemetry;

import com.pathplanner.lib.auto.AutoBuilder;

public class Core {
    //Swerve Stuff
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.75; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

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

    //Auto

     private final SendableChooser<Command> autoChooser;

    //Driver assist
    
    private final FixYawToHub fixYawToHub = new FixYawToHub(drivetrain, false);

    private final Target testTarget = new Target(31, new Transform3d(1.575, 0.0, 0, new Rotation3d(0, 0, 0)));

    private boolean hubYawAlign = false;

    private static final double TranslationalAccelerationLimit = 10; // meters per second^2
    private static final double RotationalAccelerationLimit = Math.PI * 5.5; // radians per second^2

    private final SlewRateLimiter xRateLimiter = new SlewRateLimiter(TranslationalAccelerationLimit);
    private final SlewRateLimiter yRateLimiter = new SlewRateLimiter(TranslationalAccelerationLimit);
    private final SlewRateLimiter omegaRateLimiter = new SlewRateLimiter(RotationalAccelerationLimit);

    public Core() {
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        
        configureShuffleBoard();
    }

    public void configureShuffleBoard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Test");

        tab.addDouble("Speed Shooter", () -> shooter.getSpeedRPM());
        tab.addDouble("Target Speed Shooter", () -> shooter.getTargetRPM());
        
        tab.addDouble("Speed Intake", () -> intake.getSpeedRPM());
        tab.addDouble("Target Speed Intake", () -> intake.getTargetRPM());

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
                        .withVelocityX(-driverVelocityX) // Limit translational acceleration forward/backward
                        .withVelocityY(-driverVelocityY) // Limit translational acceleration left/right
                        .withRotationalRate(desiredRotationalRate);
            })
        );

        // DRIVER BINDINGS

        // reset the field-centric heading on left bumper press
        driveController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driveController.povUp().onTrue(new InstantCommand(() -> {
            fixYawToHub.schedule();
            hubYawAlign = true;}));

        driveController.povDown().onTrue(new InstantCommand(() -> {
            fixYawToHub.cancel(); 
            hubYawAlign = false;}));

        // OPERATOR BINDINGS

        
        operatorController.a().onTrue(intake.intakeCommand());
        operatorController.b().onTrue(hopper.changeRPMCommand(100));
        operatorController.x().onTrue(hopper.changeRPMCommand(-100));
        operatorController.y().onTrue(new InstantCommand(() -> shooter.autoShoot()));

         driveController.povUp().whileTrue(intake.raiseManual()).onFalse(intake.stopPivot());
        operatorController.povRight().onTrue(intake.changeTargetRPMCommand(100));
        driveController.povDown().whileTrue(intake.lowerManual()).onFalse(intake.stopPivot());
        operatorController.povLeft().onTrue(intake.changeTargetRPMCommand(-100));

        operatorController.rightBumper().onTrue(shooter.changeKickerSpeedCommand(100));
        operatorController.rightTrigger().onTrue(shooter.changeTargetRPMCommand(100));
        operatorController.leftBumper().onTrue(shooter.changeKickerSpeedCommand(-100));
        operatorController.leftTrigger().onTrue(shooter.changeTargetRPMCommand(-100));

        operatorController.start().onTrue(hopper.pulseCommand());
        operatorController.back().onTrue(new InstantCommand(() -> shooter.toggleAutoRange()));


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public double getAxisMovementScale() {
        return (1 - (driveController.getRightTriggerAxis() * 0.75));
    }

    private double calculateRotationalRate() {
        return fixYawToHub.getRotationalRate();
    }
}