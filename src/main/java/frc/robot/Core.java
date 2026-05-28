// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.align.driverAssist.FixYawToHub;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.KickerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PowerManagement;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.Telemetry;
import frc.robot.utils.aiming.LaunchCalculator;
import frc.robot.utils.aiming.SotmTelemetry;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.NamedCommands;

public class Core {
    // Swerve Stuff
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.7; // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                      // second

    // max angular velocity
    private Command pathfindingCommand;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final SotmTelemetry sotmTelemetry = new SotmTelemetry();

    // Controllers

    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);
    private final CommandXboxController customController = new CommandXboxController(2);

    // Subsystems

    public final DriveSubsystem drivetrain = TunerConstants.createDrivetrain();

    public final VisionSubsystem vision = new VisionSubsystem();

    public final LaunchCalculator launchCalculator = new LaunchCalculator(drivetrain);

    public final HopperSubsystem hopper = new HopperSubsystem();

    public final KickerSubsystem kicker = new KickerSubsystem();

    public final HoodSubsystem hood = new HoodSubsystem();

    public final ShooterSubsystem shooter = new ShooterSubsystem(hopper, kicker, hood, drivetrain, launchCalculator);

    public final IntakeSubsystem intake = new IntakeSubsystem();

    public final PowerManagement powerManager = new PowerManagement(drivetrain, hopper, intake, shooter);

    // Auto

    private final SendableChooser<Command> autoChooser;

    // Driver assist

    private final FixYawToHub fixYawToHub = new FixYawToHub(drivetrain, launchCalculator);

    private boolean hubYawAlign = false;
    private boolean fastMode = false;

    private static final double TranslationalAccelerationLimit = 10; // meters per second^2
    private static final double RotationalAccelerationLimit = Math.PI * 7.5; // radians per second^2

    private final SlewRateLimiter xRateLimiter = new SlewRateLimiter(TranslationalAccelerationLimit);
    private final SlewRateLimiter yRateLimiter = new SlewRateLimiter(TranslationalAccelerationLimit);
    private final SlewRateLimiter omegaRateLimiter = new SlewRateLimiter(RotationalAccelerationLimit);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public Core() {
        configureBindings();

        configureAutoCommands();
        autoChooser = AutoBuilder.buildAutoChooser();

        configureShuffleBoard();
    }

    public void configureAutoCommands() {
        NamedCommands.registerCommand("Shoot", new InstantCommand(() -> shooter.autoShoot()));
        NamedCommands.registerCommand("Force Shoot", new InstantCommand(() -> shooter.manualShoot()));
        NamedCommands.registerCommand("Stop Shoot", new InstantCommand(() -> shooter.stopAll()));

        NamedCommands.registerCommand("Start Intake", new InstantCommand(() -> intake.rotate(4000))); // 4000
        NamedCommands.registerCommand("Stop Intake", intake.stopCommand());

        NamedCommands.registerCommand("Deploy Intake", new InstantCommand(() -> intake.setPivotNormalized(0.5)));
        NamedCommands.registerCommand("Deploy Intake Full", new InstantCommand(() -> intake.setPivotNormalized(0)));
        NamedCommands.registerCommand("Stop Deploy", new InstantCommand(() -> intake.stopPivot()));

        NamedCommands.registerCommand("Fluctuate Intake", intake.fluctuatingIntakeCommand());

        NamedCommands.registerCommand("Compat Fluc On", new InstantCommand(() -> intake.fluctuatingIntakeOn()));
        NamedCommands.registerCommand("Compat Fluc Off", new InstantCommand(() -> intake.fluctuatingIntakeOff()));
    }

    public void configureShuffleBoard() {
        ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
        ShuffleboardTab hopperTab = Shuffleboard.getTab("Hopper");
        ShuffleboardTab kickerTab = Shuffleboard.getTab("Kicker");
        ShuffleboardTab shooterTab = Shuffleboard.getTab("Shooter");
        ShuffleboardTab hoodTab = Shuffleboard.getTab("Hood");
        ShuffleboardTab Tab = Shuffleboard.getTab("Tab");

        intakeTab.addDouble("Speed Intake", () -> intake.getSpeedRPM());
        intakeTab.addDouble("Target Speed Intake", () -> intake.getTargetRPM());
        intakeTab.addBoolean("Intaking", () -> intake.isIntaking());

        hopperTab.addBoolean("Hopping", () -> hopper.isRolling());

        kickerTab.addDouble("Speed Kicker", () -> kicker.getSpeedRPMKicker());
        kickerTab.addDouble("Target Speed Kicker", () -> kicker.getTargetRPMKicker());
        kickerTab.addBoolean("Kicking", () -> kicker.isKicking());

        shooterTab.addDouble("Speed Shooter", () -> shooter.getSpeedRPM());
        shooterTab.addDouble("Target Speed Shooter", () -> shooter.getTargetRPM());
        shooterTab.addBoolean("Shooting", () -> shooter.isRunning());
        shooterTab.addDouble("Shooter Adjustment", () -> shooter.getShooterAdjustment());
    
        shooterTab.addBoolean("Auto Range Enabled", () -> shooter.isAutoRangeEnabled());

        hoodTab.addDouble("Hood Position", () -> hood.getHoodPosition());
        hoodTab.addDouble("Hood Target", () -> hood.getHoodTargetPosition());
        hoodTab.addBoolean("Hood Down", () -> hood.isHoodDown()); 

        Tab.addDouble("Drivetrain X", () -> drivetrain.getEstimatedPose().getX());
        Tab.addDouble("Drivetrain Y", () -> drivetrain.getEstimatedPose().getY());

        Tab.addDouble("Dist To Hub", () -> Math.round((double) shooter.getDistToHub() * 100.0) / 100.0);
        Tab.addDouble("Time", () -> getPhaseInfo().phaseTime);

        Tab.addBoolean("Our Hub Active", () -> getPhaseInfo().phaseActive);
        Tab.addString("Hub Warnings", () -> getHubActivityStatus());

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    String hubActivityStatus = "";

    public boolean getIsOurHubActive() {
        String gameData = DriverStation.getGameSpecificMessage();
        Optional<Alliance> ourAlliance = DriverStation.getAlliance();
        if (gameData.length() == 0) {
            hubActivityStatus = "NOT READY";
            return false;
        }
        if (ourAlliance.isEmpty()) {
            hubActivityStatus = "NO STATION";
            return false;
        }
        switch (gameData.charAt(0)) {
            case 'B':
                if (ourAlliance.get() == Alliance.Blue) {
                    return true;
                }
            case 'R':
                if (ourAlliance.get() == Alliance.Red) {
                    return true;
                }
            default:
                hubActivityStatus = "BAD DATA";
                return false;
        }
    }

    public String getHubActivityStatus() {
        return hubActivityStatus;
    }

    public class PhaseInfo {
        public int phaseTime;
        public boolean phaseActive;

        public PhaseInfo(int phaseTime, boolean phaseActive) {
            this.phaseTime = phaseTime;
            this.phaseActive = phaseActive;
        }
    }

    public PhaseInfo getPhaseInfo() {
        int matchTime = (int) DriverStation.getMatchTime();
        boolean isAutonomous = DriverStation.isAutonomous();
        if (isAutonomous) {
            return new PhaseInfo(matchTime, true);
        } else {
            if (matchTime > 130) {
                return new PhaseInfo(matchTime - 130, true);
            } else if (matchTime > 105) {
                return new PhaseInfo(matchTime - 105, getIsOurHubActive());
            } else if (matchTime > 80) {
                return new PhaseInfo(matchTime - 80, getIsOurHubActive());
            } else if (matchTime > 55) {
                return new PhaseInfo(matchTime - 55, getIsOurHubActive());
            } else if (matchTime > 30) {
                return new PhaseInfo(matchTime - 30, getIsOurHubActive());
            } else {
                return new PhaseInfo(matchTime, true);
            }
        }
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> {
                    double axisScale = 1;

                    double triggerScale = getTriggerAxisMovementScale();

                    if (triggerScale != 1) {
                        axisScale = triggerScale;
                    }

                    double driverVelocityX = driveController.getLeftY() * MaxSpeed * axisScale;
                    double driverVelocityY = driveController.getLeftX() * MaxSpeed * axisScale;
                    double driverRotationalRate = -driveController.getRightX() * MaxAngularRate * axisScale;

                    // Determine which controller is active
                    // boolean driverActive =
                    // Math.abs(driverVelocityX) > 0.05 ||
                    // Math.abs(driverVelocityY) > 0.05 ||
                    // Math.abs(driverRotationalRate) > 0.05;
                    boolean driverActive = Math.abs(driveController.getRightX()) > 0.1 || !hubYawAlign;

                    double desiredRotationalRate = driverActive ? driverRotationalRate : calculateRotationalRate();

                    if (Math.abs(driveController.getLeftY()) < 0.1 && Math.abs(driveController.getLeftX()) < 0.1
                            && Math.abs(driveController.getRightX()) < 0.1 && !hubYawAlign) {
                        xRateLimiter.reset(0);
                        yRateLimiter.reset(0);
                        omegaRateLimiter.reset(0);

                        drivetrain.setCommandedRobotChassisSpeeds(new ChassisSpeeds(0, 0, 0));

                        return brake;
                    }

                    driverVelocityX = xRateLimiter.calculate(driverVelocityX * 0.5);
                    driverVelocityY = yRateLimiter.calculate(driverVelocityY * 0.5);
                    desiredRotationalRate = omegaRateLimiter.calculate(desiredRotationalRate * 0.75);

                    drivetrain.setCommandedRobotChassisSpeeds(new ChassisSpeeds(
                            -driverVelocityX,
                            -driverVelocityY,
                            desiredRotationalRate));

                    return drive
                            .withVelocityX(-driverVelocityX) // Limit translational acceleration forward/backward
                            .withVelocityY(-driverVelocityY) // Limit translational acceleration left/right
                            .withRotationalRate(desiredRotationalRate);
                }));

        // DRIVER BINDINGS

        // reset the field-centric heading on left bumper press
        driveController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));

        driveController.leftTrigger().onTrue(new InstantCommand(() -> {
            CommandScheduler.getInstance().schedule(fixYawToHub);
            hubYawAlign = true;
        }));

        driveController.leftTrigger().onFalse(new InstantCommand(() -> {
            CommandScheduler.getInstance().cancel(fixYawToHub);
            hubYawAlign = false;
        }));

        // OPERATOR BINDINGS

        driveController.b().onTrue(intake.setPivotZeroCommand());

        driveController.rightBumper().onTrue(new InstantCommand(() -> toggleFastMode()));

        Pose2d targetPose2d = new Pose2d(15.540988, 7.069326, new Rotation2d());

        driveController.start().onTrue(createPathfindingCommand(targetPose2d));

        operatorController.a().toggleOnTrue(intake.intakeCommand());
        operatorController.b().onTrue(hood.manualToggleHoodMinMaxCommand());
        operatorController.x().toggleOnTrue(intake.purgeCommand());
        operatorController.y().toggleOnTrue(shooter.autoShoot());

        operatorController.povUp().whileTrue(intake.lowerManual()).onFalse(intake.stopPivot());
        operatorController.povRight().onTrue(intake.changeTargetRPMCommand(100));

        operatorController.povDown().whileTrue(intake.raiseManual()).onFalse(intake.stopPivot());
        operatorController.povLeft().onTrue(intake.changeTargetRPMCommand(-100));

        operatorController.rightBumper().onTrue(kicker.changeKickerTargetRPMCommand(100));
        operatorController.rightTrigger().onTrue(shooter.changeShooterAdjustmentCommand(100));

        operatorController.leftBumper().onTrue(kicker.changeKickerTargetRPMCommand(-100));
        operatorController.leftTrigger().onTrue(shooter.changeShooterAdjustmentCommand(-100));

        operatorController.start().onTrue(powerManager.toggleDriveBoost());
        operatorController.back().onTrue(shooter.toggleAutoRangeCommand());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public double getTriggerAxisMovementScale() {
        return (1 - (driveController.getRightTriggerAxis() * 0.75));
    }

    public boolean getBumpAxisMovementScale() {
        return driveController.rightBumper().getAsBoolean();
    }

    public void toggleFastMode() {
        if (fastMode) {
            MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.7;
        } else {
            MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.85;
        }

        fastMode = !fastMode;
    }

    private double calculateRotationalRate() {
        return fixYawToHub.getRotationalRate();
    }

    public void publishSotmTelemetry() {
        var params = launchCalculator.getParameters();
        sotmTelemetry.publish(params, frc.robot.utils.Constants.ENABLE_SHOOT_ON_MOVE, launchCalculator.atDriveGoal());
    }

    public void clearLaunchCalculatorCache() {
        launchCalculator.clearCachedParameters();
    }

    public Command createPathfindingCommand(Pose2d target) {
        PathConstraints constraints = new PathConstraints(
                .5, 2, // 3 - 4
                Units.degreesToRadians(540),
                Units.degreesToRadians(720));

        System.out.println(target);

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        pathfindingCommand = AutoBuilder.pathfindToPose(
                target,
                constraints,
                0);

        return pathfindingCommand;
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

    public void periodic() {
        
    }

}