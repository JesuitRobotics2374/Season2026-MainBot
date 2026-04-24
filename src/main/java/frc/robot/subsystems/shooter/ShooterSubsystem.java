package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.utils.Configs;
import frc.robot.utils.Constants;
import frc.robot.utils.Devices;
import frc.robot.utils.aiming.AimingUtil;
import frc.robot.utils.aiming.LaunchCalculator;
import frc.robot.utils.aiming.LaunchingParameters;
import frc.robot.utils.aiming.ShooterLookupTable;

/**
 * ShooterSubsystem
 *
 * Controls:
 * - Primary shooter flywheel (control TalonFX)
 * - Follower flywheel motor
 * - Kicker motor
 *
 * Supports:
 * - Manual RPM control
 * - Cached RPM toggling
 * - Auto-shoot command
 * - Distance-based auto range calculation (WIP)
 *
 * FRC 2026
 */
public class ShooterSubsystem extends SubsystemBase {

    // Primary shooter motor
    private final TalonFX control;

    // Secondary shooter motor (follows control)
    private final TalonFX follower;

    // References to other subsystems
    private HopperSubsystem HopperSubsystem;
    private KickerSubsystem KickerSubsystem;
    private HoodSubsystem HoodSubsystem;
    private DriveSubsystem DriveSubsystem;

    // Target speeds
    private double targetRPM = 2000;

    // Shooter limits and constants
    private static final double MAX_RPM = 5400.0;
    private static final double RPM_TO_RPS = 1.0 / 60.0; // CTRE uses rotations per second
    private static final double MIN_LAUNCH_READY_TIME_SECS = 0.15;
    private static final double MIN_COMMAND_RPM_FOR_FEED = 300.0;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

    private double shooterAdjustment = 0;

    // Auto-shoot state flags
    private boolean doAutoRange = true;
    private boolean autoShooting = false;

    // SOTM Helpers
    private final ShooterLookupTable shooterLookupTable;
    private final LaunchCalculator launchCalculator;

    private final Timer launchReadyTimer = new Timer();

    /**
     * ShooterSubsystem Constructor
     *
     * @param m_hopper     Hopper subsystem reference
     * @param m_kicker     Kicker subsystem reference
     * @param m_hood       Hood subsystem reference
     * @param m_drivetrain Drivetrain subsystem reference
     */
    public ShooterSubsystem(HopperSubsystem m_hopper, KickerSubsystem m_kicker, HoodSubsystem m_hood,
            DriveSubsystem m_drivetrain, LaunchCalculator launchCalculator) {
        this.HopperSubsystem = m_hopper;
        this.KickerSubsystem = m_kicker;
        this.HoodSubsystem = m_hood;
        this.DriveSubsystem = m_drivetrain;
        this.launchCalculator = launchCalculator;

        // CAN IDs
        control = Devices.shooterControl;
        follower = Devices.shooterFollower;

        TalonFXConfiguration shooterConfigs = Configs.getShooterConfigs();

        // Apply configurations
        control.getConfigurator().apply(shooterConfigs);
        follower.getConfigurator().apply(shooterConfigs);

        // Follower motor mirrors primary (opposed direction)
        follower.setControl(new Follower(control.getDeviceID(), MotorAlignmentValue.Opposed));

        shooterLookupTable = new ShooterLookupTable(Constants.SHOOTER_LOOKUP_TABLE);
    }

    /**
     * Sets target RPM for shooter motors (clamped to MAX_RPM).
     *
     * @param RPM Desired flywheel RPM
     */
    private void setTargetRPM(double RPM) {
        if (RPM > MAX_RPM)
            RPM = MAX_RPM;
        if (RPM < -MAX_RPM)
            RPM = -MAX_RPM;

        targetRPM = RPM;
    }

    /**
     * Runs shooter flywheel at specified RPM.
     *
     * @param targetRPM Desired RPM
     */
    private void rotate(double targetRPM) {
        control.setControl(velocityRequest.withVelocity(targetRPM * RPM_TO_RPS));
    }

    /**
     * Stops shooter flywheel.
     */
    private void stop() {
        control.stopMotor();
    }

    /**
     * Stops all shooter-related motors and hopper.
     */
    public void stopAll() {
        control.stopMotor();
        KickerSubsystem.stopKicker();
        HopperSubsystem.stopMotor();
        autoShooting = false;

        peripheralManualCommand.cancel();
    }

    /**
     * Prebuilt FunctionalCommand for auto shooting sequence.
     * Spins up shooter, waits for velocity tolerance, then feeds note.
     */
    private Command existingAutoShootCommand = new FunctionalCommand(
            () -> {
                rotate(getTargetRPM());
                KickerSubsystem.setKickerControl();
                launchReadyTimer.stop();
                launchReadyTimer.reset();
            },
            () -> {
                rotate(getTargetRPM());
                KickerSubsystem.setKickerControl();

                if (isLaunchReadyStable() || !doAutoRange) {
                    HopperSubsystem.spinForwards();
                } else {
                    HopperSubsystem.stopMotor();
                }
            },
            interrupted -> {
                stopAll();
                launchReadyTimer.stop();
                launchReadyTimer.reset();
            },
            () -> false,
            this);

    /**
     * Prebuilt FunctionalCommand for manual shooting sequence. Continuously runs
     * shooter and kicker at target RPM while held, with no auto-range or launch
     * readiness checks. Feeds note whenever command is active.
     */
    private Command peripheralManualCommand = new FunctionalCommand(
            () -> {
                rotate(getTargetRPM());
                KickerSubsystem.setKickerControl();
                launchReadyTimer.stop();
                launchReadyTimer.reset();
            },
            () -> {
                rotate(getTargetRPM());
                KickerSubsystem.setKickerControl();
                HopperSubsystem.spinForwards();
            },
            interrupted -> {
                stopAll();
                launchReadyTimer.stop();
                launchReadyTimer.reset();
            },
            () -> false,
            this);

    /**
     * Toggles auto shooting command scheduling.
     */
    public Command autoShoot() {
        System.out.println("AUTOSHOOTING: " + autoShooting);
        autoShooting = !autoShooting;

        return existingAutoShootCommand;
    }

    /**
     * Toggles auto shooting command scheduling.
     */
    public void manualShoot() {
        System.out.println("MANUAL");
        CommandScheduler.getInstance().schedule(peripheralManualCommand);
    }

    /**
     * Adjusts shooter RPM by delta amount.
     *
     * @param deltaRPM Change in RPM
     */
    public void changeTargetRPM(double deltaRPM) {
        setTargetRPM(targetRPM + deltaRPM);
    }

    /**
     * Prebuilt command to toggle shooter on/off at cached target RPM. If currently
     * running, stops the shooter; otherwise, starts it.
     * 
     * @return The prebuilt command.
     */
    public Command stopShooterCommand() {
        return new InstantCommand(() -> stop(), this);
    }

    /**
     * Toggles auto range mode.
     */
    private void toggleAutoRange() {
        doAutoRange = !doAutoRange;
        if (doAutoRange) {
        }

        if (doAutoRange == false) {
            autoShooting = false;
            existingAutoShootCommand.cancel();
        }
    }

    public Command toggleAutoRangeCommand() {
        return new InstantCommand(() -> toggleAutoRange());
    }

    /**
     * @return Shooter RPM (control motor)
     */
    public double getSpeedRPM() {
        return control.getRotorVelocity().getValueAsDouble() * 60.0;
    }

    /**
     * @return Follower motor RPM
     */
    public double getFollowerRPM() {
        return follower.getRotorVelocity().getValueAsDouble() * 60.0;
    }

    /**
     * @return Target shooter RPM
     */
    public double getTargetRPM() {
        return targetRPM;
    }

    /**
     * @return True if shooter motors are drawing current
     */
    public boolean isRunning() {
        return getShooterSupplyCurrent() > 0;
    }

    /**
     * @return True if autoShoot command is active
     */
    public boolean isAutoShooting() {
        return autoShooting;
    }

    /**
     * Returns whether auto-range mode is enabled. When enabled, periodic() will
     * calculate target RPM based on distance to target and command hood position
     * based on predefined settings; when disabled, these features are turned off
     * and the shooter can be controlled manually without interference from
     * auto-range logic.
     * 
     * @return True if auto-range mode is enabled
     */
    public boolean isAutoRangeEnabled() {
        return doAutoRange;
    }

    /**
     * Checks if both shooter motors are within 5% of target RPM.
     *
     * @return True if ready to fire
     */
    private boolean isVelocityWithinTolerance() {
        double tolerancePercent = 0.05;

        boolean controlReady = Math.abs(getTargetRPM() - getSpeedRPM()) <= getTargetRPM() * tolerancePercent;
        boolean followerReady = Math.abs(getTargetRPM() - getFollowerRPM()) <= getTargetRPM() * tolerancePercent;

        return controlReady && followerReady;
    }

    /**
     * @return Total supply current of shooter motors
     */
    public double getShooterSupplyCurrent() {
        return control.getSupplyCurrent().getValueAsDouble() +
                follower.getSupplyCurrent().getValueAsDouble();
    }

    double storedRPM;
    boolean isFirstCycleAuto = true;

    /**
     * Called once per scheduler run.
     * Handles auto-range RPM logic (WIP).
     */
    @Override
    public void periodic() {

        if (doAutoRange) {
            if (isFirstCycleAuto) {
                storedRPM = targetRPM;
                isFirstCycleAuto = false;
            }

            double distToHub = getDistToHub();

            double shooterRPM = 0;
            double kickerRPM = Constants.DEFAULT_KICKER_RPM;

            if (Constants.ENABLE_SHOOT_ON_MOVE) {
                LaunchingParameters parameters = launchCalculator.getParameters();
                shooterRPM = parameters.flywheelSpeed();
            } else {
                ShooterLookupTable.ShotSetpoint setpoint = shooterLookupTable.sample(distToHub);

                shooterRPM = setpoint.shooterRPM();
            }

            KickerSubsystem.setTargetRPMKicker(kickerRPM);

            double hoodCompensation = HoodSubsystem.isHoodDown() ? 0 : -1000;
            // Use the setter to ensure any configured RPM clamping is applied
            setTargetRPM(shooterRPM + shooterAdjustment + hoodCompensation);
        } else {
            isFirstCycleAuto = true;
        }
    }

    /**
     * Calculates straight-line distance from shooter to scoring target.
     *
     * @return Distance in meters
     */
    public double getDistToHub() {
        Pose2d robotPose = DriveSubsystem.getEstimatedPose();
        return AimingUtil.getShooterDistanceToTarget(robotPose);
    }

    /**
     * Checks all conditions for a valid launch: shooter RPM within tolerance, hood
     * in position, drive at goal, and valid launch parameters. Only relevant when
     * auto-range mode is enabled.
     * 
     * @return True if all conditions are met, false otherwise.
     */
    private boolean isLaunchReadyNow() {
        boolean shooterCommanded = Math.abs(getTargetRPM()) >= MIN_COMMAND_RPM_FOR_FEED;
        boolean hoodReady = true;
        boolean driveReady = !Constants.ENABLE_SHOOT_ON_MOVE || launchCalculator.atDriveGoal();
        boolean launchValid = !Constants.ENABLE_SHOOT_ON_MOVE || launchCalculator.getParameters().isValid();
        return shooterCommanded && isVelocityWithinTolerance() && hoodReady && driveReady && launchValid;
    }

    /**
     * Checks if launch conditions have been met for a minimum amount of time to
     * ensure stable feeding conditions, rather than just a brief momentary state.
     * Only
     * relevant when auto-range mode is enabled.
     * 
     * @return True if conditions are stable, false otherwise.
     */
    private boolean isLaunchReadyStable() {
        if (!isLaunchReadyNow()) {
            launchReadyTimer.stop();
            launchReadyTimer.reset();
            return false;
        }

        if (!launchReadyTimer.isRunning()) {
            launchReadyTimer.restart();
        }

        return launchReadyTimer.hasElapsed(MIN_LAUNCH_READY_TIME_SECS);
    }

    /**
     * Sets the shooter adjustment factor.
     *
     * @param adjustment The adjustment factor, in RPM.
     */
    private void setShooterAdjustment(double adjustment) {
        shooterAdjustment = adjustment;
    }

    /**
     * Adjusts shooter RPM adjustment factor by delta amount, which is added on top
     * of the target RPM.
     *
     * @param deltaRPM Change in RPM
     */
    private void changeShooterAdjustment(double deltaRPM) {
        setShooterAdjustment(getShooterAdjustment() + deltaRPM);
    }

    /**
     * Returns the current shooter adjustment factor, which is added on top of the
     * target RPM calculated by auto-range logic. This allows for fine-tuning
     * adjustments to shooter speed without affecting the underlying distance-based
     * calculations.
     * 
     * @return The shooter adjustment factor in RPM.
     */
    public double getShooterAdjustment() {
        return shooterAdjustment;
    }

    public Command changeShooterAdjustmentCommand(double deltaAdjustment) {
        return new InstantCommand(() -> changeShooterAdjustment(deltaAdjustment));
    }
}