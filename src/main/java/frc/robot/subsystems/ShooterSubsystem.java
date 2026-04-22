package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    // Feeds note into flywheel
    private final TalonFX kicker;

    // Adjusts shooting angle
    private final TalonFX hood;

    // Safety lock: when true, hood will never be commanded to move.
    private static final boolean AUTO_HOOD_DISABLED = true;

    // References to other subsystems
    private HopperSubsystem m_hopper;
    private DriveSubsystem m_drivetrain;

    // Reusable velocity control request to prevent object allocation in loops
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

    // Target speeds
    private double targetRPM = 2000;
    private double targetRPMKicker = Constants.DEFAULT_KICKER_RPM;

    // Shooter limits and constants
    private static final double MAX_RPM = 5400.0;
    private static final double RPM_TO_RPS = 1.0 / 60.0; // CTRE uses rotations per second
    private static final double HOOD_POSITION_TOLERANCE = 0.02;
    private static final double MIN_LAUNCH_READY_TIME_SECS = 0.15;
    private static final double MIN_COMMAND_RPM_FOR_FEED = 300.0;

    private double hoodTargetPos;
    private boolean hoodAtMax = false;
    private boolean hoodManualOverride = false;

    private double shooterAdjustment = 0;

    private final MotionMagicVoltage hoodMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

    // Auto-shoot state flags
    private boolean doAutoRange = true;
    private boolean autoShooting = false;
    private boolean manualShooting = false;
    private boolean hoodDown = true;

    // Polynomial shooter curve storage

    private final ShooterLookupTable shooterLookupTable;
    private final LaunchCalculator launchCalculator;

    private final Timer launchReadyTimer = new Timer();

    // Toggle state tracking
    private boolean isShooting = false;
    private boolean isKicking = false;

    private static final double comp_dist_offset = 0.0; // meters

    /**
     * ShooterSubsystem Constructor
     *
     * @param m_hopper     Hopper subsystem reference
     * @param m_drivetrain Drivetrain subsystem reference
     */
    public ShooterSubsystem(HopperSubsystem m_hopper, DriveSubsystem m_drivetrain, LaunchCalculator launchCalculator) {
        this.m_hopper = m_hopper;
        this.m_drivetrain = m_drivetrain;
        this.launchCalculator = launchCalculator;

        // CAN IDs
        kicker = Devices.kickerControl;
        hood = Devices.hoodControl;
        control = Devices.shooterControl;
        follower = Devices.shooterFollower;

        TalonFXConfiguration shooterConfigs = Configs.getShooterConfigs();

        // Apply configurations
        control.getConfigurator().apply(shooterConfigs);
        follower.getConfigurator().apply(shooterConfigs);

        // Follower motor mirrors primary (opposed direction)
        follower.setControl(new Follower(control.getDeviceID(), MotorAlignmentValue.Opposed));

        TalonFXConfiguration controlCfgKicker = Configs.getkickerConfigs();

        kicker.getConfigurator().apply(controlCfgKicker);

        TalonFXConfiguration hoodConfigs = Configs.getHoodConfigs();

        // Push one full config object so Slot0 + MotionMagic are guaranteed to match
        // this request.
        hood.getConfigurator().apply(hoodConfigs);

        // Assumes the hood starts at the mechanical minimum at startup.
        hood.setPosition(Constants.HOOD_MIN_SETPOINT);
        hoodTargetPos = Constants.HOOD_MIN_SETPOINT;
        hoodAtMax = false;

        zeroHood();
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
     * Sets target RPM for kicker motor (clamped).
     *
     * @param RPM Desired kicker RPM
     */
    private void setTargetRPMKicker(double RPM) {
        if (RPM > MAX_RPM)
            RPM = MAX_RPM;
        if (RPM < -MAX_RPM)
            RPM = -MAX_RPM;

        targetRPMKicker = RPM;
    }

    /**
     * Commands hood to move to target position using Motion Magic control.
     */
    private void updateHoodPos() {
        if (AUTO_HOOD_DISABLED) {
            hood.stopMotor();
            return;
        }
        hood.setControl(hoodMotionMagicRequest.withPosition(hoodTargetPos));
    }

    /**
     * Resets hood encoder position to zero (or mechanical minimum) for consistent
     * starting point.
     */
    private void zeroHood() {
        hood.setPosition(0.0);
        hoodTargetPos = 0.0; // or Constants.HOOD_MIN_SETPOINT
    }

    /**
     * Converts a hood position percentage (0.0 to 1.0) to a motor position in
     * rotations, based on defined min/max setpoints.
     * 
     * @param hoodPercent, the percent of the hood's range, where 0.0 is fully
     *                     retracted and 1.0 is fully extended.
     * @return The corresponding motor position in rotations for the hood.
     */
    private double hoodPercentToMotorPosition(double hoodPercent) {
        double clampedPercent = MathUtil.clamp(hoodPercent, 0.0, 1.0);
        return MathUtil.interpolate(Constants.HOOD_MIN_SETPOINT, Constants.HOOD_MAX_SETPOINT, clampedPercent);
    }

    /**
     * Converts a motor position in rotations to a hood position percentage (0.0 to
     * 1.0) based on defined min/max setpoints.
     * 
     * @param motorPosition, the current motor position in rotations for the hood.
     * @return The corresponding hood position percentage.
     */
    private double hoodMotorPositionToPercent(double motorPosition) {
        double percent = (motorPosition - Constants.HOOD_MIN_SETPOINT)
                / (Constants.HOOD_MAX_SETPOINT - Constants.HOOD_MIN_SETPOINT);
        return MathUtil.clamp(percent, 0.0, 1.0);
    }

    /**
     * Converts a hood position percentage to a release angle in radians, based on
     * defined min/max angles.
     * 
     * @param hoodPercent, the percent of the hood's range, where 0.0 is fully
     *                     retracted and 1.0 is fully extended.
     * @return The corresponding release angle in radians.
     */
    private double hoodPercentToReleaseAngleRadians(double hoodPercent) {
        double clampedPercent = MathUtil.clamp(hoodPercent, 0.0, 1.0);
        return MathUtil.interpolate(Constants.HOOD_ZERO_ANGLE, Constants.HOOD_LOWEST_ANGLE, clampedPercent);
    }

    /**
     * Public method to set hood position based on percentage input, with internal
     * conversion to motor rotations and Motion Magic control.
     * 
     * @param hoodPercent, the desired hood position as a percentage of its range
     *                     (0.0 to 1.0).
     */
    public void setHoodPositionPercent(double hoodPercent) {
        if (AUTO_HOOD_DISABLED) {
            hood.stopMotor();
            return;
        }
        hoodManualOverride = true;
        hoodTargetPos = hoodPercentToMotorPosition(hoodPercent);
        hoodAtMax = hoodPercent >= 0.5;
        System.out.printf("[HOOD] Manual percent request: %.3f -> target %.3f rot%n", hoodPercent, hoodTargetPos);
        updateHoodPos();
    }

    /**
     * Toggles hood between predefined minimum and maximum positions. If
     * AUTO_HOOD_DISABLED is true, this method will stop the hood motor instead of
     * moving it.
     */
    public void toggleHoodMinMax() {
        if (AUTO_HOOD_DISABLED) {
            hood.stopMotor();
            return;
        }

        hoodManualOverride = true;
        hoodAtMax = !hoodAtMax;
        hoodTargetPos = hoodAtMax ? Constants.HOOD_MAX_SETPOINT : Constants.HOOD_MIN_SETPOINT;
        System.out.printf("[HOOD] B toggle -> %s (target %.3f rot)%n", hoodAtMax ? "MAX" : "MIN", hoodTargetPos);
        updateHoodPos();
    }

    /**
     * Manually toggles hood between minimum and maximum positions, ignoring any
     * auto-range logic. This is intended for use with a dedicated manual override
     * button.
     * If AUTO_HOOD_DISABLED is true, this method will stop the hood motor instead
     * of moving it.
     */
    public void manualToggleHoodMinMax() {
        double newPosition = 0;

        if (hoodDown) {
            newPosition = Constants.HOOD_MAX_SETPOINT;
        } else {
            newPosition = Constants.HOOD_MIN_SETPOINT;
        }

        hoodDown = !hoodDown;

        hood.setControl(hoodMotionMagicRequest.withPosition(newPosition));
    }

    /**
     * Prebuilt command to toggle hood between min and max positions. If
     * AUTO_HOOD_DISABLED is true, this will return a command that stops the hood
     * motor instead.
     * 
     * @return The prebuilt command.
     */
    public Command toggleHoodMinMaxCommand() {
        return new InstantCommand(this::toggleHoodMinMax, this);
    }

    /**
     * Prebuilt command to manually toggle hood between min and max positions,
     * ignoring auto-range logic. If AUTO_HOOD_DISABLED is true, this will return a
     * command that stops the hood motor instead.
     * 
     * @return The prebuilt command.
     */
    public Command manualToggleHoodMinMaxCommand() {
        return new InstantCommand(this::manualToggleHoodMinMax, this);
    }

    /**
     * Applies closed-loop velocity control to kicker motor.
     */
    private void setKickerControl() {
        kicker.setControl(velocityRequest.withVelocity(targetRPMKicker * RPM_TO_RPS));
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
     * Toggles shooter on/off using cached targetRPM.
     */
    private void rotateAtCached() {
        if (isShooting) {
            isShooting = false;
            stop();
        } else {
            isShooting = true;
            rotate(targetRPM);
        }
    }

    /**
     * Toggles kicker motor.
     */
    private void rotateKicker() {
        if (isKicking) {
            kicker.stopMotor();
            isKicking = false;
        } else {
            setKickerControl();
            isKicking = true;
        }
    }

    /**
     * Stops shooter flywheel.
     */
    private void stop() {
        control.stopMotor();
    }

    /**
     * Stops kicker motor.
     */
    private void stopKicker() {
        kicker.stopMotor();
    }

    /**
     * Stops all shooter-related motors and hopper.
     */
    public void stopAll() {
        control.stopMotor();
        kicker.stopMotor();
        m_hopper.stopMotor();
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
                setKickerControl();
                launchReadyTimer.stop();
                launchReadyTimer.reset();
            },
            () -> {
                rotate(getTargetRPM());
                setKickerControl();

                if (isLaunchReadyStable() || !doAutoRange) {
                    m_hopper.spinForwards();
                } else {
                    m_hopper.stopMotor();
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
                setKickerControl();
                launchReadyTimer.stop();
                launchReadyTimer.reset();
            },
            () -> {
                rotate(getTargetRPM());
                setKickerControl();
                m_hopper.spinForwards();
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
    public void autoShoot() {
        System.out.println("AUTOSHOOTING: " + autoShooting);
        if (autoShooting) {
            autoShooting = false;
            existingAutoShootCommand.cancel();
        } else {
            autoShooting = true;
            CommandScheduler.getInstance().schedule(existingAutoShootCommand);
        }
    }

    /**
     * Toggles auto shooting command scheduling.
     */
    public void manualShoot() {
        System.out.println("MANUAL");
        CommandScheduler.getInstance().schedule(peripheralManualCommand);
    }

    public Command hoodPosCommand(double pos) {
        return new InstantCommand(() -> setHoodPositionPercent(pos), this);
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
     * Adjusts shooter RPM adjustment factor by delta amount, which is added on top
     * of the target RPM.
     *
     * @param deltaRPM Change in RPM
     */
    public void changeShooterAdjustment(double deltaRPM) {
        setShooterAdjustment(getShooterAdjustment() + deltaRPM);
    }

    /**
     * Adjusts kicker RPM by delta amount.
     *
     * @param deltaRPM Change in RPM
     */
    public void changeKickerTargetRPM(double deltaRPM) {
        setTargetRPMKicker(getTargetRPMKicker() + deltaRPM);
    }

    /**
     * Prebuilt command to toggle kicker on/off at cached target RPM.
     * 
     * @return The prebuilt command.
     */
    public Command setKickerControlCommand() {
        return new InstantCommand(() -> setKickerControl(), this);
    }

    /**
     * Prebuilt command to toggle kicker on/off using cached target RPM. If
     * currently
     * running, stops the kicker; otherwise, starts it.
     * 
     * @return The prebuilt command.
     */
    public Command rotateKickerCommand() {
        return new InstantCommand(() -> rotateKicker(), this);
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
     * Prebuilt command to stop kicker motor.
     * 
     * @return The prebuilt command.
     */
    public Command stopKickerCommand() {
        return new InstantCommand(() -> stopKicker(), this);
    }

    /**
     * Toggles auto range mode.
     */
    public void toggleAutoRange() {
        doAutoRange = !doAutoRange;
        if (doAutoRange) {
            hoodManualOverride = false;
            System.out.println("[HOOD] Auto-range enabled; releasing manual hood override.");
        }

        if (doAutoRange == false) {
            autoShooting = false;
            existingAutoShootCommand.cancel();
        }
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
     * @return Kicker motor RPM
     */
    public double getSpeedRPMKicker() {
        return kicker.getRotorVelocity().getValueAsDouble() * 60.0;
    }

    /**
     * @return Target shooter RPM
     */
    public double getTargetRPM() {
        return targetRPM;
    }

    /**
     * @return Target kicker RPM
     */
    public double getTargetRPMKicker() {
        return targetRPMKicker;
    }

    /**
     * @return The hood's position
     */
    public double getHoodPosition() {
        return hood.getRotorPosition().getValueAsDouble();
    }

    /**
     * @return True if shooter toggle is active
     */
    public boolean isShooting() {
        return isShooting;
    }

    /**
     * @return True if kicker toggle is active
     */
    public boolean isKicking() {
        return isKicking;
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
     * @return True if hood is in manual override mode (i.e. auto-range logic will
     *         not command hood movement)
     */
    public boolean isHoodManualOverride() {
        return hoodManualOverride;
    }

    /**
     * @return True if auto hood control is disabled
     */
    public boolean isAutoHoodDisabled() {
        return AUTO_HOOD_DISABLED;
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
     * Returns the target position for the hood.
     * 
     * @return The target hood position
     */
    public double getHoodTargetPosition() {
        return hoodTargetPos;
    }

    /**
     * Returns whether the hood is currently toggled down (i.e. at minimum
     * position). This is used for auto-range compensation, where the shooter RPM is
     * adjusted based on whether the hood is up or down to account for changes in
     * release angle.
     * 
     * @return True if the hood is down, false if it is up
     */
    public boolean isHoodDown() {
        return hoodDown;
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
     * Checks if hood is within position tolerance of target. Only relevant when
     * AUTO_HOOD_DISABLED is false; if AUTO_HOOD_DISABLED is true, this will return
     * true regardless.
     * 
     * @return True if hood is within tolerance or auto hood control is disabled,
     *         false otherwise.
     */
    private boolean isHoodWithinTolerance() {
        return Math.abs(getHoodPosition() - hoodTargetPos) <= HOOD_POSITION_TOLERANCE;
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
        boolean hoodReady = AUTO_HOOD_DISABLED || isHoodWithinTolerance();
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
     * @return Total supply current of shooter motors
     */
    public double getShooterSupplyCurrent() {
        return control.getSupplyCurrent().getValueAsDouble() +
                follower.getSupplyCurrent().getValueAsDouble();
    }

    /**
     * Sets the shooter adjustment factor.
     *
     * @param adjustment The adjustment factor, in RPM.
     */
    public void setShooterAdjustment(double adjustment) {
        shooterAdjustment = adjustment;
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

    double storedRPM;
    boolean isFirstCycleAuto = true;

    /**
     * Called once per scheduler run.
     * Handles auto-range RPM logic (WIP).
     */
    @Override
    public void periodic() {
        // if (HOOD_DISABLED) {
        // hood.stopMotor();
        // }

        if (doAutoRange) {
            if (isFirstCycleAuto) {
                storedRPM = targetRPM;
                isFirstCycleAuto = false;
            }

            double distToHub = getDistToHub() + comp_dist_offset;

            double shooterRPM = 0;
            double kickerRPM = Constants.DEFAULT_KICKER_RPM;
            boolean passingMode = false;

            if (Constants.ENABLE_SHOOT_ON_MOVE) {
                LaunchingParameters parameters = launchCalculator.getParameters();
                shooterRPM = parameters.flywheelSpeed();
                passingMode = parameters.passing();
            } else {
                ShooterLookupTable.ShotSetpoint setpoint = shooterLookupTable.sample(distToHub);

                shooterRPM = setpoint.shooterRPM();

                Pose2d robotPose = m_drivetrain.getEstimatedPose();
                passingMode = AimingUtil.getTargetTranslation(robotPose)
                        .getDistance(AimingUtil.getHubTargetTranslation()) > 1e-4;
            }

            // if (!HOOD_DISABLED && !hoodManualOverride) {
            // double fixedHoodPercent = passingMode
            // ? Constants.PASSING_FIXED_HOOD_PERCENT
            // : Constants.HUB_SIDE_FIXED_HOOD_PERCENT;
            // hoodTargetPos = hoodPercentToMotorPosition(fixedHoodPercent);
            // hoodAtMax = fixedHoodPercent >= 0.5;
            // updateHoodPos();
            // }

            double hoodCompensation = isHoodDown() ? 0 : -1000;
            targetRPM = shooterRPM + shooterAdjustment + hoodCompensation;
            targetRPMKicker = kickerRPM;

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
        Pose2d robotPose = m_drivetrain.getEstimatedPose();
        return AimingUtil.getShooterDistanceToTarget(robotPose);
    }

    /**
     * Evaluates polynomial curve at given x.
     *
     * @param xPoint Input value
     * @param coeffs Polynomial coefficients
     * @return Computed y value
     */
    private double getValueFromCurve(double xPoint, double[] coeffs) {
        double total = 0;
        double power = 0;

        for (double d : coeffs) {
            total += d * Math.pow(xPoint, power);
            power++;
        }

        return total;
    }
}