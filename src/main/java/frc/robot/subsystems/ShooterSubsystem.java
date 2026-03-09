package frc.robot.subsystems;

import org.apache.commons.math4.legacy.fitting.PolynomialCurveFitter;
import org.apache.commons.math4.legacy.fitting.WeightedObservedPoints;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.utils.Ballistics;
import frc.robot.utils.Constants;
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
    private static final boolean HOOD_DISABLED = true;

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
    private static final double CURRENT_LIMIT = 60.0; // Amps
    private static final double KICKER_CURRENT_LIMIT = 60; // Amps
    private static final double HOOD_POSITION_TOLERANCE = 0.02;
    private static final double MIN_LAUNCH_READY_TIME_SECS = 0.15;
    private static final double MIN_COMMAND_RPM_FOR_FEED = 300.0;

    private double hoodTargetPos;

    private final MotionMagicVoltage hoodMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

    // Auto-shoot state flags
    private boolean doAutoRange = true;
    private boolean autoShooting = false;
    private final boolean useTable = true;
    private final boolean useVelBased = false;

    // Polynomial shooter curve storage

        // Distance (from shooter), RPM Shooter, Hood Percent
        private final double[][] shooterValues = {
            { 1.51, 2000, 0 },
            { 2.00, 2300, 0 },
            { 2.52, 2600, 0 },
            { 3.00, 2800, 0 },
            { 3.52, 3000, 0 },
            { 4.00, 3200, 0 } };
    private double[] shooterCoeffs = {};
    private double[] velCoeffs = {};
    private final ShooterLookupTable shooterLookupTable;
    private final LaunchCalculator launchCalculator;

    private final Timer launchReadyTimer = new Timer();

    // Toggle state tracking
    private boolean isShooting = false;
    private boolean isKicking = false;

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
        kicker = new TalonFX(33);
        hood = new TalonFX(34);
        control = new TalonFX(35);
        follower = new TalonFX(36);

        // Shooter motor configuration
        TalonFXConfiguration controlCfg = new TalonFXConfiguration();
        controlCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        controlCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Current limiting
        controlCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        controlCfg.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        controlCfg.CurrentLimits.StatorCurrentLimitEnable = true;
        controlCfg.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT / 0.75;

        // PID + Feedforward tuning
        controlCfg.Slot0.kP = 0.18;
        controlCfg.Slot0.kI = 0.001;
        controlCfg.Slot0.kD = 0.002;
        controlCfg.Slot0.kV = 0.12; // ~12V feedforward

        // Kicker configuration
        TalonFXConfiguration controlCfgKicker = new TalonFXConfiguration();
        controlCfgKicker.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        controlCfgKicker.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        controlCfgKicker.CurrentLimits.SupplyCurrentLimitEnable = true;
        controlCfgKicker.CurrentLimits.SupplyCurrentLimit = KICKER_CURRENT_LIMIT;
        controlCfgKicker.CurrentLimits.StatorCurrentLimitEnable = true;
        controlCfgKicker.CurrentLimits.StatorCurrentLimit = KICKER_CURRENT_LIMIT / 0.5;

        controlCfgKicker.Slot0.kP = 0.18;
        controlCfgKicker.Slot0.kI = 0.001;
        controlCfgKicker.Slot0.kD = 0.002;
        controlCfgKicker.Slot0.kV = 0.12;

        // Apply configurations
        control.getConfigurator().apply(controlCfg);
        follower.getConfigurator().apply(controlCfg);

        // Follower motor mirrors primary (opposed direction)
        follower.setControl(new Follower(control.getDeviceID(), MotorAlignmentValue.Opposed));

        kicker.getConfigurator().apply(controlCfgKicker);

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;

        slot0Configs.kG = 0.001; // Output of voltage to overcome gravity
        slot0Configs.kV = 0; // Output per unit target velocity, perhaps not needed
        slot0Configs.kA = 0.0; // Output per unit target acceleration, perhaps not needed
        slot0Configs.kP = 0.01; // Controls the response to position error—how much the motor reacts to the
                                // difference between the current position and the target position.
        slot0Configs.kI = 0.01; // Addresses steady-state error, which occurs when the motor doesn’t quite reach
        // the target position due to forces like gravity or friction.
        slot0Configs.kD = 0.01; // Responds to the rate of change of the error, damping the motion as the motor
                                // approaches the target. This reduces overshooting and oscillations.

        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motionMagicConfigs.MotionMagicCruiseVelocity = 1; // Target velocity in rps
        motionMagicConfigs.MotionMagicAcceleration = 5; // Target acceleration in rps/s
        motionMagicConfigs.MotionMagicJerk = 50; // Target jerk in rps/s/s

        hood.getConfigurator().apply(talonFXConfigs);
        hood.getConfigurator().apply(slot0Configs);
        hood.getConfigurator().apply(motionMagicConfigs);

        if (!useTable) {
            calculateShooterCurves();
        }

        shooterLookupTable = new ShooterLookupTable(shooterValues);

    }

    /**
     * Fits a 3rd degree polynomial to shooterValues dataset.
     * Used for velocity-based RPM interpolation.
     */
    private void calculateShooterCurves() {

        WeightedObservedPoints shooterPoints = new WeightedObservedPoints();
        WeightedObservedPoints velPoints = new WeightedObservedPoints();

        for (int i = 0; i < shooterValues.length; i++) {
            double[] values = shooterValues[i];
            shooterPoints.add(values[0], values[1]);
            velPoints.add(Ballistics.CalculateNeededShooterSpeed(values[0], 0, 0, Constants.HOOD_ZERO_ANGLE),
                    values[1]);
        }

        PolynomialCurveFitter fitterShooter = PolynomialCurveFitter.create(3);
        shooterCoeffs = fitterShooter.fit(shooterPoints.toList());

        PolynomialCurveFitter fitterVel = PolynomialCurveFitter.create(3);
        velCoeffs = fitterVel.fit(velPoints.toList());
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

    private void updateHoodPos() {
        if (HOOD_DISABLED) {
            hood.stopMotor();
            return;
        }
        hood.setControl(hoodMotionMagicRequest.withPosition(hoodTargetPos));
    }

    private double hoodPercentToMotorPosition(double hoodPercent) {
        double clampedPercent = MathUtil.clamp(hoodPercent, 0.0, 1.0);
        return MathUtil.interpolate(Constants.HOOD_MIN_SETPOINT, Constants.HOOD_MAX_SETPOINT, clampedPercent);
    }

    private double hoodMotorPositionToPercent(double motorPosition) {
        double percent = (motorPosition - Constants.HOOD_MIN_SETPOINT)
                / (Constants.HOOD_MAX_SETPOINT - Constants.HOOD_MIN_SETPOINT);
        return MathUtil.clamp(percent, 0.0, 1.0);
    }

    private double hoodPercentToReleaseAngleRadians(double hoodPercent) {
        double clampedPercent = MathUtil.clamp(hoodPercent, 0.0, 1.0);
        return MathUtil.interpolate(Constants.HOOD_ZERO_ANGLE, Constants.HOOD_LOWEST_ANGLE, clampedPercent);
    }

    private double getCurrentHoodReleaseAngleRadians() {
        return hoodPercentToReleaseAngleRadians(hoodMotorPositionToPercent(getHoodPosition()));
    }

    public void setHoodPositionPercent(double hoodPercent) {
        if (HOOD_DISABLED) {
            hood.stopMotor();
            return;
        }
        hoodTargetPos = hoodPercentToMotorPosition(hoodPercent);
        updateHoodPos();
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

                if (isLaunchReadyStable()) {
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

    public void changeKickerTargetRPM(double deltaRPM) {
        setTargetRPMKicker(getTargetRPMKicker() + deltaRPM);
    }

    public Command setKickerControlCommand() {
        return new InstantCommand(() -> setKickerControl(), this);
    }

    public Command rotateKickerCommand() {
        return new InstantCommand(() -> rotateKicker(), this);
    }

    public Command stopShooterCommand() {
        return new InstantCommand(() -> stop(), this);
    }

    public Command stopKickerCommand() {
        return new InstantCommand(() -> stopKicker(), this);
    }

    /**
     * Toggles auto range mode.
     */
    public void toggleAutoRange() {
        doAutoRange = !doAutoRange;

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

    private boolean isHoodWithinTolerance() {
        return Math.abs(getHoodPosition() - hoodTargetPos) <= HOOD_POSITION_TOLERANCE;
    }

    private boolean isLaunchReadyNow() {
        boolean shooterCommanded = Math.abs(getTargetRPM()) >= MIN_COMMAND_RPM_FOR_FEED;
        boolean hoodReady = HOOD_DISABLED || isHoodWithinTolerance();
        boolean driveReady = !Constants.ENABLE_SHOOT_ON_MOVE || launchCalculator.atDriveGoal();
        boolean launchValid = !Constants.ENABLE_SHOOT_ON_MOVE || launchCalculator.getParameters().isValid();
        return shooterCommanded && isVelocityWithinTolerance() && hoodReady && driveReady && launchValid;
    }

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

    double storedRPM;
    boolean isFirstCycleAuto = true;

    /**
     * Called once per scheduler run.
     * Handles auto-range RPM logic (WIP).
     */
    @Override
    public void periodic() {
        if (HOOD_DISABLED) {
            hood.stopMotor();
        }

        if (doAutoRange) {
            if (isFirstCycleAuto) {
                storedRPM = targetRPM;
                isFirstCycleAuto = false;
            }

            double distToHub = getDistToHub();

            double shooterRPM = 0;
            double kickerRPM = Constants.DEFAULT_KICKER_RPM;

            ChassisSpeeds robotSpeeds = m_drivetrain.getCurrentRobotChassisSpeeds();

            if (useTable) {
                if (Constants.ENABLE_SHOOT_ON_MOVE) {
                    LaunchingParameters parameters = launchCalculator.getParameters();
                    shooterRPM = parameters.flywheelSpeed();
                    if (!HOOD_DISABLED) {
                        double hoodPercent = (parameters.hoodAngle() - Constants.HOOD_ZERO_ANGLE)
                                / (Constants.HOOD_LOWEST_ANGLE - Constants.HOOD_ZERO_ANGLE);
                        setHoodPositionPercent(hoodPercent);
                    }
                } else {
                    ShooterLookupTable.ShotSetpoint setpoint = shooterLookupTable.sample(distToHub);

                    shooterRPM = setpoint.shooterRPM();

                    if (!HOOD_DISABLED) {
                        setHoodPositionPercent(setpoint.hoodPercent());
                    }
                }
            } else {
                if (useVelBased) {
                    shooterRPM = getValueFromCurve(
                            Ballistics.CalculateNeededShooterSpeed(distToHub, robotSpeeds.vxMetersPerSecond,
                                    robotSpeeds.vyMetersPerSecond, getCurrentHoodReleaseAngleRadians()),
                            velCoeffs);
                } else {
                    shooterRPM = getValueFromCurve(distToHub, shooterCoeffs);
                }
            }

            targetRPM = shooterRPM;
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
        Pose2d robotPose = m_drivetrain.getEstimator();
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
