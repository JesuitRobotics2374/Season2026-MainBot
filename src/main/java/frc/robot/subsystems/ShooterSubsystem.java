package frc.robot.subsystems;

import org.apache.commons.math4.legacy.fitting.PolynomialCurveFitter;
import org.apache.commons.math4.legacy.fitting.WeightedObservedPoints;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.utils.Ballistics;

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

    // References to other subsystems
    private HopperSubsystem m_hopper;
    private DriveSubsystem m_drivetrain;

    // Reusable velocity control request to prevent object allocation in loops
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

    // Target speeds
    private double targetRPM = 2000;
    private double targetRPMKicker = 2000;

    // Shooter limits and constants
    private static final double MAX_RPM = 5400.0;
    private static final double RPM_TO_RPS = 1.0 / 60.0; // CTRE uses rotations per second
    private static final double CURRENT_LIMIT = 60.0; // Amps
    private static final double KICKER_CURRENT_LIMIT = 60; // Amps

    private double hoodTargetPos;

    // Auto-shoot state flags
    private boolean doAutoShoot = false;
    private boolean autoShooting = false;
    private boolean isRed;
    private double MIN_RANGE = 0.5; // meters

    // Polynomial shooter curve storage
    private double[][] shooterValues = { {} };
    private double[] shooterCoeffs = {};

    // Toggle state tracking
    private boolean isShooting = false;
    private boolean isKicking = false;

    /**
     * ShooterSubsystem Constructor
     *
     * @param m_hopper     Hopper subsystem reference
     * @param isRed        Alliance color flag
     * @param m_drivetrain Drivetrain subsystem reference
     */
    public ShooterSubsystem(HopperSubsystem m_hopper, boolean isRed, DriveSubsystem m_drivetrain) {

        this.m_hopper = m_hopper;
        this.m_drivetrain = m_drivetrain;

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
        controlCfg.Slot0.kP = 0.09;
        controlCfg.Slot0.kI = 0;
        controlCfg.Slot0.kD = 0.001;
        controlCfg.Slot0.kV = 0.12; // ~12V feedforward

        // Kicker configuration
        TalonFXConfiguration controlCfgKicker = new TalonFXConfiguration();
        controlCfgKicker.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        controlCfgKicker.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        controlCfgKicker.CurrentLimits.SupplyCurrentLimitEnable = true;
        controlCfgKicker.CurrentLimits.SupplyCurrentLimit = KICKER_CURRENT_LIMIT;
        controlCfgKicker.CurrentLimits.StatorCurrentLimitEnable = true;
        controlCfgKicker.CurrentLimits.StatorCurrentLimit = KICKER_CURRENT_LIMIT / 0.5;

        controlCfgKicker.Slot0.kP = 0.09;
        controlCfgKicker.Slot0.kI = 0;
        controlCfgKicker.Slot0.kD = 0.001;
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

        //setZeroHood();

        this.isRed = isRed;
    }

    /**
     * Fits a 3rd degree polynomial to shooterValues dataset.
     * Used for velocity-based RPM interpolation.
     */
    private void calculateShooterCurves() {

        WeightedObservedPoints points = new WeightedObservedPoints();

        for (int i = 0; i < shooterValues.length; i++) {
            double[] values = shooterValues[i];
            points.add(values[0], values[1]);
        }

        PolynomialCurveFitter fitter = PolynomialCurveFitter.create(3);
        shooterCoeffs = fitter.fit(points.toList());
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
        MotionMagicVoltage m_request = new MotionMagicVoltage(hoodTargetPos);

        hood.setControl(m_request);
    }

    private void setHoodPos(double pos) {
        hoodTargetPos = pos;

        updateHoodPos();
    }

    private void hoodChangeBy(double deltaPos) {
        hoodTargetPos += deltaPos;
        updateHoodPos();
    }

    private void setZeroHood() {
        hood.setPosition(0.0);
        hoodTargetPos = 0;

        updateHoodPos();
    }

    /**
     * Adjusts shooter RPM by delta amount.
     *
     * @param deltaRPM Change in RPM
     */
    private void changeTargetRPM(double deltaRPM) {
        setTargetRPM(targetRPM + deltaRPM);
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
    private void stopAll() {
        control.stopMotor();
        kicker.stopMotor();
        m_hopper.stopMotor();
    }

    /**
     * Prebuilt FunctionalCommand for auto shooting sequence.
     * Spins up shooter, waits for velocity tolerance, then feeds note.
     */
    private Command existingAutoShootCommand = new FunctionalCommand(
            () -> {
                rotate(getTargetRPM());
                setKickerControl();
            },
            () -> {
                rotate(getTargetRPM());
                setKickerControl();
                if (isVelocityWithinTolerance()) {
                    m_hopper.spinForwards();
                }
            },
            interrupted -> {
                stopAll();
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
        return new InstantCommand(() -> hoodPosCommand(pos), this);
    }
    
    public Command hoodChangeCommand(double deltaPos) {
        return new InstantCommand(() -> hoodChangeBy(deltaPos), this);
    }

    public Command zeroHoodCommand() {
        return new InstantCommand(() -> setZeroHood(), this);
    }

    public Command changeTargetRPMCommand(double deltaRPM) {
        return new InstantCommand(() -> changeTargetRPM(deltaRPM), this);
    }

    public Command changeKickerSpeedCommand(double deltaRPM) {
        return new InstantCommand(() -> setTargetRPMKicker(getTargetRPMKicker() + deltaRPM), this);
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
        doAutoShoot = !doAutoShoot;
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
        if (doAutoShoot) {
            if (isFirstCycleAuto) {
                storedRPM = targetRPM;
                isFirstCycleAuto = false;
            }

            Translation2d absoluteTargetTranslation = getAbsoluteTranslation(isRed);

            double delta_x = absoluteTargetTranslation.getX() - m_drivetrain.getRobotX();
            double delta_y = absoluteTargetTranslation.getY() - m_drivetrain.getRobotY();

            double hyp = Math.sqrt(delta_x * delta_x + delta_y * delta_y);

            if (hyp < MIN_RANGE) {
                return;
            }

            double minRange = 1.5;
            double maxRange = 7;
            double step = 0.5;
            double val = 0;

            for (double i = minRange; i < maxRange; i += step) {
                val++;
                if (Math.abs(i - hyp) <= 0.25) {
                    break;
                }
            }

        } else {
            isFirstCycleAuto = true;
        }
    }

    /**
     * Calculates straight-line distance from shooter to scoring target.
     *
     * @return Distance in meters
     */
    public double getDistToHubFromShooter() {
        Translation2d absoluteTargetTranslation = getAbsoluteTranslation(isRed);

        double delta_x = absoluteTargetTranslation.getX() - m_drivetrain.getRobotX();
        double delta_y = absoluteTargetTranslation.getY() - m_drivetrain.getRobotY();

        double hyp = Math.sqrt(delta_x * delta_x + delta_y * delta_y);

        return hyp;
    }

    /**
     * Returns field position of scoring target based on alliance.
     *
     * @param isRed Alliance color
     * @return Absolute field translation
     */
    private Translation2d getAbsoluteTranslation(boolean isRed) {
        if (isRed) {
            return new Translation2d(11.915394, 4.034536);
        } else {
            return new Translation2d(4.625594, 4.034536);
        }
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
