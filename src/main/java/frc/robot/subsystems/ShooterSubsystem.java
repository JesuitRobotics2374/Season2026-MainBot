package frc.robot.subsystems;

import org.apache.commons.math4.legacy.fitting.PolynomialCurveFitter;
import org.apache.commons.math4.legacy.fitting.WeightedObservedPoints;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

    // Target speeds
    private double targetRPM = 0.0;
    private double targetRPMKicker = 2000;

    // Shooter limits and constants
    private static final double MAX_RPM = 6000.0;
    private static final double MIN_RPM = 5;
    private static final double RPM_TO_RPS = 1.0 / 60.0; // CTRE uses rotations per second
    private static final double CURRENT_LIMIT = 60.0; // Amps
    private static final double KICKER_CURRENT_LIMIT = 60; // Amps

    private static final double HOOD_SPEED = 0.1; // Rotations per second, TODO: INCREASE
    private static final double MIN_HOOD = 0.0; // Rotations
    private static final double MAX_HOOD = 0.5; // Rotations

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
        control = new TalonFX(34);
        follower = new TalonFX(35);
        kicker = new TalonFX(36);
        hood = new TalonFX(37);

        // Shooter motor configuration
        TalonFXConfiguration controlCfg = new TalonFXConfiguration();
        controlCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        controlCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

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
        controlCfgKicker.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        controlCfgKicker.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

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
        follower.setControl(new Follower(34, MotorAlignmentValue.Opposed));

        kicker.getConfigurator().apply(controlCfgKicker);

        hood.setNeutralMode(NeutralModeValue.Brake);
        hood.setPosition(0);

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

    /**
     * Sets the hood to a given speed.
     * 
     * @param speed The speed to set the hood to
     */
    private void setHoodSpeed(double speed) {
        hood.set(speed);
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
     * Applies closed-loop velocity control to kicker motor.
     */
    public void setKickerControl() {
        kicker.setControl(velocityRequest.withVelocity(targetRPMKicker * RPM_TO_RPS));
    }

    /**
     * Runs shooter flywheel at specified RPM.
     *
     * @param targetRPM Desired RPM
     */
    public void rotate(double targetRPM) {
        control.setControl(velocityRequest.withVelocity(targetRPM * RPM_TO_RPS));
    }

    /**
     * Toggles shooter on/off using cached targetRPM.
     */
    public void rotateAtCached() {
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
    public void rotateKicker() {
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
    public void stop() {
        control.stopMotor();
    }

    /**
     * Stops kicker motor.
     */
    public void stopKicker() {
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
            },
            () -> {
                rotate(getTargetRPM());
                if (isVelocityWithinTolerance()) {
                    setKickerControl();
                    m_hopper.roll();
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

    /**
     * Sets the hood to a given position
     */
    public Command moveHood(double rotations) {
        double pos = 0;
        if (rotations < MIN_HOOD) {
            pos = MIN_HOOD;
        }
        if (rotations > MAX_HOOD) {
            pos = MAX_HOOD;
        }

        rotations = pos;
        
        final double speed; //IDK IF THIS BEING FINAL WILL MESS IT UP

        if (getHoodPosition() - rotations > 0) {
            speed = HOOD_SPEED;
        }
        else if (getHoodPosition() == rotations) {
            speed = 0;
        }
        else {
            speed = -HOOD_SPEED;
        }

        return new FunctionalCommand(() -> {
            setHoodSpeed(speed);
        },
                () -> {
                    setHoodSpeed(speed);
                },
                interrupted -> {
                    setHoodSpeed(0);
                },
                () -> Math.abs(getHoodPosition() - rotations) < 0.01,
                this);
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
