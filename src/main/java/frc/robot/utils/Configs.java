package frc.robot.utils;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Configs {
    public static TalonFXConfiguration getIntakeControlConfigs() {
        TalonFXConfiguration controlCfg = new TalonFXConfiguration();

        controlCfg.Slot0.kP = 0.2;
        controlCfg.Slot0.kI = 0.001;
        controlCfg.Slot0.kD = 0.01;
        controlCfg.Slot0.kV = 0.12;
        controlCfg.Slot0.kS = 0.01;

        controlCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        controlCfg.CurrentLimits.SupplyCurrentLimit = Constants.INTAKE_CURRENT_LIMIT;
        controlCfg.CurrentLimits.StatorCurrentLimitEnable = true;
        controlCfg.CurrentLimits.StatorCurrentLimit = Constants.INTAKE_CURRENT_LIMIT / 0.75;

        return controlCfg;
    }

    public static TalonFXConfiguration getIntakePivotConfigs() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;

        slot0Configs.kG = 0.1; // Output of voltage to overcome gravity
        slot0Configs.kV = 2; // Output per unit target velocity, perhaps not needed
        slot0Configs.kA = 0.3; // Output per unit target acceleration, perhaps not needed
        slot0Configs.kP = 15; // Controls the response to position error—how much the motor reacts to the
                              // difference between the current position and the target position.
        slot0Configs.kI = 0.01; // Addresses steady-state error, which occurs when the motor doesn’t quite reach
        // the target position due to forces like gravity or friction.
        slot0Configs.kD = 0.1; // Responds to the rate of change of the error, damping the motion as the motor
                               // approaches the target. This reduces overshooting and oscillations.

        talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motionMagicConfigs.MotionMagicCruiseVelocity = 50; // Target velocity in rps
        motionMagicConfigs.MotionMagicAcceleration = 70; // Target acceleration in rps/s
        motionMagicConfigs.MotionMagicJerk = 100; // Target jerk in rps/s/s

        return talonFXConfigs;
    }

    public static TalonFXConfiguration getHopperConfigs() {
        TalonFXConfiguration controlCfg = new TalonFXConfiguration();
        controlCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        controlCfg.Slot0.kP = 0.18;
        controlCfg.Slot0.kI = 0.001;
        controlCfg.Slot0.kD = 0.002;
        controlCfg.Slot0.kV = 0.12;

        return controlCfg;
    }

    public static TalonFXConfiguration getkickerConfigs() {
        TalonFXConfiguration controlCfgKicker = new TalonFXConfiguration();
        controlCfgKicker.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        controlCfgKicker.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        controlCfgKicker.CurrentLimits.SupplyCurrentLimitEnable = true;
        controlCfgKicker.CurrentLimits.SupplyCurrentLimit = Constants.KICKER_CURRENT_LIMIT;
        controlCfgKicker.CurrentLimits.StatorCurrentLimitEnable = true;
        controlCfgKicker.CurrentLimits.StatorCurrentLimit = Constants.KICKER_CURRENT_LIMIT / 0.5;

        controlCfgKicker.Slot0.kP = 0.18;
        controlCfgKicker.Slot0.kI = 0.001;
        controlCfgKicker.Slot0.kD = 0.002;
        controlCfgKicker.Slot0.kV = 0.12;

        return controlCfgKicker;
    }

    public static TalonFXConfiguration getShooterConfigs() {
        TalonFXConfiguration controlCfg = new TalonFXConfiguration();
        controlCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        controlCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Current limiting
        controlCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        controlCfg.CurrentLimits.SupplyCurrentLimit = Constants.SHOOTER_CURRENT_LIMIT;
        controlCfg.CurrentLimits.StatorCurrentLimitEnable = true;
        controlCfg.CurrentLimits.StatorCurrentLimit = Constants.SHOOTER_CURRENT_LIMIT / 0.75;

        // PID + Feedforward tuning
        controlCfg.Slot0.kP = 0.18;
        controlCfg.Slot0.kI = 0.001;
        controlCfg.Slot0.kD = 0.002;
        controlCfg.Slot0.kV = 0.12; // ~12V feedforward


        return controlCfg;
    }

    public static TalonFXConfiguration getHoodConfigs() {
        TalonFXConfiguration hoodConfigs = new TalonFXConfiguration();
        Slot0Configs slot0Configs = hoodConfigs.Slot0;
        MotionMagicConfigs motionMagicConfigs = hoodConfigs.MotionMagic;

        // Hood Motion Magic gains (voltage output mode).
        slot0Configs.kG = 0.22;
        slot0Configs.kV = 0.0;
        slot0Configs.kA = 0.0;
        slot0Configs.kP = 8.0;
        slot0Configs.kI = 0.0;
        slot0Configs.kD = 0.15;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        hoodConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        hoodConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motionMagicConfigs.MotionMagicCruiseVelocity = 30; // Target velocity in rps
        motionMagicConfigs.MotionMagicAcceleration = 50; // Target acceleration in rps/s
        motionMagicConfigs.MotionMagicJerk = 200; // Target jerk in rps/s/s

        return hoodConfigs;
    }
}
