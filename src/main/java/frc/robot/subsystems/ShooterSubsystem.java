// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.lang.annotation.Retention;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class ShooterSubsystem extends SubsystemBase {

  private TalonFX shooterLeft;
  private TalonFX shooterRight;

  private double currentSpeed = 0.0;
  private double targetRpm = 0.0;

  // Constants
  private static final double MAX_RPM = 6000.0;
  private static final double RPM_TO_RPS = 1.0 / 60.0;
  private static final double CURRENT_LIMIT = 40.0; // Amps

  /** Creates a new Shooter. */
  public ShooterSubsystem() {
    this.shooterLeft = new TalonFX(0); // change device ID
    this.shooterRight = new TalonFX(1); // change device ID

    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();

    leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
    leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
    rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    shooterLeft.getConfigurator().apply(leftConfig);
    shooterRight.getConfigurator().apply(rightConfig);

  }

  private void setSpeed(double speed) {
    shooterLeft.set(speed);
    shooterRight.set(speed);
  }

  /**
   * Starts the shooter when ready to shoot
   * 
   * @param speed
   */
  public Command shoot(double speed) {
    return new InstantCommand(() -> setSpeed(Math.abs(speed)), this);
  }

  /**
   * Stops the shooter when needed
   * 
   * @param speed
   */
  public Command stopShoot() {
    return new InstantCommand(() -> setSpeed(0), this);
  }

  /**
   * Purges the shooter if balls get stuck
   * 
   * @param speed output speed
   * @return
   */
  public Command purgeShooter(double speed) {
    return new InstantCommand(() -> setSpeed(-Math.abs(speed)), this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
