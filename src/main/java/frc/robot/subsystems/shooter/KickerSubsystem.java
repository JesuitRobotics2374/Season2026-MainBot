// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Configs;
import frc.robot.utils.Constants;
import frc.robot.utils.Devices;

public class KickerSubsystem extends SubsystemBase {
  /** Creates a new KickerSubsystem. */

  // Feeds ball into flywheel
  private final TalonFX kicker;

  // Reusable velocity control request to prevent object allocation in loops
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  private double targetRPMKicker = Constants.DEFAULT_KICKER_RPM;
  private static final double MAX_RPM = 5400;
  private static final double RPM_TO_RPS = 1.0 / 60.0;

  public KickerSubsystem() {
    kicker = Devices.kickerControl;

    TalonFXConfiguration controlCfgKicker = Configs.getkickerConfigs();

    kicker.getConfigurator().apply(controlCfgKicker);
  }

  /**
   * Sets target RPM for kicker motor (clamped).
   *
   * @param RPM Desired kicker RPM
   */
  public void setTargetRPMKicker(double RPM) {
    if (RPM > MAX_RPM)
      RPM = MAX_RPM;
    if (RPM < -MAX_RPM)
      RPM = -MAX_RPM;

    targetRPMKicker = RPM;
  }

  /**
   * Adjusts kicker RPM by delta amount.
   *
   * @param deltaRPM Change in RPM
   */
  public void changeKickerTargetRPM(double deltaRPM) {
    setTargetRPMKicker(targetRPMKicker + deltaRPM);
  }

  /**
   * Applies closed-loop velocity control to kicker motor.
   */
  public void setKickerControl() {
    kicker.setControl(velocityRequest.withVelocity(targetRPMKicker * RPM_TO_RPS));
  }

  /**
   * Stops kicker motor.
   */
  public void stopKicker() {
    kicker.stopMotor();
  }

  /**
   * @return Target kicker RPM
   */
  public double getTargetRPMKicker() {
    return targetRPMKicker;
  }

  /**
   * @return Kicker motor RPM
   */
  public double getSpeedRPMKicker() {
    return kicker.getRotorVelocity().getValueAsDouble() * 60.0;
  }

  /**
   * Checks if the kicker is currently kicking.
   * @return true if the kicker is kicking, false otherwise
   */
  public boolean isKicking() {
    return Math.abs(getSpeedRPMKicker()) > 100; // Threshold to consider "kicking"
  }

  public Command changeKickerTargetRPMCommand(double deltaRPM) {
    return new InstantCommand(() -> changeKickerTargetRPM(deltaRPM));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
