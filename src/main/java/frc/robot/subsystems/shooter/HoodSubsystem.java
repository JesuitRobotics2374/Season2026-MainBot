// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Configs;
import frc.robot.utils.Constants;
import frc.robot.utils.Devices;

public class HoodSubsystem extends SubsystemBase {
  /** Creates a new HoodSubsystem. */

  // Adjusts shooting angle
  private final TalonFX hood;

  // Safety lock: when true, hood will never be commanded to move.
  private static final boolean HOOD_DISABLED = true;

  private double hoodTargetPos;

  private boolean hoodDown = true;

  private final MotionMagicVoltage hoodMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0);

  public HoodSubsystem() {
    hood = Devices.hoodControl;

    TalonFXConfiguration hoodConfigs = Configs.getHoodConfigs();

    // Push one full config object so Slot0 + MotionMagic are guaranteed to match
    // this request.
    hood.getConfigurator().apply(hoodConfigs);

    // Assumes the hood starts at the mechanical minimum at startup.
    hood.setPosition(Constants.HOOD_MIN_SETPOINT);
    hoodTargetPos = Constants.HOOD_MIN_SETPOINT;

    hoodDown = true;
    zeroHood();
  }

  /**
   * Resets hood encoder position to zero (or mechanical minimum) for consistent
   * starting point.
   */
  public void zeroHood() {
    hood.setPosition(0.0);
    hoodTargetPos = 0.0; // or Constants.HOOD_MIN_SETPOINT
  }

  /**
   * Commands hood to move to target position using Motion Magic control.
   */
  public void moveHoodtoPos() {
    if (HOOD_DISABLED) {
      hood.stopMotor();
      return;
    }

    hood.setControl(hoodMotionMagicRequest.withPosition(hoodTargetPos));
  }

  /**
   * Sets the target position for the hood. 
   * @param targetPos Desired hood position
   */
  public void setHoodTargetPosition(double targetPos) {
    hoodTargetPos = targetPos;
  }

  /**
   * Manually toggles hood between minimum and maximum positions, ignoring any
   * auto-range logic. This is intended for use with a dedicated manual override
   * button.
   * If AUTO_HOOD_DISABLED is true, this method will stop the hood motor instead
   * of moving it.
   */
  private void manualToggleHoodMinMax() {
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
   * Returns the target position for the hood.
   * 
   * @return The target hood position
   */
  public double getHoodTargetPosition() {
    return hoodTargetPos;
  }

  /**
   * @return The hood's position
   */
  public double getHoodPosition() {
    return hood.getRotorPosition().getValueAsDouble();
  }
  
  /**
   * @return True if hood is currently in the down (minimum) position, false if
   *         up.
   */
  public boolean isHoodDown() {
    return hoodDown;
  }

  public Command manualToggleHoodMinMaxCommand() {
    return new InstantCommand(() -> manualToggleHoodMinMax());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
