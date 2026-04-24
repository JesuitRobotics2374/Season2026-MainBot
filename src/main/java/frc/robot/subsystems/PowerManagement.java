// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class PowerManagement extends SubsystemBase {

  private DriveSubsystem drivetrain;

  private double driveLimit;
  private double steerLimit;

  private boolean isBoosted;

  /** Creates a new PowerManagementSubsystem. */
  public PowerManagement(DriveSubsystem drivetrain, HopperSubsystem hopper,
      IntakeSubsystem intake, ShooterSubsystem shooter) {
    this.drivetrain = drivetrain;

    driveLimit = Constants.DEFAULT_DRIVE_CURRENT;
    steerLimit = Constants.DEFAULT_STEER_CURRENT;

    isBoosted = false;

    drivetrain.setDriveCurrentLimit(driveLimit, driveLimit / 0.65);
    drivetrain.setSteerCurrentLimit(steerLimit, steerLimit / 0.65);

    System.out.println("Power Management Initialized");
  }

  public double getSteerLimit() {
    return steerLimit;
  }

  public double getDriveLimit() {
    return driveLimit;
  }

  double clock = 0;

  public Command toggleDriveBoost() {
    return new InstantCommand(() -> {
      if (isBoosted) {
        driveLimit = Constants.DEFAULT_DRIVE_CURRENT;
        steerLimit = Constants.DEFAULT_STEER_CURRENT;
      } else {
        driveLimit = Constants.BOOSTED_DRIVE_CURRENT;
        steerLimit = Constants.BOOSTED_STEER_CURRENT;
      }

      isBoosted = !isBoosted;

      drivetrain.setDriveCurrentLimit(driveLimit, driveLimit / 0.65);
      drivetrain.setSteerCurrentLimit(steerLimit, steerLimit / 0.65);
    }, this);
  }

  @Override
  public void periodic() {

  }
}