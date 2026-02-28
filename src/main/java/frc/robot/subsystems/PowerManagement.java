// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.subsystems.drivetrain.DriveSubsystem;

public class PowerManagement extends SubsystemBase {

  private ClimberSubsystem climber;
  private HopperSubsystem hopper;
  private IntakeSubsystem intake;
  private ShooterSubsystem shooter;
  private DriveSubsystem drivetrain;

  private double driveLimit;
  private double steerLimit;

  /** Creates a new PowerManagementSubsystem. */
  public PowerManagement(DriveSubsystem drivetrain, ClimberSubsystem climber, HopperSubsystem hopper,
      IntakeSubsystem intake, ShooterSubsystem shooter) {
    this.climber = climber;
    this.hopper = hopper;
    this.intake = intake;
    this.shooter = shooter;
    this.drivetrain = drivetrain;

    driveLimit = Constants.DEFAULT_DRIVE_CURRENT;
    steerLimit = Constants.DEFAULT_STEER_CURRENT;

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

  private boolean lastShooting = false;
  private boolean lastIntaking = false;

  double clock = 0;

  private void asyncUpdateLimits() {
    Command asyncUpdateLimitsCommand =  new InstantCommand(() -> {
      drivetrain.setDriveCurrentLimit(driveLimit, driveLimit / 0.65);
      drivetrain.setSteerCurrentLimit(steerLimit, steerLimit / 0.65);
    });
    CommandScheduler.getInstance().schedule(asyncUpdateLimitsCommand);
  }

  @Override
  public void periodic() {
    // clock++;
    // if (clock == 10) {
    //   clock = 0;
    //   //System.out.println("ran periodic");
    // }
  

    // boolean currentShooting = shooter.isShooting();
    // boolean currentIntaking = intake.isIntaking();

    // // Only continue if something changed
    // if (currentShooting == lastShooting &&
    //     currentIntaking == lastIntaking) {
    //   return; // Nothing changed → skip config entirely
    // }

    // System.out.println("changing configs");

    // // Update stored states
    // lastShooting = currentShooting;
    // lastIntaking = currentIntaking;

    // // Determine limits
    // if (currentShooting && currentIntaking) {
    //   driveLimit = 20;
    //   steerLimit = 10;
    // } else if (currentShooting) {
    //   driveLimit = 25;
    //   steerLimit = 15;
    // } else if (currentIntaking) {
    //   driveLimit = 30;
    //   steerLimit = 20;
    // } else {
    //   driveLimit = Constants.DEFAULT_DRIVE_CURRENT;
    //   steerLimit = Constants.DEFAULT_STEER_CURRENT;
    // }

    // // Apply config ONLY when state changed
    // asyncUpdateLimits();
  }

}