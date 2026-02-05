// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.DriveSubsystem;

public class PowerManagement extends SubsystemBase {

  private ClimberSubsystem climber;
  private HopperSubsystem hopper;
  private IntakeSubsystem intake;
  private ShooterSubsystem shooter;
  private DriveSubsystem drivetrain;

  private double climberSupplyCurrent;
  private double intakeSupplyCurrent;
  private double hopperSupplyCurrent;
  private double shooterSupplyCurrent;

  private double drivetrainSupplyCurrent;

  private final double maxCurrent;

  /** Creates a new PowerManagementSubsystem. */
  public PowerManagement(DriveSubsystem drivetrain, ClimberSubsystem climber, HopperSubsystem hopper,
      IntakeSubsystem intake, ShooterSubsystem shooter) {
    this.climber = climber;
    this.hopper = hopper;
    this.intake = intake;
    this.shooter = shooter;
    this.drivetrain = drivetrain;

    maxCurrent = Constants.MAX_CURRENT;
  }

  @Override
  public void periodic() {
    climberSupplyCurrent = climber.getClimberSupplyCurrent();
    intakeSupplyCurrent = intake.getIntakeSupplyCurrent();
    hopperSupplyCurrent = hopper.getHopperSupplyCurrent();
    shooterSupplyCurrent = shooter.getShooterSupplyCurrent();

    drivetrainSupplyCurrent = drivetrain.getTotalDriveSupplyCurrent();

    // FOR NOW (speculative values)
    if (!climber.hasReachedMax() || !climber.hasReachedMin()
        || !climber.hasReachedMax() && !climber.hasReachedMin() && shooter.isRunning(shooterSupplyCurrent)) {
      drivetrain.setDriveCurrentLimit(40); // Lowest limit because demand is highest
    }
    // Check shooter and climber, I dont think we should reduce power to drive when
    // intake is on
    else if (shooter.isRunning(shooterSupplyCurrent)) {
      drivetrain.setDriveCurrentLimit(45);
    } else if (!climber.hasReachedMax() || !climber.hasReachedMin()
        || !climber.hasReachedMax() && !climber.hasReachedMin()) {
      drivetrain.setDriveCurrentLimit(45);
    }
    // If nothing else is happening
    else {
      drivetrain.setDriveCurrentLimit(50); // Max speed whilst only drive is running
    }
    // for steerMotor
    if (!climber.hasReachedMax() || !climber.hasReachedMin()
        || !climber.hasReachedMax() && !climber.hasReachedMin() && shooter.isRunning(shooterSupplyCurrent)) {
      // Lowest power if both climber and shooter are enabled
      drivetrain.setSteerCurrentLimit(20);
    } else if (shooter.isRunning(shooterSupplyCurrent)) {
      // Just the shooter is on
      drivetrain.setSteerCurrentLimit(25);
    } else if (climber.hasReachedMax() || climber.hasReachedMin()
        || !climber.hasReachedMax() && !climber.hasReachedMin()) {
      // Just the climber is on
      drivetrain.setSteerCurrentLimit(25);
    } else {
      // Max steering speed if nothing else is on
      drivetrain.setSteerCurrentLimit(30);
    }

    drivetrain.setDriveCurrentLimit(
        Math.min(160, (maxCurrent - shooterSupplyCurrent - intakeSupplyCurrent - hopperSupplyCurrent) * 0.66));
    drivetrain.setSteerCurrentLimit(
        Math.min(80, (maxCurrent - shooterSupplyCurrent - intakeSupplyCurrent - hopperSupplyCurrent) * 0.34));

  }
}