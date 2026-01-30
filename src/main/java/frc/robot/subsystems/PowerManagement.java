// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.mutable.GenericMutableMeasureImpl;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class PowerManagement extends SubsystemBase {

  private ClimberSubsystem climber;
  private HopperSubsystem hopper;
  private IntakeSubsystem intake;
  private ShooterSubsystem shooter;
  private CommandSwerveDrivetrain drivetrain;

  private double climberSupplyCurrent;
  private double intakeSupplyCurrent;
  private double hopperSupplyCurrent;
  private double shooterSupplyCurrent;

  private double drivetrainSupplyCurrent;
  private double maxDrivetrainCurrent;

  private double subsystemsCurrent; //total current drawn besides drivetrain

  /** Creates a new PowerManagementSubsystem. */
  public PowerManagement(CommandSwerveDrivetrain drivetrain, ClimberSubsystem climber, HopperSubsystem hopper, IntakeSubsystem intake, ShooterSubsystem shooter) {
    this.climber = climber;
    this.hopper = hopper;
    this.intake = intake;
    this.shooter = shooter;
    this.drivetrain = drivetrain;
  }

  @Override
  public void periodic() {
    climberSupplyCurrent = climber.getClimberSupplyCurrent();
    intakeSupplyCurrent = intake.getIntakeSupplyCurrent();
    hopperSupplyCurrent = hopper.getHopperSupplyCurrent();
    shooterSupplyCurrent = shooter.getShooterSupplyCurrent();

    drivetrainSupplyCurrent = drivetrain.getTotalDriveSupplyCurrent();

    //FOR NOW
    climber.setClimberCurrentLimit(-1);
    intake.setIntakeCurrentLimit(-1);
    hopper.setHopperCurrentLimit(-1);
    shooter.setShooterCurrentLimit(-1);

    drivetrain.setDriveCurrentLimit(-1);
    drivetrain.setSteerCurrentLimit(-1);
  }
}