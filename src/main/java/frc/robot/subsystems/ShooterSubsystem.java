// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterSubsystem extends SubsystemBase {

  double speed;
  private double targetRpm = 0.0;

  private static final double MAX_RPM = 6000.0;
  private static final double RPM_TO_RPS = 1.0 / 60.0;
  private static final double CURRENT_LIMIT = 40.0; // Amps
  // private final (motor name) shooterMotor;
  // private final (something) velocityController;
  // private final relativeEncoder encoder;

  /** Creates a new Shooter. */
  public ShooterSubsystem() {

  }

  private void startShooting(double speed) {
    // motor.set(speed);
  }

  private void stopShooting() {
    // motor.set(0);
  }

  private void startPurge(double speed) {
    // motor.set(-speed);
  }


  /**
   * Starts the shooter when ready to shoot
   * 
   * @param speed
   */
  public Command startShoot(double speed) {
    return new InstantCommand(() -> startShooting(speed));
  }

  /**
   * Stops the shooter when needed
   * 
   * @param speed
   */
  public Command stopShoot() {
    return new InstantCommand(() -> stopShooting());
  }

  /**
   * Purges the shooter if balls get stuck
   * 
   * @param speed
   * @return
   */
  public Command purgeShooter(double speed) {
    return new InstantCommand(() -> startPurge(speed));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
