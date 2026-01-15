// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase {
  // private final motorName feedMotor;
  private boolean feeding;

  
  public HopperSubsystem() {
    feeding = false;
  }

  /**
   * stops feeding & updates boolean var
   */
  private void haltFeed() {
    feeding = false;
    // feedMotor.stopMotor();
  }

  /**
   * moves feed motor backwards and updates boolean
   * @param speed speed that hopper purges
   */
  private void purgeFeed(double speed) {
    feeding = false;
    // feedMotor.set(-speed);
  }

  /**
   * starts feeding into shooter and updates boolean var
   */
  private void startFeed() {
    feeding = true;
    // feedMotor.set(speed);
  }

  /**
   * getter command to get feedstate
   */
  public boolean getFeeding() {
    return feeding;
  }

  public Command feed() {
    return null;
    // key binds for feed
  }

  public Command purge() {
    return null;
    // key binds for purging
  }

  public Command stop() {
    return null;
    // key binds for stopping hopper motors
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
