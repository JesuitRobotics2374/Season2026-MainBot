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
  // private final motorName funnelMotor;
  private boolean funnelling;

  
  public HopperSubsystem() {
    funnelling = false;
  }

  /**
   * stops funnelling & updates boolean var
   */
  private void haltFunnel() {
    funnelling = false;
    // funnelMotor.stopMotor();
  }

  /**
   * moves feed motor backwards and updates boolean
   * @param speed speed that hopper purges
   */
  private void purgeFunnel(double speed) {
    funnelling = false;
    // funnelMotor.set(-speed);
  }

  /**
   * starts funnelling into shooter and updates boolean var
   */
  private void startFunnel() {
    funnelling = true;
    // funnelMotor.set(speed);
  }

  /**
   * getter command to get feedstate
   */
  public boolean getfunnelling() {
    return funnelling;
  }

  public Command funnel() {
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
