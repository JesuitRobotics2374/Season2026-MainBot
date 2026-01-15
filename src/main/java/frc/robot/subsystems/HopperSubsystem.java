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
  /** Creates a new HopperSubsystem. */
  // private final motorName feedMotor;
  private boolean feeding;
  public HopperSubsystem() {
    feeding = false;
  }

  private void haltFeed() {
    feeding = false;
    // feedMotor.stopMotor();
  }

  private void purgeFeed(double speed){
    feeding = false;
    // feedMotor.set(-speed);
  }

  private void startFeed(){
    feeding = true;
    // feedMotor.set(speed);
  }

  public boolean getFeeding(){
    return feeding;
  }

  // public Command Feed(){

  // }

  // public Command Purge(){

  // }

  // public Command Stop(){

  // }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
