// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import javax.naming.ldap.StartTlsResponse;

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
  private void startFunnel(double speed) {
    funnelling = true;
    // funnelMotor.set(speed);
  }

  /**
   * getter command to get feedstate
   */
  public boolean getfunnelling() {
    return funnelling;
  }

  public Command funnel(double speed) {
    return new InstantCommand(() -> startFunnel(speed));
  }

  public Command purge(double speed) {
    return new InstantCommand(() -> purgeFunnel(speed));
  }

  public Command stop() {
    return new InstantCommand(() -> haltFunnel());
  }

 /**
   * @return The current supplied to this motor in amps
   */
  public double getHopperSupplyCurrent() {
    return 0; //FOR NOW
  }

   /**
   * Sets the current limit of this motor
   * currentLimit at -1 means default
   */
  public void setHopperCurrentLimit(double currentLimit) {
    if (currentLimit == -1) {
      return;
    }

    //configure motor current limit
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    
  }
}
