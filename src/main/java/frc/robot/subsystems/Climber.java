// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

public class Climber extends SubsystemBase {

  private TalonFX climberMotor;

  /** Creates a new Climber. */
  public Climber() {
    this.climberMotor = new TalonFX(1);

  }

  public void setClimberMotorSpeed(double speed) {
    
  }

  /**
   * Extends the arm
   * @return Instant Command to extend arm.
   */
  public Command extendArm() {
    return null;
  }

  /**
   * Retracts the arm
   * @return Instant Command to retract arm.
   */
  public Command retractArm() {
    return null;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
