// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

public class Climber extends SubsystemBase {

  private TalonFX climberMotor;
  private double rotations;

  /** Creates a new Climber. */
  public Climber() {
    this.climberMotor = new TalonFX(1);
    this.rotations = 100; //FIX
  }
  
  /**
   * Sets the speed of the motor to extend or retract the arm
   * @param speed
   */
  public void setClimberMotorSpeed(double speed) {
    climberMotor.set(speed);

  }

  /**
   * Extends the arm
   * @return Instant Command to extend arm.
   */
  public Command extendArm() {
    //return new InstantCommand(() -> setClimberMotorSpeed(1));

    return new FunctionalCommand(
      //init
      () -> {setClimberMotorSpeed(1);},
      //execute
      () -> {},
      //interrupt
      interrupted -> {setClimberMotorSpeed(0);},
      //isFinished
      () -> isFinished(),
      //requirements
      this
    );

  }

  /**
   * Retracts the arm
   * @return Instant Command to retract arm.
   */
  public Command retractArm() {
    // new InstantCommand(() -> setClimberMotorSpeed(-1));
    return new FunctionalCommand(
      //init
      () -> {setClimberMotorSpeed(-1);},
      //execute
      () -> {},
      //interrupt
      interrupted -> {setClimberMotorSpeed(0);},
      //isFinished
      () -> isFinished(),
      //requirements
      this
    );

  }
  /**
   * Checks the position of the motor and stops it
   * @return
   */
  public boolean isFinished() {
    if (climberMotor.getPosition().getValueAsDouble() <= rotations) {
      return true;
    }
    return false;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
}
