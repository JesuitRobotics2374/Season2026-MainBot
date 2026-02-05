// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberSubsystem extends SubsystemBase {

  private TalonFX climberMotor;
  private final double topRotations = 0;
  private final double bottomRotations = 78;

  /** Creates a new Climber. */
  public ClimberSubsystem() {
    this.climberMotor = new TalonFX(16);
    climberMotor.setPosition(0);
    climberMotor.setNeutralMode(NeutralModeValue.Brake);
  }
  
  /**
   * Sets the speed of the motor to extend or retract the arm
   * @param speed
   */
  private void setMotorSpeed(double speed) {
    climberMotor.set(speed);

  }

  public void rotations() {
    System.out.println(climberMotor.getPosition().getValueAsDouble());
  }

  /**
   * Extends the arm
   * @return Functional Command to extend arm.
   */
  public Command extendArm() {
    //return new InstantCommand(() -> setClimberMotorSpeed(1));

    return new FunctionalCommand(
      //init
      () -> {setMotorSpeed(-0.5);},
      //execute
      () -> {},
      //interrupt
      interrupted -> {setMotorSpeed(0);},
      //isFinished
      () -> hasReachedMax(),
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
      () -> {setMotorSpeed(0.5);},
      //execute
      () -> {},
      //interrupt
      interrupted -> {setMotorSpeed(0);},
      //isFinished
      () -> hasReachedMin(),
      //requirements
      this
    );

  }

  public Command zeroClimber() {
    return new InstantCommand(() -> climberMotor.setPosition(0));
  }

  public Command setSpeed(double speed) {
    return new InstantCommand(() -> setMotorSpeed(speed));
  }

  /**
   * Checks the position of the motor and stops it
   * @return boolean
   */
  public boolean hasReachedMax() {
    if (Math.abs(climberMotor.getPosition().getValueAsDouble() - topRotations) < 0.1) {
      System.out.println(true);
      return true;
    }
    return false;
  }

  /**
   * Checks the position of the motor and stops it
   * @return
   */
  public boolean hasReachedMin() {
    if (Math.abs(climberMotor.getPosition().getValueAsDouble() - bottomRotations) < 0.1) {
      System.out.println(true);
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
}
