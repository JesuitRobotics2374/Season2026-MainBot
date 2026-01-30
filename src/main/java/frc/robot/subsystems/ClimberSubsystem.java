// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberSubsystem extends SubsystemBase {

  private final TalonFX climberMotor;
  private final double minRotations = 0;
  private final double maxRotations = -31;

  /** Creates a new Climber. */
  public ClimberSubsystem() {
    this.climberMotor = new TalonFX(1);
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

  /**
   * Extends the arm
   * @return Functional Command to extend arm.
   */
  public Command extendArm() {
    //return new InstantCommand(() -> setClimberMotorSpeed(1));

    return new FunctionalCommand(
      //init
      () -> {setMotorSpeed(0.5);},
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
      () -> {setMotorSpeed(-0.5);},
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

  /**
   * Checks the position of the motor and stops it
   * @return boolean
   */
  public boolean hasReachedMax() {
    if (climberMotor.getPosition().getValueAsDouble() <= maxRotations) {
      return true;
    }
    return false;
  }

  /**
   * Checks the position of the motor and stops it
   * @return
   */
  public boolean hasReachedMin() {
    if (climberMotor.getPosition().getValueAsDouble() >= minRotations) {
      return true;
    }
    return false;
  }

  /**
   * @return The current supplied to this motor in amps
   */
  public double getClimberSupplyCurrent() {
    return climberMotor.getSupplyCurrent().getValueAsDouble();
  }

  /**
   * Sets the current limit of this motor
   * current limit at -1 means default
   */
  public void setClimberCurrentLimit(double currentLimit) {
    if (currentLimit == -1) {
      return;
    }
    CurrentLimitsConfigs configs = new CurrentLimitsConfigs().withSupplyCurrentLimit(currentLimit).withSupplyCurrentLimitEnable(true);
    climberMotor.getConfigurator().apply(configs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
