// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class IntakeSubsystem extends SubsystemBase {
  
  private boolean intaking;
  private TalonFX intakeMotor;

  public IntakeSubsystem() {
    this.intakeMotor = new TalonFX(38, "FastFD"); 
    intaking = false;
  }

  /**
   * Stops intaking
   */
  private void stopIntake() {
    intaking = false;
    intakeMotor.stopMotor();
  }

  /**
   * Stops intaking and reverse intake to get rid of a stuck fuel or smth
   * @param speed the speed that the intake purges the fuel
   */
  private void purge(double speed) {
    intaking = false;
    intakeMotor.set(speed);
  }

  /**
   * Intakes fuel into the hopper
   * @param speed the speed that the robot intakes
   */
  private void intakeFuel(double speed) {
    intaking = true;
    intakeMotor.set(speed);
  }

  /**
   * Tells if intake is intaking
   * @return true = intaking
   */
  public boolean getIntaking() {
    return intaking;
  }

  public Command intake() {
    return new InstantCommand(() -> intakeFuel(-0.5));
  }

  public Command purge() {
    return new InstantCommand(() -> purge(0.5));
  }

  public Command stop() {
    return new InstantCommand(() -> stopIntake());
  }

  /**
   * @return The current supplied to this motor in amps
   */
  public double getIntakeSupplyCurrent() {
    return intakeMotor.getSupplyCurrent().getValueAsDouble();
  }

   /**
   * Sets the current limit of this motor
   * -1 current limit means default configs
   */
  public void setIntakeCurrentLimit(double currentLimit) {
    if (currentLimit == -1) {
      return;
    }
    CurrentLimitsConfigs configs = new CurrentLimitsConfigs().withSupplyCurrentLimit(currentLimit).withSupplyCurrentLimitEnable(true);
    intakeMotor.getConfigurator().apply(configs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}