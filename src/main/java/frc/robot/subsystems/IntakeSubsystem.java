// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor;
  private final TalonFX pivotMotor;
  private boolean raised;
  private boolean lowered;

  private double targetSpeed = -0.8;

  private static final double CURRENT_LIMIT = 25.0; // Amps

  private boolean isIntaking;

  /** Creates a new Intake. */
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(37);
    pivotMotor = new TalonFX(35); 

    pivotMotor.setPosition(0);
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    raised = true;
    lowered = false;

    TalonFXConfiguration controlCfg = new TalonFXConfiguration();
    controlCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    controlCfg.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
    controlCfg.CurrentLimits.StatorCurrentLimitEnable = true;
    controlCfg.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT / 0.75;

    intakeMotor.getConfigurator().apply(controlCfg);
  }

  public Command raiseManual() {
    return new InstantCommand(() -> pivotMotor.set(0.1));
  }

  public Command lowerManual() {
    return new InstantCommand(() -> pivotMotor.set(-0.1));
  }

  public Command stopPivot() {
    return new InstantCommand(() -> pivotMotor.set(0));
  }

  //FIX THESE SOMEDAY

  // public Command raise() {
  //   return new FunctionalCommand(
  //       () -> {
  //       },
  //       () -> {
  //         pivotMotor.set(0.5);
  //         raised = pivotMotor.getPosition().getValueAsDouble() >= 0;
  //       },
  //       interrupted -> {
  //         pivotMotor.set(0);
  //       },
  //       () -> raised,
  //       this);
  // }

  // public Command lower() {
  //   return new FunctionalCommand(
  //       () -> {
  //       },
  //       () -> {
  //         pivotMotor.set(-0.5);
  //         lowered = pivotMotor.getPosition().getValueAsDouble() <= -2.25; //SEVERELY INCORRECT
  //       },
  //       interrupted -> {
  //         pivotMotor.set(0);
  //       },
  //       () -> lowered,
  //       this);
  // }

  public double getTargetSpeed() {
    return targetSpeed;
  }

  public void increaseTargetSpeed(double delta) {
    targetSpeed += -Math.abs(delta);
  }

  public void decreaseTargetSpeed(double delta) {
    targetSpeed += Math.abs(delta);
  }

  public void intake() {
    if (isIntaking) {
      isIntaking = false;
      intakeMotor.stopMotor();;
    }
    else {
      isIntaking = true;
      intakeMotor.set(targetSpeed);
    }
  }

  public Command stop() {
    isIntaking = false;
    return new InstantCommand(() -> intakeMotor.set(0), this);
  }

  public Command purge() {
    return new InstantCommand(() -> intakeMotor.set(0.3), this);
  }

  public double getIntakeSupplyCurrent() {
    return intakeMotor.getSupplyCurrent().getValueAsDouble() +
           pivotMotor.getSupplyCurrent().getValueAsDouble();
  }

  public boolean isIntaking() {
    return isIntaking;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}