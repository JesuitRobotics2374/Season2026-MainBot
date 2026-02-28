// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor;
  private final TalonFX pivotMotor;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private boolean raised;
  private boolean lowered;

  private double MAX_RPM = 2000;
  private double targetRPM = 4000;

  private final double RPM_TO_RPS = 1.0 / 60.0;
  private static final double CURRENT_LIMIT = 60.0; // Amps

  private double targetPos; // the target position of the pivotMotor

  private boolean isIntaking;

  /** Creates a new Intake. */
  public IntakeSubsystem() {
    pivotMotor = new TalonFX(30);
    intakeMotor = new TalonFX(31);

    TalonFXConfiguration controlCfg = new TalonFXConfiguration();

    controlCfg.Slot0.kP = 0.2;
    controlCfg.Slot0.kI = 0.001;
    controlCfg.Slot0.kD = 0.01;
    controlCfg.Slot0.kV = 0.12;
    controlCfg.Slot0.kS = 0.01;

    controlCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    controlCfg.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
    controlCfg.CurrentLimits.StatorCurrentLimitEnable = true;
    controlCfg.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT / 0.75;

    intakeMotor.getConfigurator().apply(controlCfg);

    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    // Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    // MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;

    // slot0Configs.kG = 0.2; // Output of voltage to overcome gravity
    // slot0Configs.kV = 2; // Output per unit target velocity, perhaps not needed
    // slot0Configs.kA = 0.3; // Output per unit target acceleration, perhaps not needed
    // slot0Configs.kP = 15; // Controls the response to position error—how much the motor reacts to the
    //                       // difference between the current position and the target position.
    // slot0Configs.kI = 1.5; // Addresses steady-state error, which occurs when the motor doesn’t quite reach
    // // the target position due to forces like gravity or friction.
    // slot0Configs.kD = 0.3; // Responds to the rate of change of the error, damping the motion as the motor
    //                        // approaches the target. This reduces overshooting and oscillations.

    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target velocity in rps
    // motionMagicConfigs.MotionMagicAcceleration = 68; // Target acceleration in rps/s
    // motionMagicConfigs.MotionMagicJerk = 400; // Target jerk in rps/s/s

    pivotMotor.getConfigurator().apply(talonFXConfigs);
    // pivotMotor.getConfigurator().apply(slot0Configs);
    // pivotMotor.getConfigurator().apply(motionMagicConfigs);

    //setZero();

    raised = true;
    lowered = false;
  }

  // private void updateIntakePos() {

  //   MotionMagicVoltage m_request = new MotionMagicVoltage(targetPos);

  //   pivotMotor.setControl(m_request);
  // }

  // private void intakeChangeBy(double deltaPos) {
  //   targetPos += deltaPos;

  //   updateIntakePos();
  // }

  // private void setPositionIntake(double pos) {
  //   targetPos = pos;

  //   updateIntakePos();
  // }

  // private void setZero() {
  //   pivotMotor.setPosition(0.0);
  //   targetPos = 0;

  //   updateIntakePos();
  // }

  private void stop() {
    intakeMotor.stopMotor();
  }

  private void setTargetRPM(double RPM) {
    if (RPM > MAX_RPM)
      RPM = MAX_RPM;
    if (RPM < -MAX_RPM)
      RPM = -MAX_RPM;

    targetRPM = RPM;
  }

  private void changeTargetRPM(double deltaRPM) {
    setTargetRPM(targetRPM + deltaRPM);
  }

  private void rotate(double targetRPM) {
    System.out.println("targetrpm intake:" + getTargetRPM());
    intakeMotor.setControl(velocityRequest.withVelocity(targetRPM * RPM_TO_RPS));
  }

  // public Command deltaPivotCommand(double delta) {
  //   return new InstantCommand(() -> intakeChangeBy(delta), this);
  // }

  // public Command setPositionCommand(double pos) {
  //   return new InstantCommand(() -> setPositionIntake(pos), this);
  // }

  // public Command zeroPivotCommand() {
  //   return new InstantCommand(() -> setZero(), this);
  // }

  public Command raiseManual() {
    return new InstantCommand(() -> pivotMotor.set(0.1));
  }

  public Command lowerManual() {
    return new InstantCommand(() -> pivotMotor.set(-0.1));
  }

  public Command stopPivot() {
    return new InstantCommand(() -> pivotMotor.set(0));
  }

  public Command intakeCommand() {
    return new FunctionalCommand(
        () -> {
          // rotate(getTargetRPM());
        },
        () -> {
          rotate(getTargetRPM());
        },
        interrupted -> {
          stop();
        },
        () -> false,
        this);
  }

  public Command stopCommand() {
    return new InstantCommand(() -> stop(), this);
  }

  public Command changeTargetRPMCommand(double deltaRPM) {
    return new InstantCommand(() -> changeTargetRPM(deltaRPM), this);
  }

  public double getTargetRPM() {
    return targetRPM;
  }

  public double getIntakeSupplyCurrent() {
    return intakeMotor.getSupplyCurrent().getValueAsDouble() +
        pivotMotor.getSupplyCurrent().getValueAsDouble();
  }

  public boolean isIntaking() {
    return isIntaking;
  }

  public double getSpeedRPM() {
    return intakeMotor.getRotorVelocity().getValueAsDouble() * 60;
  }

  public boolean isPurging() {
    return getSpeedRPM() < 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // private void rotateAtCached() {
  //   if (isIntaking) {
  //     isIntaking = false;
  //     stop();
  //   } else {
  //     isIntaking = true;
  //     rotate(targetRPM);
  //   }
  // }
}