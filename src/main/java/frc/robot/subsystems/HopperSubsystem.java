// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class HopperSubsystem extends SubsystemBase {

  private final TalonFX rollerMotor;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  private double targetRPM = 1800;

  private boolean rolling;
  private boolean pulsing;

  private final double RPM_TO_RPS = 1.0 / 60.0;

  public HopperSubsystem() {
    rolling = false;
    pulsing = false;
    rollerMotor = new TalonFX(32);

    TalonFXConfiguration controlCfg = new TalonFXConfiguration();
    controlCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    
    controlCfg.Slot0.kP = 0.09;
    controlCfg.Slot0.kI = 0;
    controlCfg.Slot0.kD = 0.001;
    controlCfg.Slot0.kV = 0.12;

    rollerMotor.getConfigurator().apply(controlCfg);
  }

  private void changeRPM(double delta) {
    targetRPM += delta;
  }

  public void spinForwards() {
    rollerMotor.setControl(velocityRequest.withVelocity(targetRPM * RPM_TO_RPS));
  }

  public void spinBackwards() {
    rollerMotor.setControl(velocityRequest.withVelocity(-targetRPM * RPM_TO_RPS));
  }

  public void stopMotor() {
    rolling = false;
    pulsing = false;
    rollerMotor.stopMotor();
  }

  public Command rollCommand() {
    if (rolling) {
      rolling = false;
      return new InstantCommand(() -> rollerMotor.stopMotor());
    } else {
      rolling = true;
      return new InstantCommand(() -> spinForwards());
    }
  }

  public Command purgeCommand() {
    rolling = false;
    pulsing = false;
    return new InstantCommand(() -> spinBackwards());
  }

  public Command stopCommand() {
    return new InstantCommand(() -> stopMotor());
  }

  public Command changeRPMCommand(double delta) {
    return new InstantCommand(() -> changeRPM(delta));
  }

  public Command pulseCommand() {
    Timer timer = new Timer();

    return new FunctionalCommand(
        // 1. Initialize: Start the timer when the command begins
        () -> {
          timer.restart();
          rolling = false;
          pulsing = true;
        },
        // 2. Execute: Toggle motor power based on the timer
        () -> {
          // Pulse logic: 0.4s ON, 0.2s OFF (Total 0.6s cycle)
          if ((timer.get() % 0.6) < 0.4) {
            spinForwards();
          } else {
            spinBackwards();
          }
        },
        // 3. End: Stop the motor when the command is interrupted/finished
        interrupted -> {
          stopMotor();
        },
        // 4. isFinished: Return false so it runs until you release the button
        () -> !pulsing,
        // Add the subsystem requirement
        this);
  }

  /**
   * getter method to get feedstate
   */
  public boolean isRolling() {
    return rolling;
  }

  /**
   * @return The current supplied to this motor in amps
   */
  public double getHopperSupplyCurrent() {
    return rollerMotor.getSupplyCurrent().getValueAsDouble(); // FOR NOW
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}