// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class TestMotor extends SubsystemBase {
  private TalonFX testMotor;
  public double MAX_RPM = 100;
  public double targetRpm = 0;

  /** Creates a new TestMotor. */
  public TestMotor() {
    this.testMotor = new TalonFX(11);
    VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

    this.testMotor = new TalonFX(11);
    testMotor.setControl(velocityRequest.withVelocity(2));
  }

  private void setTargetRpm(double rpm) {
    if (rpm > MAX_RPM) rpm = MAX_RPM;
    if (rpm < -MAX_RPM) rpm = -MAX_RPM;
    targetRpm = rpm;
    }

  public Command setTargetRPM(double rpm) {
    return new InstantCommand(() -> setTargetRpm(rpm));
  }

  public Command setRPM() {
    return new InstantCommand(() -> setTargetRpm(2));
  }

  public Command stop() {
    return new InstantCommand(() -> setTargetRPM(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
