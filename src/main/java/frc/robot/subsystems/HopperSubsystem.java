// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

public class HopperSubsystem extends SubsystemBase {
  private final TalonFX rollerMotor;
  private boolean rolling;
  private boolean pulsing;

  public HopperSubsystem() {
    rolling = false;
    pulsing = false;
    rollerMotor = new TalonFX(23); 
  }

  /**
   * getter command to get feedstate
   */
  public boolean isRolling() {
    return rolling;
  }

  public void roll() {
    if (rolling) {
      rolling = false;
      rollerMotor.stopMotor();
    }
    else {
      rolling = true;
      rollerMotor.set(0.4);
    }
  }

  public Command purge() {
    rolling = false;
    pulsing = false;
    return new InstantCommand(() -> rollerMotor.set(-0.4));
  }

  public Command stopCommand() {
    rolling = false;
    pulsing = false;
    return new InstantCommand(() -> rollerMotor.stopMotor());
  }

  public void stopMotor() {
    rolling = false;
    pulsing = false;
    rollerMotor.stopMotor();
  }

  public Command pulse() {
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
            rollerMotor.set(0.4);
          } else {
            rollerMotor.set(-0.4);
          }
        },
        // 3. End: Stop the motor when the command is interrupted/finished
        interrupted -> {
          rollerMotor.stopMotor();
          rolling = false;
          pulsing = false;
        },
        // 4. isFinished: Return false so it runs until you release the button
        () -> !pulsing,
        // Add the subsystem requirement
        this);
  }

   /**
   * @return The current supplied to this motor in amps
   */
  public double getHopperSupplyCurrent() {
    return rollerMotor.getSupplyCurrent().getValueAsDouble(); //FOR NOW
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}