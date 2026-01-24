// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.*;

public class LogSubsystem extends SubsystemBase {
  private final DoubleLogEntry currentLog;
  private final DoubleLogEntry voltageLog;

  public LogSubsystem(String motorName) {
      var log = DataLogManager.getLog();
      currentLog = new DoubleLogEntry(log, "/motors/" + motorName + "/current");
      voltageLog = new DoubleLogEntry(log, "/motors/" + motorName + "/voltage");
  }

  public void log(double current, double voltage) {
    currentLog.append(current);
    voltageLog.append(voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
