// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Configs;
import frc.robot.utils.Devices;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeControl;
  private final TalonFX pivotMotor;
  private final TalonFX intakeFollower;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0).withSlot(0);

  // Pivot motion limits in mechanism rotations (motor sensor rotations).
  // Tune these based on your zeroing process and physical hard stops.
  private static final double PIVOT_MIN_ROT = -29; // lowered
  private static final double PIVOT_MAX_ROT = 0; // raised
  private static final double PIVOT_CMD_EPSILON_ROT = 0.002;

  private double MAX_RPM = 6300;
  private double targetRPM = 5000;

  private double purgeRPM = -2000;

  private final double RPM_TO_RPS = 1.0 / 60.0;
  private static final double CURRENT_LIMIT = 60.0; // Amps

  private double targetPos; // target position of the pivot motor in rotations

  private boolean isIntaking;

  /** Creates a new Intake. */
  public IntakeSubsystem() {
    pivotMotor = Devices.intakePivot;
    intakeControl = Devices.intakeControl;
    intakeFollower = Devices.intakeFollower;

    TalonFXConfiguration intakeControlConfigs = Configs.getIntakeControlConfigs();

    intakeControl.getConfigurator().apply(intakeControlConfigs);

    intakeFollower.setControl(new Follower(intakeControl.getDeviceID(), MotorAlignmentValue.Opposed));

    TalonFXConfiguration intakePivotConfigs = Configs.getIntakePivotConfigs();

    pivotMotor.getConfigurator().apply(intakePivotConfigs);

    targetPos = PIVOT_MAX_ROT;

    setPivotZero();
    pivotMotor.setControl(pivotRequest.withPosition(targetPos));
  }

  private void updateIntakePos() {

    MotionMagicVoltage m_request = new MotionMagicVoltage(targetPos);

    pivotMotor.setControl(m_request);
  }

  private void intakeChangeBy(double deltaPos) {
    targetPos += deltaPos;

    updateIntakePos();
  }

  private void setPositionIntake(double pos) {
    targetPos = pos;

    updateIntakePos();
  }

  public void setPivotZero() {
    pivotMotor.setPosition(0.0);
    targetPos = 0;
    updateIntakePos();
  }

  public void stop() {
    intakeControl.stopMotor();
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

  public void rotate(double targetRPM) {
    intakeControl.setControl(velocityRequest.withVelocity(targetRPM * RPM_TO_RPS));
  }

  // public Command deltaPivotCommand(double delta) {
  // return new InstantCommand(() -> intakeChangeBy(delta), this);
  // }

  public Command setPositionCommand(double pos) {
    return new InstantCommand(() -> setPositionIntake(pos), this);
  }

  public Command addPositionCommand(double pos) {
    return new InstantCommand(() -> intakeChangeBy(pos), this);
  }

  // public Command zeroPivotCommand() {
  // return new InstantCommand(() -> setZero(), this);
  // }

  public Command raiseManual() {
    return new InstantCommand(() -> pivotMotor.set(-0.2));
  }

  public Command lowerManual() {
    return new InstantCommand(() -> pivotMotor.set(0.2));
  }

  public Command stopPivot() {
    return new InstantCommand(() -> pivotMotor.set(0));
  }

  public void setPivotPositionRotations(double positionRotations) {
    double clamped = MathUtil.clamp(positionRotations, PIVOT_MIN_ROT, PIVOT_MAX_ROT);
    if (Math.abs(clamped - targetPos) < PIVOT_CMD_EPSILON_ROT) {
      return;
    }

    targetPos = clamped;
    pivotMotor.setControl(pivotRequest.withPosition(targetPos));
  }

  /**
   * Sets intake pivot with a normalized input where 0 = fully lowered, 1 = fully
   * raised.
   */
  public void setPivotNormalized(double normalizedPosition) {
    double normalized = MathUtil.clamp(normalizedPosition, 0.0, 1.0);
    double positionRot = MathUtil.interpolate(PIVOT_MIN_ROT, PIVOT_MAX_ROT, normalized);
    setPivotPositionRotations(positionRot);
  }

  public Command intakeCommand() {
    return new FunctionalCommand(
        () -> {
          isIntaking = true;
        },
        () -> {
          rotate(getTargetRPM());
        },
        interrupted -> {
          stop();
          isIntaking = false;
        },
        () -> false,
        this);
  }

  public Command purgeCommand() {
    return new FunctionalCommand(
        () -> {
          isIntaking = false;
        },
        () -> {
          rotate(purgeRPM);
        },
        interrupted -> {
          stop();
        },
        () -> false,
        this);
  }

  public Command fluctuatingIntakeCommand() {
    Timer timer = new Timer();

    return new FunctionalCommand(
        // 1. Initialize: Start the timer when the command begins
        () -> {
          timer.restart();
        },
        // 2. Execute: Toggle motor power based on the timer
        () -> {
          // Pulse logic: 0.4s ON, 0.2s OFF (Total 0.6s cycle)
          if ((timer.get() % 0.5) < 0.25) {
            setPivotNormalized(1.0);
          } else {
            setPivotNormalized(0);
          }
        },
        // 3. End: Stop the motor when the command is interrupted/finished
        interrupted -> {
          setPivotNormalized(0);
        },
        // 4. isFinished: Return false so it runs until you release the button
        () -> false,
        // Add the subsystem requirement
        this);
  }

  private final Timer compatFluctuateTimer = new Timer();
  private boolean compatFluctuatingEnabled = false;

  public void fluctuatingIntakeOn() {
    compatFluctuatingEnabled = true;
    compatFluctuateTimer.restart();
  }

  public void fluctuatingIntakeOff() {
    compatFluctuatingEnabled = false;
    stop();
    setPivotNormalized(0);
  }

  public Command stopCommand() {
    return new InstantCommand(() -> {
      stop();
      isIntaking = false;
    }, this);
  }

  public Command setRPMCommand(double RPM) {
    return new InstantCommand(() -> setTargetRPM(RPM));
  }

  public Command changeTargetRPMCommand(double deltaRPM) {
    return new InstantCommand(() -> changeTargetRPM(deltaRPM));
  }

  public double getTargetRPM() {
    return targetRPM;
  }

  public double getIntakeSupplyCurrent() {
    return intakeControl.getSupplyCurrent().getValueAsDouble() +
        pivotMotor.getSupplyCurrent().getValueAsDouble();
  }

  public boolean isIntaking() {
    return isIntaking;
  }

  public double getSpeedRPM() {
    return intakeControl.getRotorVelocity().getValueAsDouble() * 60;
  }

  public boolean isPurging() {
    return getSpeedRPM() < 0;
  }

  @Override
  public void periodic() {
    if (compatFluctuatingEnabled) {
      rotate(getTargetRPM());
      if ((compatFluctuateTimer.get() % 1) < 0.5) {
        setPivotNormalized(0.4);
      } else {
        setPivotNormalized(0);
      }
    }
  }

  // private void rotateAtCached() {
  // if (isIntaking) {
  // isIntaking = false;
  // stop();
  // } else {
  // isIntaking = true;
  // rotate(targetRPM);
  // }
  // }
}
