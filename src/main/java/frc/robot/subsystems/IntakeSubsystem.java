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

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX intakeControl;
  private final TalonFX pivotMotor;
  private final TalonFX intakeFollower;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0).withSlot(0);
  private boolean raised;
  private boolean lowered;

  // Pivot motion limits in mechanism rotations (motor sensor rotations).
  // Tune these based on your zeroing process and physical hard stops.
  private static final double PIVOT_MIN_ROT = 0.0; // lowered
  private static final double PIVOT_MAX_ROT = 0.38; // raised
  private static final double PIVOT_CMD_EPSILON_ROT = 0.002;

  private double MAX_RPM = 6300;
  private double targetRPM = 4000;

  private double purgeRPM = -2000;
  private double purgeTime = 0.1; // seconds

  private final double RPM_TO_RPS = 1.0 / 60.0;
  private static final double CURRENT_LIMIT = 60.0; // Amps

  private double targetPos; // target position of the pivot motor in rotations

  private boolean isIntaking;
  private boolean isPurging;

  /** Creates a new Intake. */
  public IntakeSubsystem() {
    pivotMotor = new TalonFX(30);
    intakeControl = new TalonFX(31);
    intakeFollower = new TalonFX(37);

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

    intakeControl.getConfigurator().apply(controlCfg);

    intakeFollower.setControl(new Follower(intakeControl.getDeviceID(), MotorAlignmentValue.Opposed));

    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;

    slot0Configs.kG = 0.2; // Output of voltage to overcome gravity
    slot0Configs.kV = 2; // Output per unit target velocity, perhaps not needed
    slot0Configs.kA = 0.3; // Output per unit target acceleration, perhaps not needed
    slot0Configs.kP = 15; // Controls the response to position error—how much the motor reacts to the
                          // difference between the current position and the target position.
    slot0Configs.kI = 1.5; // Addresses steady-state error, which occurs when the motor doesn’t quite reach
    // the target position due to forces like gravity or friction.
    slot0Configs.kD = 0.3; // Responds to the rate of change of the error, damping the motion as the motor
                           // approaches the target. This reduces overshooting and oscillations.

    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target velocity in rps
    motionMagicConfigs.MotionMagicAcceleration = 68; // Target acceleration in rps/s
    motionMagicConfigs.MotionMagicJerk = 400; // Target jerk in rps/s/s

    pivotMotor.getConfigurator().apply(talonFXConfigs);
    pivotMotor.getConfigurator().apply(slot0Configs);
    pivotMotor.getConfigurator().apply(motionMagicConfigs);

    //setZero();
    pivotMotor.setPosition(PIVOT_MIN_ROT);
    targetPos = PIVOT_MIN_ROT;
    pivotMotor.setControl(pivotRequest.withPosition(targetPos));

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

  private boolean isPurgeDone() {
   return purgeClock > (purgeTime / 0.02);
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

    lowered = targetPos <= (PIVOT_MIN_ROT + 0.01);
    raised = targetPos >= (PIVOT_MAX_ROT - 0.01);
  }

  /**
   * Sets intake pivot with a normalized input where 0 = fully lowered, 1 = fully raised.
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

  private double purgeClock = 0;

  public Command purgeCommand() {
    return new FunctionalCommand(
      () -> {
        isPurging = true;
        isIntaking = false;
        purgeClock = 0;
      },
      () -> {
        rotate(purgeRPM);
      },
      interrupted -> {
        stop();
        isPurging = false;
      },
      this::isPurgeDone,
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
            pivotMotor.set(-0.1);
          } else {
            pivotMotor.set(0.1);
          }
        },
        // 3. End: Stop the motor when the command is interrupted/finished
        interrupted -> {
          pivotMotor.stopMotor();
        },
        // 4. isFinished: Return false so it runs until you release the button
        () -> false,
        // Add the subsystem requirement
        this);
  }

  public Command stopCommand() {
    return new InstantCommand(() -> {
      stop();
      isIntaking = false;
      isPurging = false;
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
    if (isPurging) {
      purgeClock++;
    }
    else {
      purgeClock = 0;
    }
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
