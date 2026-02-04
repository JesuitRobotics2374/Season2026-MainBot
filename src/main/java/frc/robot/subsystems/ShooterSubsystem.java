
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Ballistics;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX control;
  private final TalonFX follower;

  // Request object to avoid allocation in loops
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  private final Translation2d absoluteHubTranslation;

  private double targetRpm = 0.0;

  private boolean doAutoShoot;

  private Pose2d globalFieldPose;

  private ChassisSpeeds speeds;

  // Constants
  private static final double MAX_RPM = 6000.0;
  private static final double RPM_TO_RPS = 1.0 / 60.0;
  private static final double CURRENT_LIMIT = 40.0; // Amps

  public ShooterSubsystem(boolean isRed) {

    control = new TalonFX(11);
    follower = new TalonFX(12);

    TalonFXConfiguration controlCfg = new TalonFXConfiguration();
    controlCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    controlCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Current Limits
    controlCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    controlCfg.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;

    controlCfg.Slot0.kP = 0.11;
    controlCfg.Slot0.kI = 0.5;
    controlCfg.Slot0.kD = 0.0001;
    controlCfg.Slot0.kV = 0.12; // ~12V

    control.getConfigurator().apply(controlCfg);

    TalonFXConfiguration followerCfg = new TalonFXConfiguration();
    followerCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // Apply same current limits to follower
    followerCfg.CurrentLimits = controlCfg.CurrentLimits;

    follower.getConfigurator().apply(followerCfg);

    follower.setControl(new Follower(control.getDeviceID(), MotorAlignmentValue.Opposed));

    control.setNeutralMode(NeutralModeValue.Brake);

    doAutoShoot = false;

    if (isRed) {
      absoluteHubTranslation = new Translation2d(); // TODO
    } else {
      absoluteHubTranslation = new Translation2d(4.625594, 4.034536);
    }
  }

  /**
   * Runs the motor at the specified RPM using closed-loop control.
   * 
   * @param rpm Target RPM
   */
  private void rotate(double rpm) {
    // Convert RPM to RPS
    control.setControl(velocityRequest.withVelocity(rpm * RPM_TO_RPS));
  }

  private void stopMotor() {
    control.stopMotor();
    setTargetRpm(0);
  }

  private void setTargetRpm(double rpm) {
    if (rpm > MAX_RPM)
      rpm = MAX_RPM;
    if (rpm < -MAX_RPM)
      rpm = -MAX_RPM;
    targetRpm = rpm;
  }

  private void changeTargetRpm(double deltaRpm) {
    setTargetRpm(targetRpm + deltaRpm);
  }

  private void setAutoShoot(boolean active) {
    doAutoShoot = active;
  }

  public Command stop() {
    return new InstantCommand(() -> stopMotor());
  }

  public Command setTargetRPM(double rpm) {
    return new InstantCommand(() -> setTargetRpm(rpm));
  }

  public Command changeTargetRPM(double deltaRpm) {
    return new InstantCommand(() -> changeTargetRpm(deltaRpm));
  }

  public Command enableAutoShoot() {
    return new InstantCommand(() -> {
      setAutoShoot(true);
      setTargetRPM(0);
    });
  }

  public Command disableAutoShoot(double newTargetRPM) {
    return new InstantCommand(() -> {
      setAutoShoot(false);
      setTargetRPM(newTargetRPM);
    });
  }

  public double getTargetRpm() {
    return targetRpm;
  }

  /**
   * @return Current velocity in RPM
   */
  public double getSpeedRpm() {
    return control.getRotorVelocity().getValueAsDouble() * 60.0;
  }

  public void passRobotPose(Pose2d globalFieldPose) {
    this.globalFieldPose = globalFieldPose;
  }

  public void passChassisSpeeds(ChassisSpeeds speeds) {
    this.speeds = speeds;
  }

  @Override
  public void periodic() {
    double useRPM = targetRpm;

    if (doAutoShoot && globalFieldPose != null && speeds != null) {

      double neededAngle = Math.atan2(
          absoluteHubTranslation.getY() - globalFieldPose.getY(),
          absoluteHubTranslation.getX() - globalFieldPose.getX());

      double robotYaw = globalFieldPose.getRotation().getRadians();

      double yawError = neededAngle - robotYaw;
      yawError = Math.atan2(Math.sin(yawError), Math.cos(yawError));

      boolean facingHub = Math.abs(yawError) < Math.toRadians(2.5);

      if (facingHub) {
        double neededX = Math.abs(absoluteHubTranslation.getY() - globalFieldPose.getY()); // this may need to be translation.getDistance() not sure, needs testing

        double neededVel = Ballistics.CalculateNeededShooterSpeed(
            neededX,
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond);

        // useRPM = convertVelToRPM(neededVel);
      } else {
        // Not facing hub → disable shooting
        useRPM = 0.0;
      }
    }

    rotate(useRPM);
  }

}