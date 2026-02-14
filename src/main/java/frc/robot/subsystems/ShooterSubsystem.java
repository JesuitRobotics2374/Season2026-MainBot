
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX control;
    private final TalonFX follower;
    private final TalonFX kicker;
    
    // Request object to avoid allocation in loops
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

    private double targetRpm = 3000.0;
    private double targetKickerRpm = 500.0;

    private boolean isKicking = false;
    private boolean isShooting = false;
    
    // Constants
    private static final double MAX_RPM = 6000.0; 
    private static final double RPM_TO_RPS = 1.0 / 60.0;
    private static final double CURRENT_LIMIT = 40.0; // Amps

    public ShooterSubsystem() {

        control = new TalonFX(11);
        follower = new TalonFX(12);

        kicker = new TalonFX(13);

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
    }

     /**
     * Runs the motor at the specified RPM using closed-loop control.
     * @param rpm Target RPM
     */
    private void rotate(double rpm) {
        // Convert RPM to RPS
        control.setControl(velocityRequest.withVelocity(rpm * RPM_TO_RPS));
    }

    public void rotateKicker() {
      if (isKicking) {
        isKicking = false;
        kicker.stopMotor();
      }
      else {
        isKicking = true;
        kicker.setControl(velocityRequest.withVelocity(targetKickerRpm * RPM_TO_RPS));
      }
    }
    

      /**
     * Runs the motor at the specified RPM using closed-loop control.
     * @param rpm Target RPM
     */
    public void rotateAtCached() {
        // Convert RPM to RPS
        if (isShooting) {
          isShooting = false;
          stop();
        }
        else {
          isShooting = true;
          rotate(targetRpm);
        }
    }

    private void stop() {
      control.stopMotor();
    }

    private void setTargetRpm(double rpm) {
        if (rpm > MAX_RPM) rpm = MAX_RPM;
        if (rpm < -MAX_RPM) rpm = -MAX_RPM;
        targetRpm = rpm;
    }

    public void changeTargetRpm(double deltaRpm) {
        setTargetRpm(targetRpm + deltaRpm);
    }

    public boolean getShooting() {
      return isShooting;
    }
    public boolean getKicking() {
      return isKicking;
    }

    /**
     * @return Current velocity in RPM
     */
    public double getSpeedRpm() {
      return control.getRotorVelocity().getValueAsDouble() * 60.0;
    }

    public double getTargetRpm() {
      return targetRpm;
    }

    /**
     * @return Current velocity in RPM
     */
    public double getKickerSpeedRpm() {
      return kicker.getRotorVelocity().getValueAsDouble() * 60.0;
    }

    public double getKickerTargetRpm() {
      return targetKickerRpm;
    }

    @Override
    public void periodic() {
      rotate(targetRpm);
    }
}