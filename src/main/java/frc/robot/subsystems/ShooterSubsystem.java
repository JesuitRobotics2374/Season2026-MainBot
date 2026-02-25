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
    private final TalonFX hood;
    
    // Request object to avoid allocation in loops
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

    private double targetRpm = 0.0; 
    
    // Constants
    private static final double MAX_RPM = 6000.0; 
    private static final double RPM_TO_RPS = 1.0 / 60.0;
    private static final double CURRENT_LIMIT = 40.0; // Amps

    public ShooterSubsystem() {

        control = new TalonFX(11);
        follower = new TalonFX(12);
        hood = new TalonFX(13); // placeholder 

        TalonFXConfiguration controlCfg = new TalonFXConfiguration();
        TalonFXConfiguration hoodCfg = new TalonFXConfiguration();
        controlCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        controlCfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        // Current Limits
        controlCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        controlCfg.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;

        controlCfg.Slot0.kP = 0.11;
        controlCfg.Slot0.kI = 0.5;
        controlCfg.Slot0.kD = 0.0001;
        controlCfg.Slot0.kV = 0.12; // ~12V

        // for hood config i dont have any voltage limits set and also dont know what p i and d values should be
        hoodCfg.Slot0.kP = 10;
        hoodCfg.Slot0.kI = 0;
        hoodCfg.Slot0.kD = 0; 

        control.getConfigurator().apply(controlCfg);
        hood.getConfigurator().apply(hoodCfg);
        
        TalonFXConfiguration followerCfg = new TalonFXConfiguration();
        followerCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // Apply same current limits to follower
        followerCfg.CurrentLimits = controlCfg.CurrentLimits;
        
        follower.getConfigurator().apply(followerCfg);

        follower.setControl(new Follower(control.getDeviceID(), MotorAlignmentValue.Opposed));

        control.setNeutralMode(NeutralModeValue.Brake);
        hood.setNeutralMode(NeutralModeValue.Brake); 
    }

     /**
     * Runs the motor at the specified RPM using closed-loop control.
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
        if (rpm > MAX_RPM) rpm = MAX_RPM;
        if (rpm < -MAX_RPM) rpm = -MAX_RPM;
        targetRpm = rpm;
    }

    private void changeTargetRpm(double deltaRpm) {
        setTargetRpm(targetRpm + deltaRpm);
    }

    
    private void hoodStart() { // should run on startup, idk if necessary
      hood.setPosition(0);
    }

    private void incrementHood(double rotations){
      motor.set(ControlMode.Position, rotations);
    }
    /**
     * @param target in rotations
     */
    private void setHoodPos(double target){
      double currentPos = hood.getPosition().getValue();
      double targetPos = currentPos + target;
      while (currentPos < targetPos){
        incrementHood(0.01); // adjust for overshoot if necessary
        currentPos += 0.01; 
      }
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

    public Command hoodUp(double rotations){
      return new InstantCommand(() -> incrementHood(0.1));
    }

    public Command hoodDown(double rotations){
      return new InstantCommand(() -> incrementHood(-0.1));
    }

    public Command setHood(double pos){
      return new Instantcommand(() -> setHoodPos(pos));
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

    @Override
    public void periodic() {
      rotate(targetRpm);
    }
}