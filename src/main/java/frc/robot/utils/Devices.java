package frc.robot.utils;

import com.ctre.phoenix6.hardware.TalonFX;

public class Devices {

    // Intake
    public static TalonFX intakeControl = new TalonFX(31);
    public static TalonFX intakeFollower = new TalonFX(37);
    public static TalonFX intakePivot = new TalonFX(30);

    // Hopper
    public static TalonFX hopperRoller = new TalonFX(32);

    // Shooter
    public static TalonFX kickerControl = new TalonFX(33);
    public static TalonFX shooterControl = new TalonFX(35);
    public static TalonFX shooterFollower = new TalonFX(36);
    public static TalonFX hoodControl = new TalonFX(34);
}
