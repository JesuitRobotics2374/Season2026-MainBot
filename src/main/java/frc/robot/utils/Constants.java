package frc.robot.utils;

import edu.wpi.first.apriltag.AprilTagFields;

public class Constants {
    // Vision constants
    public static int numberOfCams = 1;
    public static final AprilTagFields FIELD_LAYOUT = AprilTagFields.k2026RebuiltWelded;

    public static final double CENTER_TO_SHOOTER_X = 0.41275;
    public static final double CENTER_TO_SHOOTER_Y = 0.19;
    public static final double HOOD_ZERO_ANGLE = 63.7;
    
    public static final double AUTO_X_OFFSET = 0;
    public static final double AUTO_ROLL_OFFSET = 0;
    public static final double AUTO_Y_OFFSET = 0;
    public static final double AUTO_PITCH_OFFSET = 0;
    public static final double AUTO_Z_OFFSET = 0;
    public static final double AUTO_YAW_OFFSET = 0;

    public static final double DEFAULT_DRIVE_CURRENT = 40;
    public static final double DEFAULT_STEER_CURRENT = 30;
}