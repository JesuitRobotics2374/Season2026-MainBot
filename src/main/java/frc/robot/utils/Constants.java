package frc.robot.utils;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
    // Vision constants
    public static int numberOfCams = 1;
    public static final AprilTagFields FIELD_LAYOUT = AprilTagFields.k2026RebuiltWelded;

    public static final double CENTER_TO_SHOOTER_X = 0.18;
    public static final double CENTER_TO_SHOOTER_Y = 0.22;

    // Speaker / hub constants
    public static final double HUB_RED_X = 11.915394;
    public static final double HUB_BLUE_X = 4.625594;
    public static final double HUB_Y = 4.034536;

    // Placeholder corner targets for "beyond hub" shooting fallback.
    // Replace with tuned/scouted values once tested on-field.
    public static final Translation2d BLUE_SIDE_CORNER_NEAR = new Translation2d(1.20, 1.00);
    public static final Translation2d BLUE_SIDE_CORNER_FAR = new Translation2d(1.20, 7.00);
    public static final Translation2d RED_SIDE_CORNER_NEAR = new Translation2d(15.30, 1.00);
    public static final Translation2d RED_SIDE_CORNER_FAR = new Translation2d(15.30, 7.00);

    // Real world release angles
    public static final double HOOD_ZERO_ANGLE = Math.toRadians(63.7);
    public static final double HOOD_LOWEST_ANGLE = Math.toRadians(26);

    // Motor-recognized values
    public static final double HOOD_MIN_SETPOINT = 0.0;
    public static final double HOOD_MAX_SETPOINT = 0.20;
    
    public static final double AUTO_X_OFFSET = 0;
    public static final double AUTO_ROLL_OFFSET = 0;
    public static final double AUTO_Y_OFFSET = 0;
    public static final double AUTO_PITCH_OFFSET = 0;
    public static final double AUTO_Z_OFFSET = 0;
    public static final double AUTO_YAW_OFFSET = 0;

    public static final double DEFAULT_DRIVE_CURRENT = 40;
    public static final double DEFAULT_STEER_CURRENT = 30;
}