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
    public static final Translation2d BLUE_SIDE_CORNER_NEAR = new Translation2d(1.00, 2.00);
    public static final Translation2d BLUE_SIDE_CORNER_FAR = new Translation2d(1.00, 6.00);
    public static final Translation2d RED_SIDE_CORNER_NEAR = new Translation2d(15.54, 2.00);
    public static final Translation2d RED_SIDE_CORNER_FAR = new Translation2d(15.54, 6.00);

    // Real world release angles
    public static final double HOOD_ZERO_ANGLE = Math.toRadians(63.7);
    public static final double HOOD_LOWEST_ANGLE = Math.toRadians(26);

    // Motor-recognized values
    public static final double HOOD_MIN_SETPOINT = 0.0;
    public static final double HOOD_MAX_SETPOINT = 0.20;

    // Shooter constants
    public static final double DEFAULT_KICKER_RPM = 2500.0;
        public static final double[][] SHOOTER_LOOKUP_TABLE = {
            { 1.90, 2300, 0 },
            { 2.25, 2500, 0 },
            { 2.50, 2600, 0 },
            { 3.00, 2800, 0 },
            { 3.50, 3100, 0 },
            { 4.00, 3300, 0 },
            { 4.50, 3500, 0 } };

        // Shoot-on-the-move (SOTM) constants
        public static final boolean ENABLE_SHOOT_ON_MOVE = true;
        public static final double SOTM_PHASE_DELAY_SECONDS = 0.03;
        public static final double SOTM_VELOCITY_BLEND_SETPOINT_WEIGHT = 0.5;
        public static final int SOTM_LOOKAHEAD_ITERATIONS = 20;
        public static final double SOTM_MIN_DISTANCE_METERS = 1.2;
        public static final double SOTM_MAX_DISTANCE_METERS = 6.0;
        public static final double SOTM_DRIVE_YAW_TOLERANCE_RAD = Math.toRadians(5.0);
        public static final double SOTM_DRIVE_KP = 12.0;
        public static final double SOTM_DRIVE_KD = 2.5;

        // Distance (m), TOF (s)
        public static final double[][] SOTM_TOF_TABLE = {
            { 2.5, 1.05 },
            { 4.0, 1.35 } };
    
    public static final double AUTO_X_OFFSET = 0;
    public static final double AUTO_ROLL_OFFSET = 0;
    public static final double AUTO_Y_OFFSET = 0;
    public static final double AUTO_PITCH_OFFSET = 0;
    public static final double AUTO_Z_OFFSET = 0;
    public static final double AUTO_YAW_OFFSET = 0;

    public static final double DEFAULT_DRIVE_CURRENT = 100;
    public static final double DEFAULT_STEER_CURRENT = 30;
}