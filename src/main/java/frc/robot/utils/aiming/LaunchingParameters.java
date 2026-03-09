package frc.robot.utils.aiming;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

public record LaunchingParameters(
        boolean isValid,
        Rotation2d driveAngle,
        double driveVelocity,
        double hoodAngle,
        double hoodVelocity,
        double flywheelSpeed,
        double distance,
        double distanceNoLookahead,
        double timeOfFlight,
        boolean passing,
        Pose3d shooterPose,
        Pose3d lookaheadShooterPose,
        Pose3d targetPose,
        Translation3d robotFieldVelocity,
        Translation3d shooterFieldVelocity,
        Translation3d compensationVector,
        Translation3d shooterToTargetVector,
        Translation3d lookaheadToTargetVector) {
}
