package frc.robot.utils.aiming;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.utils.Constants;

public final class AimingUtil {
    private static final double BEYOND_HUB_X_MARGIN_METERS = 1.75;

    private AimingUtil() {
    }

    public static boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }

    public static Translation2d getShooterFieldPosition(Pose2d robotPose) {
        Translation2d shooterOffset = new Translation2d(
                Constants.CENTER_TO_SHOOTER_X,
                Constants.CENTER_TO_SHOOTER_Y);
        return robotPose.getTranslation().plus(shooterOffset.rotateBy(robotPose.getRotation()));
    }

    public static Translation2d getTargetTranslation(Pose2d robotPose) {
        return getTargetTranslation(robotPose, isRedAlliance());
    }

    public static Translation2d getHubTargetTranslation() {
        return getHubTargetTranslation(isRedAlliance());
    }

    public static Translation2d getHubTargetTranslation(boolean isRedAlliance) {
        return isRedAlliance
                ? new Translation2d(Constants.HUB_RED_X, Constants.HUB_Y)
                : new Translation2d(Constants.HUB_BLUE_X, Constants.HUB_Y);
    }

    public static Translation2d getTargetTranslation(Pose2d robotPose, boolean isRedAlliance) {
        Translation2d hubTarget = getHubTargetTranslation(isRedAlliance);

        Translation2d shooterPosition = getShooterFieldPosition(robotPose);

        boolean beyondHub = isRedAlliance
                ? shooterPosition.getX() < Constants.HUB_RED_X - BEYOND_HUB_X_MARGIN_METERS
                : shooterPosition.getX() > Constants.HUB_BLUE_X + BEYOND_HUB_X_MARGIN_METERS;

        if (!beyondHub) {
            return hubTarget;
        }

        Translation2d nearCorner = isRedAlliance
                ? Constants.RED_SIDE_CORNER_NEAR
                : Constants.BLUE_SIDE_CORNER_NEAR;
        Translation2d farCorner = isRedAlliance
                ? Constants.RED_SIDE_CORNER_FAR
                : Constants.BLUE_SIDE_CORNER_FAR;

        double nearDistance = shooterPosition.getDistance(nearCorner);
        double farDistance = shooterPosition.getDistance(farCorner);

        return nearDistance <= farDistance ? nearCorner : farCorner;
    }

    public static double getShooterDistanceToTarget(Pose2d robotPose) {
        Translation2d shooterPosition = getShooterFieldPosition(robotPose);
        Translation2d targetPosition = getTargetTranslation(robotPose);
        return shooterPosition.getDistance(targetPosition);
    }
}
