package frc.robot.utils.aiming;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.utils.Constants;

public class LaunchCalculator {
    private final DriveSubsystem drivetrain;
    private final ShooterLookupTable shooterLookupTable;
    private final InterpolatingDoubleTreeMap tofMap = new InterpolatingDoubleTreeMap();

    private final LinearFilter hoodVelocityFilter = LinearFilter.movingAverage(5);
    private final LinearFilter driveVelocityFilter = LinearFilter.movingAverage(5);

    private LaunchingParameters cachedParameters;
    private double lastHoodAngle = Double.NaN;
    private Rotation2d lastDriveAngle = null;

    public LaunchCalculator(DriveSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        this.shooterLookupTable = new ShooterLookupTable(Constants.SHOOTER_LOOKUP_TABLE);

        for (double[] row : Constants.SOTM_TOF_TABLE) {
            tofMap.put(row[0], row[1]);
        }
    }

    public LaunchingParameters getParameters() {
        if (cachedParameters != null) {
            return cachedParameters;
        }

        Pose2d estimatedPose = drivetrain.getEstimator();
        ChassisSpeeds measuredRobotRelativeVelocity = drivetrain.getCurrentRobotChassisSpeeds();

        ChassisSpeeds setpointRobotRelativeVelocity = drivetrain.getCommandedRobotChassisSpeeds();
        double setpointWeight = MathUtil.clamp(Constants.SOTM_VELOCITY_BLEND_SETPOINT_WEIGHT, 0.0, 1.0);
        ChassisSpeeds blendedRobotRelativeVelocity = new ChassisSpeeds(
                measuredRobotRelativeVelocity.vxMetersPerSecond * (1.0 - setpointWeight)
                        + setpointRobotRelativeVelocity.vxMetersPerSecond * setpointWeight,
                measuredRobotRelativeVelocity.vyMetersPerSecond * (1.0 - setpointWeight)
                        + setpointRobotRelativeVelocity.vyMetersPerSecond * setpointWeight,
                measuredRobotRelativeVelocity.omegaRadiansPerSecond * (1.0 - setpointWeight)
                        + setpointRobotRelativeVelocity.omegaRadiansPerSecond * setpointWeight);

        estimatedPose = estimatedPose.exp(new Twist2d(
                blendedRobotRelativeVelocity.vxMetersPerSecond * Constants.SOTM_PHASE_DELAY_SECONDS,
                blendedRobotRelativeVelocity.vyMetersPerSecond * Constants.SOTM_PHASE_DELAY_SECONDS,
                blendedRobotRelativeVelocity.omegaRadiansPerSecond * Constants.SOTM_PHASE_DELAY_SECONDS));

        Translation2d target = AimingUtil.getTargetTranslation(estimatedPose);

        Translation2d shooterPosition = AimingUtil.getShooterFieldPosition(estimatedPose);
        Translation2d lookaheadShooterPosition = shooterPosition;
        Translation2d lookaheadRobotPosition = estimatedPose.getTranslation();
        double distanceNoLookahead = target.getDistance(shooterPosition);
        double lookaheadDistance = distanceNoLookahead;
        double timeOfFlight = tofMap.get(lookaheadDistance);

        ChassisSpeeds blendedFieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(
                blendedRobotRelativeVelocity,
                estimatedPose.getRotation());

        for (int i = 0; i < Constants.SOTM_LOOKAHEAD_ITERATIONS; i++) {
            timeOfFlight = tofMap.get(lookaheadDistance);
            double offsetX = blendedFieldVelocity.vxMetersPerSecond * timeOfFlight;
            double offsetY = blendedFieldVelocity.vyMetersPerSecond * timeOfFlight;
            Translation2d offset = new Translation2d(offsetX, offsetY);
            lookaheadShooterPosition = shooterPosition.plus(offset);
            lookaheadRobotPosition = estimatedPose.getTranslation().plus(offset);
            lookaheadDistance = target.getDistance(lookaheadShooterPosition);
        }

        Pose2d lookaheadRobotPose = new Pose2d(lookaheadRobotPosition, estimatedPose.getRotation());
        Rotation2d driveAngle = getDriveAngleWithShooterOffset(lookaheadRobotPose, target);

        ShooterLookupTable.ShotSetpoint setpoint = shooterLookupTable.sample(lookaheadDistance);
        double hoodPercent = MathUtil.clamp(setpoint.hoodPercent(), 0.0, 1.0);
        double hoodAngle = MathUtil.interpolate(Constants.HOOD_ZERO_ANGLE, Constants.HOOD_LOWEST_ANGLE, hoodPercent);

        if (Double.isNaN(lastHoodAngle)) {
            lastHoodAngle = hoodAngle;
        }
        if (lastDriveAngle == null) {
            lastDriveAngle = driveAngle;
        }

        double hoodVelocity = hoodVelocityFilter.calculate((hoodAngle - lastHoodAngle) / 0.02);
        double driveVelocity = driveVelocityFilter.calculate(
                driveAngle.minus(lastDriveAngle).getRadians() / 0.02);

        lastHoodAngle = hoodAngle;
        lastDriveAngle = driveAngle;

        boolean passing = isPassingTarget(target);
        boolean isValid = lookaheadDistance >= Constants.SOTM_MIN_DISTANCE_METERS
                && lookaheadDistance <= Constants.SOTM_MAX_DISTANCE_METERS;

        Pose3d shooterPose3d = new Pose3d(
                shooterPosition.getX(),
                shooterPosition.getY(),
                0.0,
                new Rotation3d(0.0, 0.0, estimatedPose.getRotation().getRadians()));
        Pose3d lookaheadShooterPose3d = new Pose3d(
                lookaheadShooterPosition.getX(),
                lookaheadShooterPosition.getY(),
                0.0,
                new Rotation3d(0.0, 0.0, driveAngle.getRadians()));
        Pose3d targetPose3d = new Pose3d(target.getX(), target.getY(), 0.0, new Rotation3d());

        Translation3d robotFieldVelocity = new Translation3d(
                blendedFieldVelocity.vxMetersPerSecond,
                blendedFieldVelocity.vyMetersPerSecond,
                0.0);

        Translation3d compensationVector = new Translation3d(
                lookaheadShooterPosition.getX() - shooterPosition.getX(),
                lookaheadShooterPosition.getY() - shooterPosition.getY(),
                0.0);

        Translation3d shooterToTarget = new Translation3d(
                target.getX() - shooterPosition.getX(),
                target.getY() - shooterPosition.getY(),
                0.0);

        Translation3d lookaheadToTarget = new Translation3d(
                target.getX() - lookaheadShooterPosition.getX(),
                target.getY() - lookaheadShooterPosition.getY(),
                0.0);

        cachedParameters = new LaunchingParameters(
                isValid,
                driveAngle,
                driveVelocity,
                hoodAngle,
                hoodVelocity,
                setpoint.shooterRPM(),
                lookaheadDistance,
                distanceNoLookahead,
                timeOfFlight,
                passing,
                shooterPose3d,
                lookaheadShooterPose3d,
                targetPose3d,
                robotFieldVelocity,
                robotFieldVelocity,
                compensationVector,
                shooterToTarget,
                lookaheadToTarget);

        return cachedParameters;
    }

    public void clearCachedParameters() {
        cachedParameters = null;
    }

    public boolean atDriveGoal() {
        LaunchingParameters parameters = getParameters();
        double yawError = drivetrain.getEstimator().getRotation().minus(parameters.driveAngle()).getRadians();
        return Math.abs(yawError) <= Constants.SOTM_DRIVE_YAW_TOLERANCE_RAD;
    }

    private static Rotation2d getDriveAngleWithShooterOffset(Pose2d robotPose, Translation2d target) {
        Rotation2d fieldToTargetAngle = target.minus(robotPose.getTranslation()).getAngle();
        double robotToShooterY = Constants.CENTER_TO_SHOOTER_Y;
        double distance = target.getDistance(robotPose.getTranslation());
        Rotation2d offsetAngle = new Rotation2d(
                Math.asin(MathUtil.clamp(robotToShooterY / Math.max(distance, 1e-6), -1.0, 1.0)));
        return fieldToTargetAngle.plus(offsetAngle);
    }

    private boolean isPassingTarget(Translation2d target) {
        Translation2d hubTarget = AimingUtil.getHubTargetTranslation();
        return target.getDistance(hubTarget) > 1e-4;
    }
}
