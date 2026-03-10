package frc.robot.utils.aiming;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

public class SotmTelemetry {
    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("SOTM");

    private final StructPublisher<Pose3d> shooterPosePub = table.getStructTopic("ShooterPose3d", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> lookaheadPosePub = table.getStructTopic("LookaheadShooterPose3d", Pose3d.struct)
            .publish();
    private final StructPublisher<Pose3d> targetPosePub = table.getStructTopic("TargetPose3d", Pose3d.struct).publish();

    private final StructPublisher<Translation3d> robotVelPub = table
            .getStructTopic("RobotFieldVelocityMps", Translation3d.struct).publish();
    private final StructPublisher<Translation3d> shooterVelPub = table
            .getStructTopic("ShooterFieldVelocityMps", Translation3d.struct).publish();
    private final StructPublisher<Translation3d> compensationPub = table
            .getStructTopic("CompensationVectorM", Translation3d.struct).publish();
    private final StructPublisher<Translation3d> shooterToTargetPub = table
            .getStructTopic("ShooterToTargetVectorM", Translation3d.struct).publish();
    private final StructPublisher<Translation3d> lookaheadToTargetPub = table
            .getStructTopic("LookaheadToTargetVectorM", Translation3d.struct).publish();

    private final StructArrayPublisher<Pose3d> shooterToTargetLinePub = table
            .getStructArrayTopic("ShooterToTargetLine3d", Pose3d.struct).publish();
    private final StructArrayPublisher<Pose3d> lookaheadToTargetLinePub = table
            .getStructArrayTopic("LookaheadToTargetLine3d", Pose3d.struct).publish();
    private final StructArrayPublisher<Pose3d> compensationLinePub = table
            .getStructArrayTopic("CompensationLine3d", Pose3d.struct).publish();

    private final DoublePublisher distancePub = table.getDoubleTopic("DistanceM").publish();
    private final DoublePublisher distanceRawPub = table.getDoubleTopic("DistanceNoLookaheadM").publish();
    private final DoublePublisher tofPub = table.getDoubleTopic("TimeOfFlightS").publish();
    private final DoublePublisher driveAnglePub = table.getDoubleTopic("DriveAngleRad").publish();
    private final DoublePublisher driveVelPub = table.getDoubleTopic("DriveOmegaRadPerSec").publish();
    private final DoublePublisher hoodAnglePub = table.getDoubleTopic("HoodAngleRad").publish();
    private final DoublePublisher hoodVelPub = table.getDoubleTopic("HoodOmegaRadPerSec").publish();
    private final DoublePublisher flywheelPub = table.getDoubleTopic("FlywheelRPM").publish();

    private final BooleanPublisher enabledPub = table.getBooleanTopic("Enabled").publish();
    private final BooleanPublisher validPub = table.getBooleanTopic("IsValid").publish();
    private final BooleanPublisher passingPub = table.getBooleanTopic("Passing").publish();
    private final BooleanPublisher atDriveGoalPub = table.getBooleanTopic("AtDriveGoal").publish();

    public void publish(LaunchingParameters params, boolean enabled, boolean atDriveGoal) {
        enabledPub.set(enabled);
        validPub.set(params.isValid());
        passingPub.set(params.passing());
        atDriveGoalPub.set(atDriveGoal);

        shooterPosePub.set(params.shooterPose());
        lookaheadPosePub.set(params.lookaheadShooterPose());
        targetPosePub.set(params.targetPose());

        robotVelPub.set(params.robotFieldVelocity());
        shooterVelPub.set(params.shooterFieldVelocity());
        compensationPub.set(params.compensationVector());
        shooterToTargetPub.set(params.shooterToTargetVector());
        lookaheadToTargetPub.set(params.lookaheadToTargetVector());

        // Publish explicit start->end line segments so field visualization has both endpoints.
        shooterToTargetLinePub.set(new Pose3d[] { params.shooterPose(), params.targetPose() });
        lookaheadToTargetLinePub.set(new Pose3d[] { params.lookaheadShooterPose(), params.targetPose() });
        compensationLinePub.set(new Pose3d[] { params.shooterPose(), params.lookaheadShooterPose() });

        distancePub.set(params.distance());
        distanceRawPub.set(params.distanceNoLookahead());
        tofPub.set(params.timeOfFlight());
        driveAnglePub.set(params.driveAngle().getRadians());
        driveVelPub.set(params.driveVelocity());
        hoodAnglePub.set(params.hoodAngle());
        hoodVelPub.set(params.hoodVelocity());
        flywheelPub.set(params.flywheelSpeed());
    }
}
