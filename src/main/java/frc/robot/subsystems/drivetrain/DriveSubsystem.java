package frc.robot.subsystems.drivetrain;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.drivetrain.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.vision.PoseEstimateValues;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class DriveSubsystem extends TunerSwerveDrivetrain implements Subsystem {

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    // CUSTOM DECLARATIONS

    private final SwerveDrivePoseEstimator estimator;
    private List<PoseEstimateValues> estimates = new ArrayList<>();

    Field2d field = new Field2d();

    private RobotConfig config;
    private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();

    private double timeSinceLastEstimatorUpdate;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public DriveSubsystem(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);

        estimator = new SwerveDrivePoseEstimator(getKinematics(), getGyroscopeRotation(), getSwerveModulePositions(),
                new Pose2d());

        SmartDashboard.putData("Field", field);

        try {
            config = RobotConfig.fromGUISettings(); // TODO UPDATE BEFORE COMP
        } catch (Exception e) {
            System.out.println("Failed to initialize robot configs!");
        }

        AutoBuilder.configure(
                this::getEstimator, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getCurrentRobotChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> this.setControl(autoRequest.withSpeeds(speeds)), // Method that will drive the
                                                                                           // robot given ROBOT RELATIVE
                                                                                           // ChassisSpeeds. Also
                                                                                           // optionally outputs
                                                                                           // individual module
                                                                                           // feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                                // holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

        System.out.println("DriveSubsystem constructed");
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }

        estimator.update(getGyroscopeRotation(), getSwerveModulePositions());

        timeSinceLastEstimatorUpdate = Utils.getCurrentTimeSeconds();

        PoseEstimateValues bestEstimate = null;
        double lowestScore = Double.MAX_VALUE;

        for (PoseEstimateValues estimate : estimates) {
            if (estimate == null) {
                continue;
            }
            
            Matrix<N3, N1> s = estimate.standardDeviations;

            double score = s.get(0, 0) * s.get(0, 0) + // x
                    s.get(1, 0) * s.get(1, 0) + // y
                    0.5 * s.get(2, 0) * s.get(2, 0); // theta (less important)

            if (score < lowestScore) {
                lowestScore = score;
                bestEstimate = estimate;
            }
        }

        if (bestEstimate != null) {
            addVisionMeasurement(
                    bestEstimate.estimatedPose.toPose2d(),
                    bestEstimate.timestampSeconds,
                    bestEstimate.standardDeviations);
            estimator.addVisionMeasurement(
                    bestEstimate.estimatedPose.toPose2d(),
                    bestEstimate.timestampSeconds,
                    bestEstimate.standardDeviations);

            timeSinceLastEstimatorUpdate = Utils.getCurrentTimeSeconds();
        }

        field.setRobotPose(getState().Pose);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));

        timeSinceLastEstimatorUpdate = Utils.getCurrentTimeSeconds();
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement
     *                                 in the form [x, y, theta]ᵀ, with units in
     *                                 meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);

        timeSinceLastEstimatorUpdate = Utils.getCurrentTimeSeconds();
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return getKinematics().toChassisSpeeds(getState().ModuleStates);
    }

    private Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(getState().Pose.getRotation().getDegrees());
    }

    private SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] smp = new SwerveModulePosition[4];
        @SuppressWarnings("rawtypes")
        SwerveModule[] sms = getModules();
        for (int i = 0; i < 4; i++) {
            smp[i] = sms[i].getPosition(false);
        }
        return smp;
    }

    public void setRobotPose(Pose2d pose) {
        estimator.resetPose(pose);
    }

    public Field2d getField() {
        return field;
    }

    public double getRobotX() {
        return estimator.getEstimatedPosition().getX();
    }

    public double getRobotY() {
        return estimator.getEstimatedPosition().getY();
    }

    public double getRobotR() {
        return estimator.getEstimatedPosition().getRotation().getDegrees();
    }

    public Pose2d getEstimator() {
        return estimator.getEstimatedPosition();
    }

    public void setLabel(Pose2d pose2d, String label) {
        field.getObject(label).setPose(pose2d);
    }

    public void passGlobalEstimates(List<PoseEstimateValues> estimates) {
        this.estimates = estimates;
    }

    public double getTimeSinceLastEstimatorUpdate() {
        return timeSinceLastEstimatorUpdate;
    }

            /**
     * @return The total current supplied to the drivetrain
     */
    public double getTotalDriveSupplyCurrent() {
        double totalCurrent = 0;
        for (int i = 0; i < 4; i++) {
            // Access each module's drive motor and get its stator current
            totalCurrent += this.getModule(i).getDriveMotor().getSupplyCurrent().getValueAsDouble();
        }
        for (int i = 0; i < 4; i++) {
            // Access each module's drive motor and get its stator current
            totalCurrent += this.getModule(i).getSteerMotor().getSupplyCurrent().getValueAsDouble();
        }
        return totalCurrent;
    }

    /**
     * Sets a current limit for the drive motors
     * @param current In amps to supply to each drive motor. -1 for default
     */
    public void setDriveCurrentLimit(double supply, double stator) {
        if (supply == -1) {
            return;
        }

        // 1. Get the existing configuration from the motor
        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();

        for (int i = 0; i < 4; i++) {

            this.getModule(i).getDriveMotor().getConfigurator().refresh(currentConfigs);

            // 2. Modify only the specific fields you need
            currentConfigs.SupplyCurrentLimit = supply;
            currentConfigs.SupplyCurrentLimitEnable = true;
            currentConfigs.StatorCurrentLimit = stator;
            currentConfigs.StatorCurrentLimitEnable = true;

            // 3. Apply the updated object back to the motor
            this.getModule(i).getDriveMotor().getConfigurator().apply(currentConfigs);

        }
    }

    /**
     * Sets a current limit for the steer motors
     * @param current In amps to supply to each steer motor. -1 for default
     */
    public void setSteerCurrentLimit(double supply, double stator) {
        if (supply == -1) {
            return;
        }

        // 1. Get the existing configuration from the motor
        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();

        for (int i = 0; i < 4; i++) {
            
            this.getModule(i).getSteerMotor().getConfigurator().refresh(currentConfigs);

            // 2. Modify only the specific fields you need
            currentConfigs.SupplyCurrentLimit = supply;
            currentConfigs.SupplyCurrentLimitEnable = true;
            currentConfigs.StatorCurrentLimit = stator;
            currentConfigs.StatorCurrentLimitEnable = true;

            // 3. Apply the updated object back to the motor
            this.getModule(i).getSteerMotor().getConfigurator().apply(currentConfigs);

        }
    }
}