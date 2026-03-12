package frc.robot.align.driverAssist;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.utils.Constants;
import frc.robot.utils.aiming.AimingUtil;
import frc.robot.utils.aiming.LaunchCalculator;
import frc.robot.utils.aiming.LaunchingParameters;

public class FixYawToHub extends Command {

    // PID for Rotation
    private final PIDController yawController;

    // Rate limiter
    private final SlewRateLimiter yawRateLimiter = new SlewRateLimiter(6.0);

    // Position tolerance
    private static final double YAW_TOLERANCE = 0.5 * Math.PI / 180; // radians

    // Maximum output values
    private static final double MAX_ANGULAR_SPEED = 2;

    private static final double THETA_SPEED_MODIFIER = 1.25;

    // Minimum output to overcome static friction
    private static final double MIN_ANGULAR_COMMAND = 0.2;
    private static final double MAX_LEAD_TIME_SECONDS = 0.25;

    private final DriveSubsystem drivetrain;
    private final LaunchCalculator launchCalculator;

    private Translation2d absoluteTargetTranslation;

    private double dtheta;
    private double error_yaw;

    boolean finishedOverride;

    // private Translation2d getAbsoluteTranslation(Pose2d robotPose) {
    //     boolean isRed = (DriverStation.getAlliance()).get() == Alliance.Red;

    //     double robotX = robotPose.getX();
    //     Translation2d robotTranslation = robotPose.getTranslation();

    //     Translation2d hubTarget = isRed
    //             ? new Translation2d(Constants.HUB_RED_X, Constants.HUB_Y)
    //             : new Translation2d(Constants.HUB_BLUE_X, Constants.HUB_Y);

    //     boolean beyondHub = isRed
    //             ? robotX < Constants.HUB_RED_X - 1.75
    //             : robotX > Constants.HUB_BLUE_X + 1.75;

    //     if (!beyondHub) {
    //         return hubTarget;
    //     }

    //     Translation2d nearCorner = isRed
    //             ? Constants.RED_SIDE_CORNER_NEAR
    //             : Constants.BLUE_SIDE_CORNER_NEAR;
    //     Translation2d farCorner = isRed
    //             ? Constants.RED_SIDE_CORNER_FAR
    //             : Constants.BLUE_SIDE_CORNER_FAR;

    //     double nearDistance = robotTranslation.getDistance(nearCorner);
    //     double farDistance = robotTranslation.getDistance(farCorner);

    //     return nearDistance <= farDistance ? nearCorner : farCorner;
    // }

    // private double printClock = 0;

    private double calculateRelativeTheta(Pose2d robotPose) {
        Translation2d shooterFieldPos = AimingUtil.getShooterFieldPosition(robotPose);

        // --- Latency / estimator lead compensation ---
        double dt = Utils.getCurrentTimeSeconds()
                - drivetrain.getTimeSinceLastEstimatorUpdate();

        dt = Math.max(0.0, Math.min(dt, MAX_LEAD_TIME_SECONDS));

        ChassisSpeeds robotRelativeSpeeds = drivetrain.getCurrentRobotChassisSpeeds();
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                robotRelativeSpeeds,
                robotPose.getRotation());

        Translation2d predictedShooterPos =
            shooterFieldPos.plus(
                new Translation2d(
                    fieldRelativeSpeeds.vxMetersPerSecond * dt,
                    fieldRelativeSpeeds.vyMetersPerSecond * dt
                )
            );
        
        absoluteTargetTranslation = AimingUtil.getTargetTranslation(robotPose);

        // --- Vector from shooter to hub ---
        Translation2d toHub =
            absoluteTargetTranslation.minus(predictedShooterPos);
            

        // --- Desired field heading ---
        Rotation2d desiredHeading = toHub.getAngle();

        // --- Angular error (desired - current) ---
        return desiredHeading
                .minus(robotPose.getRotation())
                .getRadians();    
}

    public FixYawToHub(DriveSubsystem drivetrain, LaunchCalculator launchCalculator) {
        System.out.println("YAW LOCK CREATED");
        finishedOverride = false;

        this.drivetrain = drivetrain;
        this.launchCalculator = launchCalculator;


        // Yaw PID coefficients
        yawController = new PIDController(3, 0.0, 2);
        yawController.setTolerance(YAW_TOLERANCE);
        yawController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        System.out.println("YAW LOCK STARTED");

        // Reset controllers and rate limiters
        yawController.reset();
        yawRateLimiter.reset(0);
    }

    private int logClock = 0;

    @Override
    public void execute() {
        if (Constants.ENABLE_SHOOT_ON_MOVE) {
            LaunchingParameters parameters = launchCalculator.getParameters();
            double measuredOmega = drivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond;
            double omegaOutput = parameters.driveVelocity()
                + (parameters.driveAngle().minus(drivetrain.getState().Pose.getRotation()).getRadians()
                    * Constants.SOTM_DRIVE_KP)
                + ((parameters.driveVelocity() - measuredOmega) * Constants.SOTM_DRIVE_KD);

            dtheta = Math.max(-MAX_ANGULAR_SPEED, Math.min(omegaOutput, MAX_ANGULAR_SPEED));
            dtheta = yawRateLimiter.calculate(dtheta);
            return;
        }

        error_yaw = calculateRelativeTheta(drivetrain.getState().Pose);

        // Normalize yaw error to -π to π range
        error_yaw = Rotation2d.fromRadians(error_yaw).getRadians();

        // Calculate PID output from yaw error to zero error setpoint.
        dtheta = yawController.calculate(error_yaw, 0.0);

        // Apply minimum command if needed
        if (Math.abs(error_yaw) > YAW_TOLERANCE && Math.abs(dtheta) < MIN_ANGULAR_COMMAND) {
            dtheta = MIN_ANGULAR_COMMAND * Math.signum(dtheta);
        }

        // Limit output to maximum value
        dtheta = Math.max(-MAX_ANGULAR_SPEED, Math.min(dtheta * THETA_SPEED_MODIFIER, MAX_ANGULAR_SPEED));

        // Apply rate limiting for smoother motion
        dtheta = yawRateLimiter.calculate(dtheta);

        logClock++;
        if (logClock == 10) {
        //     System.out.println("DVE: " + drivetrain.getState().Pose);
        //     System.out.println("PER: " + tempA);
        //     System.out.println("ERR: " + error_yaw);
        //     System.out.println("OTP: " + dtheta);
        //     // SmartDashboard.putNumber("DVE", drivetrain.getEstimator());

            logClock = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        finishedOverride = true;

        if (interrupted) {
            System.out.println("YAW LOCK INTERRUPTED");
        } else {
            System.out.println("YAW LOCK FINISHED");
        }
    }

    public double getRotationalRate() {
        return dtheta;
    }
}
// public class FixYawToHub extends Command {

//     // PID for Rotation
//     private final PIDController yawController;

//     // Rate limiter
//     private final SlewRateLimiter yawRateLimiter = new SlewRateLimiter(6.0);

//     // Position tolerance
//     private static final double YAW_TOLERANCE = 0.5 * Math.PI / 180.0; // radians

//     // Maximum output values
//     private static final double MAX_ANGULAR_SPEED = 2.0;
//     private static final double THETA_SPEED_MODIFIER = 1.25;

//     // Minimum output to overcome static friction
//     private static final double MIN_ANGULAR_COMMAND = 0.08;
//     private static final double MAX_LEAD_TIME_SECONDS = 0.25;

//     private final DriveSubsystem drivetrain;
//     private final Translation2d absoluteTargetTranslation;

//     private double dtheta = 0;

//     public FixYawToHub(DriveSubsystem drivetrain, boolean isRed) {

//         this.drivetrain = drivetrain;
//         this.absoluteTargetTranslation = getAbsoluteTranslation(isRed);

//         yawController = new PIDController(3.0, 0.0, 2.0);
//         yawController.setTolerance(YAW_TOLERANCE);
//         yawController.enableContinuousInput(-Math.PI, Math.PI);

//         addRequirements(drivetrain);
//     }

//     private Translation2d getAbsoluteTranslation(boolean isRed) {
//         if (isRed) {
//             return new Translation2d(11.915394, 4.034536); // TODO verify 2026 coords
//         } else {
//             return new Translation2d(4.625594, 4.034536);
//         }
//     }

//     @Override
//     public void initialize() {
//         yawController.reset();
//         yawRateLimiter.reset(0);
//     }

//     private double calculateRelativeTheta(Pose2d robotPose) {

//         // --- Shooter offset in robot frame ---
//         Translation2d shooterOffset = new Translation2d(
//             Constants.CENTER_TO_SHOOTER_X,
//             Constants.CENTER_TO_SHOOTER_Y
//         );

//         // --- Shooter position in field frame ---
//         Translation2d shooterFieldPos =
//             robotPose.getTranslation().plus(
//                 shooterOffset.rotateBy(robotPose.getRotation())
//             );

//         // --- Latency / estimator lead compensation ---
//         double dt = Utils.getCurrentTimeSeconds()
//                 - drivetrain.getTimeSinceLastEstimatorUpdate();

//         dt = Math.max(0.0, Math.min(dt, MAX_LEAD_TIME_SECONDS));

//         ChassisSpeeds robotSpeeds = drivetrain.getCurrentRobotChassisSpeeds();

//         ChassisSpeeds fieldSpeeds =
//             ChassisSpeeds.fromRobotRelativeSpeeds(
//                 robotSpeeds,
//                 robotPose.getRotation()
//             );

//         Translation2d predictedShooterPos =
//             shooterFieldPos.plus(
//                 new Translation2d(
//                     fieldSpeeds.vxMetersPerSecond * dt,
//                     fieldSpeeds.vyMetersPerSecond * dt
//                 )
//             );

//         // --- Vector from shooter → hub ---
//         Translation2d toHub =
//             absoluteTargetTranslation.minus(predictedShooterPos);

//         // --- Desired field heading ---
//         Rotation2d desiredHeading = toHub.getAngle();

//         // --- Angular error (desired - current) ---
//         return desiredHeading
//                 .minus(robotPose.getRotation())
//                 .getRadians();
//     }

//     @Override
//     public void execute() {

//         Pose2d robotPose = drivetrain.getState().Pose;

//         double errorYaw = calculateRelativeTheta(robotPose);

//         // PID: treat error as measurement, 0 as setpoint
//         dtheta = yawController.calculate(errorYaw, 0.0);

//         // Apply minimum command to overcome static friction
//         if (Math.abs(errorYaw) > YAW_TOLERANCE &&
//             Math.abs(dtheta) < MIN_ANGULAR_COMMAND) {

//             dtheta = MIN_ANGULAR_COMMAND * Math.signum(dtheta);
//         }

//         // Apply gain modifier + clamp
//         dtheta *= THETA_SPEED_MODIFIER;
//         dtheta = Math.max(-MAX_ANGULAR_SPEED,
//                           Math.min(dtheta, MAX_ANGULAR_SPEED));

//         // Rate limit
//         dtheta = yawRateLimiter.calculate(dtheta);
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }

//     @Override
//     public void end(boolean interrupted) {
//         dtheta = 0;
//     }

//     public double getRotationalRate() {
//         return dtheta;
//     }
// }