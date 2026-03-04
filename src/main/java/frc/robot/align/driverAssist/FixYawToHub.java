package frc.robot.align.driverAssist;

import org.opencv.dnn.DetectionModel;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.utils.Constants;

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

    private Translation2d absoluteTargetTranslation;

    private double dtheta;
    private double error_yaw;

    boolean finishedOverride;

    private Translation2d getAbsoluteTranslation(boolean isRed) {
        if (isRed) {
            return new Translation2d(11.915394, 4.034536); // TODO
        } else {
            return new Translation2d(4.625594, 4.034536);
        }
    }

    private double printClock = 0;

    private double calculateRelativeTheta(Pose2d robotPose) {
    //     double delta_x = absoluteTargetTranslation.getX() - robotPose.getX();
    //     double delta_y = absoluteTargetTranslation.getY() - robotPose.getY();

    //     double dt = Utils.getCurrentTimeSeconds() - drivetrain.getTimeSinceLastEstimatorUpdate();
    //     dt = Math.max(0.0, Math.min(dt, MAX_LEAD_TIME_SECONDS));

    //     ChassisSpeeds speeds = drivetrain.getCurrentRobotChassisSpeeds();

    //     ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, robotPose.getRotation());

    //     // Lead the target by expected estimator staleness (or equivalent latency) in field frame.
    //     // If your sign convention drives the wrong way, flip these to -=.
    //     delta_x += fieldRelativeSpeeds.vxMetersPerSecond * dt;
    //     delta_y += fieldRelativeSpeeds.vyMetersPerSecond * dt;

    //     double hyp = Math.hypot(delta_x, delta_y);

    //     Rotation2d rotation = new Rotation2d(Math.atan2(delta_y, delta_x));

    //     Rotation2d shooterFix = new Rotation2d(Math.atan2(Constants.CENTER_TO_SHOOTER_Y, hyp));

    //     printClock++;

    //     if (printClock >= 5) {
    //         printClock = 0;
    //         System.out.println("DVE: " + drivetrain.getState().Pose);
    //         System.out.println("ERRX: " + delta_x);
    //         System.out.println("ERRY: " + delta_y);
    //     }

    //     return robotPose.getRotation().minus(rotation).plus(shooterFix).getRadians(); // if turn direction is inverted: rotation.minus(robotPose.getRotation())

        // --- 1️⃣ Shooter offset in robot frame ---
        Translation2d shooterOffset = new Translation2d(
            Constants.CENTER_TO_SHOOTER_X,
            Constants.CENTER_TO_SHOOTER_Y
        );

        // --- 2️⃣ Shooter position in field frame ---
        Translation2d shooterFieldPos =
            robotPose.getTranslation().plus(
                shooterOffset.rotateBy(robotPose.getRotation())
            );

        // --- 3️⃣ Latency / estimator lead compensation ---
        double dt = Utils.getCurrentTimeSeconds()
                - drivetrain.getTimeSinceLastEstimatorUpdate();

        dt = Math.max(0.0, Math.min(dt, MAX_LEAD_TIME_SECONDS));

        ChassisSpeeds robotSpeeds = drivetrain.getCurrentRobotChassisSpeeds();

        ChassisSpeeds fieldSpeeds =
            ChassisSpeeds.fromRobotRelativeSpeeds(
                robotSpeeds,
                robotPose.getRotation()
            );

        Translation2d predictedShooterPos =
            shooterFieldPos.plus(
                new Translation2d(
                    fieldSpeeds.vxMetersPerSecond * dt,
                    fieldSpeeds.vyMetersPerSecond * dt
                )
            );

        // --- 4️⃣ Vector from shooter → hub ---
        Translation2d toHub =
            absoluteTargetTranslation.minus(predictedShooterPos);

        // --- 5️⃣ Desired field heading ---
        Rotation2d desiredHeading = toHub.getAngle();

        // --- 6️⃣ Angular error (desired - current) ---
        return desiredHeading
                .minus(robotPose.getRotation())
                .getRadians();    
}

    public FixYawToHub(DriveSubsystem drivetrain, boolean isRed) {
        System.out.println("YAW LOCK CREATED");
        finishedOverride = false;

        this.drivetrain = drivetrain;

        this.absoluteTargetTranslation = getAbsoluteTranslation(isRed);

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

        error_yaw = calculateRelativeTheta(drivetrain.getState().Pose);

        // Normalize yaw error to -π to π range
        error_yaw = Rotation2d.fromRadians(error_yaw).getRadians();

        // Calculate PID outputs
        dtheta = yawController.calculate(error_yaw);

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

//         // --- 1️⃣ Shooter offset in robot frame ---
//         Translation2d shooterOffset = new Translation2d(
//             Constants.CENTER_TO_SHOOTER_X,
//             Constants.CENTER_TO_SHOOTER_Y
//         );

//         // --- 2️⃣ Shooter position in field frame ---
//         Translation2d shooterFieldPos =
//             robotPose.getTranslation().plus(
//                 shooterOffset.rotateBy(robotPose.getRotation())
//             );

//         // --- 3️⃣ Latency / estimator lead compensation ---
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

//         // --- 4️⃣ Vector from shooter → hub ---
//         Translation2d toHub =
//             absoluteTargetTranslation.minus(predictedShooterPos);

//         // --- 5️⃣ Desired field heading ---
//         Rotation2d desiredHeading = toHub.getAngle();

//         // --- 6️⃣ Angular error (desired - current) ---
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