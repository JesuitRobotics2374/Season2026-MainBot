package frc.robot.align.driverAssist;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.utils.Ballistics;

public class FixYawToHub extends Command {

    // PID for Rotation
    private final PIDController yawController;

    // Rate limiter
    private final SlewRateLimiter yawRateLimiter = new SlewRateLimiter(6.0);

    // Position tolerance
    private static final double YAW_TOLERANCE = 3 * Math.PI / 180; // radians

    // Maximum output values
    private static final double MAX_ANGULAR_SPEED = 2;

    private static final double THETA_SPEED_MODIFIER = 1.25;

    // Minimum output to overcome static friction
    private static final double MIN_ANGULAR_COMMAND = 0.08;

    private final DriveSubsystem drivetrain;

    private Translation2d absoluteTargetTranslation;

    private double dtheta;

    boolean finishedOverride;

    private Translation2d getAbsoluteTranslation(boolean isRed) {
        if (isRed) {
            return new Translation2d(); // TODO
        } else {
            return new Translation2d(4.625594, 4.034536);
        }
    }

    //private double tempA = 0;

    private double calculateRelativeTheta(Pose2d robotPose) {
        double vRobotX = drivetrain.getCurrentRobotChassisSpeeds().vxMetersPerSecond;
        double vRobotY = drivetrain.getCurrentRobotChassisSpeeds().vyMetersPerSecond;

        double delta_x = absoluteTargetTranslation.getX() - robotPose.getX();
        double delta_y = absoluteTargetTranslation.getY() - robotPose.getY();

        double dt = Utils.getCurrentTimeSeconds() - drivetrain.getTimeSinceLastEstimatorUpdate();

        double error_x = vRobotX * dt;
        double error_y = vRobotY * dt;

        delta_x += error_x;
        delta_y += error_y;

        Rotation2d rotation = new Rotation2d(Math.atan2(delta_y, delta_x));
        
        //tempA = rotation.getRadians();

        return robotPose.getRotation().minus(rotation).getRadians();
    }

    public FixYawToHub(DriveSubsystem drivetrain, boolean isRed) {
        System.out.println("YAW LOCK CREATED");
        finishedOverride = false;

        this.drivetrain = drivetrain;

        this.absoluteTargetTranslation = getAbsoluteTranslation(isRed);

        // Yaw PID coefficients
        yawController = new PIDController(5, 0.7, 2.5);
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

        double error_yaw = calculateRelativeTheta(drivetrain.getState().Pose);

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
        dtheta = 0.0;

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