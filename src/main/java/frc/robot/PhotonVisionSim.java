package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * Simulation helper for PhotonVision that tracks the ground truth robot pose
 * independently of odometry drift. This allows testing vision correction
 * by providing ground truth to the simulated cameras.
 */
public class PhotonVisionSim {

    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;

    /** The ground truth pose tracks where the robot actually is in simulation physics. */
    private Pose2d groundTruthPose = new Pose2d();

    /** Track accumulated distance for telemetry */
    private double totalDistanceTraveled = 0.0;
    private double totalRotation = 0.0;

    /** Last update time for delta calculation */
    private double lastUpdateTime;

    /**
     * Constructs a PhotonVisionSim instance.
     * This class is only intended for use in simulation.
     *
     * @param drivetrain The swerve drivetrain to track and manipulate
     * @throws IllegalStateException if called outside of simulation mode
     */
    public PhotonVisionSim(SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain) {
        if (!Robot.isSimulation()) {
            throw new IllegalStateException("PhotonVisionSim should only be instantiated in simulation mode");
        }
        this.drivetrain = drivetrain;
        this.lastUpdateTime = Utils.getCurrentTimeSeconds();
    }

    /**
     * Updates the ground truth pose by integrating chassis speeds.
     * Call this from Robot.simulationPeriodic().
     */
    public void updateGroundTruthPose() {
        double currentTime = Utils.getCurrentTimeSeconds();
        double deltaTime = currentTime - lastUpdateTime;
        lastUpdateTime = currentTime;

        ChassisSpeeds speeds = drivetrain.getState().Speeds;

        // Calculate how much the robot moved this timestep
        double dx = speeds.vxMetersPerSecond * deltaTime;
        double dy = speeds.vyMetersPerSecond * deltaTime;
        double dtheta = speeds.omegaRadiansPerSecond * deltaTime;

        double distanceThisStep = Math.hypot(dx, dy);
        double rotationThisStep = Math.abs(dtheta);

        totalDistanceTraveled += distanceThisStep;
        totalRotation += rotationThisStep;

        // Update the ground truth pose (using field-relative velocities)
        // Rotate the robot-relative velocity by the current heading to get field-relative
        double cos = Math.cos(groundTruthPose.getRotation().getRadians());
        double sin = Math.sin(groundTruthPose.getRotation().getRadians());
        double fieldDx = dx * cos - dy * sin;
        double fieldDy = dx * sin + dy * cos;

        groundTruthPose = new Pose2d(
            groundTruthPose.getX() + fieldDx,
            groundTruthPose.getY() + fieldDy,
            groundTruthPose.getRotation().plus(new Rotation2d(dtheta))
        );
    }

    /**
     * Gets the ground truth simulated pose (where the robot actually is based on physics).
     * Use this for PhotonVision simulation so cameras see the correct AprilTags.
     *
     * @return The ground truth pose of the robot in simulation
     */
    public Pose2d getGroundTruthPose() {
        return groundTruthPose;
    }

    /**
     * Resets the ground truth simulated pose to match the current estimated pose.
     * Call this when you reset the robot pose.
     */
    // I commented this out because I dont like the idea of just resetting to the simulation pose; you cant
    // do that in real life, so I dont let it happen in this sim either.  Instead, we allow resetting
    // the robot position to a well known position, just like resetting the robot in real life.
    //public void resetGroundTruthPose() {
    //    groundTruthPose = drivetrain.getState().Pose;
    //    totalDistanceTraveled = 0.0;
    //    totalRotation = 0.0;
    //}

    /**
     * Resets both the ground truth pose and the drivetrain pose to the origin.
     * Useful for resetting the simulation to a known state.
     */
    public void resetAllPoses() {
        groundTruthPose = new Pose2d();
        drivetrain.resetPose(new Pose2d());
        totalDistanceTraveled = 0.0;
        totalRotation = 0.0;
    }

    /**
     * Resets both the ground truth pose and the drivetrain pose to the specified pose.
     *
     * @param pose The pose to reset both ground truth and drivetrain to
     */
    public void resetAllPoses(Pose2d pose) {
        groundTruthPose = pose;
        drivetrain.resetPose(pose);
        totalDistanceTraveled = 0.0;
        totalRotation = 0.0;
    }

    /**
     * Resets both poses to a PathPlanner auto starting pose, flipping for red alliance if needed.
     * PathPlanner paths are designed for blue alliance origin; this mirrors the pose when on red.
     *
     * @param blueAlliancePose The pose as defined in PathPlanner (blue alliance origin)
     */
    public void resetAllPosesToSelectedAutoPos(Pose2d blueAlliancePose) {
        Pose2d pose = isRedAlliance()
            ? FlippingUtil.flipFieldPose(blueAlliancePose)
            : blueAlliancePose;
        resetAllPoses(pose);
    }

    /**
     * @return true if the robot is currently configured as red alliance
     */
    private boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }


    /**
     * Introduces simulated odometry drift by offsetting the pose estimator.
     * The "true" pose remains unchanged, but the pose estimator
     * is reset to a drifted position. Vision should then correct this drift.
     *
     * @param translationOffsetMeters How far to offset the estimated position (meters)
     * @param rotationOffsetDegrees How far to offset the estimated heading (degrees)
     */
    public void injectDrift(double translationOffsetMeters, double rotationOffsetDegrees) {
        // Get current estimated pose
        Pose2d currentPose = drivetrain.getState().Pose;

        // Create offset - add random direction for translation
        double angle = Math.random() * 2 * Math.PI;
        double dx = translationOffsetMeters * Math.cos(angle);
        double dy = translationOffsetMeters * Math.sin(angle);

        // Apply random sign to rotation
        double dtheta = rotationOffsetDegrees * (Math.random() > 0.5 ? 1 : -1);

        // Create the drifted pose
        Pose2d driftedPose = new Pose2d(
            currentPose.getX() + dx,
            currentPose.getY() + dy,
            currentPose.getRotation().plus(Rotation2d.fromDegrees(dtheta))
        );

        // Reset the pose estimator to the drifted position
        // The ground truth pose remains at the actual position
        drivetrain.resetPose(driftedPose);
    }

    /**
     * Publishes simulation telemetry to SmartDashboard.
     * Call this from Robot.simulationPeriodic().
     */
    public void publishTelemetry() {
        SmartDashboard.putNumber("Sim/GroundTruth/X", groundTruthPose.getX());
        SmartDashboard.putNumber("Sim/GroundTruth/Y", groundTruthPose.getY());
        SmartDashboard.putNumber("Sim/GroundTruth/RotationDeg", groundTruthPose.getRotation().getDegrees());

        Pose2d estimatedPose = drivetrain.getState().Pose;
        SmartDashboard.putNumber("Sim/EstimatedPose/X", estimatedPose.getX());
        SmartDashboard.putNumber("Sim/EstimatedPose/Y", estimatedPose.getY());
        SmartDashboard.putNumber("Sim/EstimatedPose/RotationDeg", estimatedPose.getRotation().getDegrees());

        double poseError = groundTruthPose.getTranslation().getDistance(estimatedPose.getTranslation());
        double headingError = Math.abs(groundTruthPose.getRotation().minus(estimatedPose.getRotation()).getDegrees());
        SmartDashboard.putNumber("Sim/PoseErrorMeters", poseError);
        SmartDashboard.putNumber("Sim/HeadingErrorDeg", headingError);
        SmartDashboard.putNumber("Sim/TotalDistanceTraveled", totalDistanceTraveled);
    }

    /**
     * Updates ground truth pose, runs vision simulation, and publishes telemetry.
     * Call this from Robot.simulationPeriodic().
     */
    public void simulationPeriodicPhotonSim() {
        // Update the ground truth pose tracking
        updateGroundTruthPose();

        // Publish simulation telemetry (pose errors, etc.)
        publishTelemetry();
    }
}
