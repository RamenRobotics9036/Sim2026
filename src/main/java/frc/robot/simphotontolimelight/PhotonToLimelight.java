package frc.robot.simphotontolimelight;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Robot;
import frc.robot.sim.VisionSimConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Manages multiple PhotonVision cameras, publishing each to its own Limelight table.
 * Enables seamless simulation of Limelight data using PhotonVision simulation.
 */
public class PhotonToLimelight {

    private static class CameraInstance {
        final PhotonCamera camera;
        final LimelightTablePublisher publisher;
        final CameraMapping mapping;
        final PhotonPoseEstimator poseEstimator;

        CameraInstance(CameraMapping mapping) {
            this.mapping = mapping;
            this.camera = new PhotonCamera(mapping.photonCameraName);
            this.publisher = new LimelightTablePublisher(mapping.limelightTableName);
            this.poseEstimator = new PhotonPoseEstimator(
                VisionSimConstants.Vision.kTagLayout,
                mapping.robotToCamera);
        }
    }

    private final List<CameraInstance> cameras = new ArrayList<>();

    /**
     * Create with a single camera mapping (backwards compatible).
     */
    public PhotonToLimelight(CameraMapping mapping) {
        this(List.of(mapping));
    }

    /**
     * Create with multiple camera mappings.
     */
    public PhotonToLimelight(List<CameraMapping> mappings) {
        if (!Robot.isSimulation()) {
            throw new IllegalStateException(
                "PhotonToLimelight should only be instantiated in simulation");
        }

        for (CameraMapping mapping : mappings) {
            cameras.add(new CameraInstance(mapping));
        }
    }

    /**
     * Process all cameras and publish to their respective Limelight tables.
     */
    public void periodic() {
        for (CameraInstance instance : cameras) {
            for (PhotonPipelineResult result : instance.camera.getAllUnreadResults()) {
                LimelightData data = PhotonToLimelightConverter.convertPipelineResult(
                    result, instance.mapping.robotToCamera);

                // Estimate robot pose using multi-tag first, then lowest ambiguity fallback
                Optional<EstimatedRobotPose> estimatedPose = instance.poseEstimator.estimateCoprocMultiTagPose(result);
                if (estimatedPose.isEmpty()) {
                    estimatedPose = instance.poseEstimator.estimateLowestAmbiguityPose(result);
                }

                Pose3d robotPose = estimatedPose.map(est -> est.estimatedPose).orElse(null);
                double totalLatencyMs = data.pipelineLatencyMs + data.captureLatencyMs;

                PhotonToLimelightConverter.convertBotpose(
                    robotPose,
                    result.getTargets(),
                    instance.mapping.robotToCamera,
                    totalLatencyMs,
                    data);

                instance.publisher.publish(data);

                // Read back from LimelightHelpers and print fiducial info
                // LimelightHelpers.RawFiducial[] fiducials =
                //     LimelightHelpers.getRawFiducials(instance.mapping.limelightTableName);
                // for (LimelightHelpers.RawFiducial f : fiducials) {
                //     System.out.printf("[%s] Fiducial ID=%d tx=%.2f ty=%.2f ta=%.2f distCam=%.2fm distRobot=%.2fm ambiguity=%.3f%n",
                //         instance.mapping.limelightTableName, f.id, f.txnc, f.tync, f.ta, f.distToCamera, f.distToRobot, f.ambiguity);
                // }
            }
        }
    }
}
