package frc.robot.simphotontolimelight;

import org.photonvision.PhotonCamera;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;

/**
 * Manages multiple PhotonVision cameras, publishing each to its own Limelight table.
 * Enables seamless simulation of Limelight data using PhotonVision simulation.
 */
public class PhotonToLimelight {

    private static class CameraInstance {
        final PhotonCamera camera;
        final LimelightTablePublisher publisher;
        final CameraMapping mapping;

        CameraInstance(CameraMapping mapping) {
            this.mapping = mapping;
            this.camera = new PhotonCamera(mapping.photonCameraName);
            this.publisher = new LimelightTablePublisher(mapping.limelightTableName);
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
            for (var result : instance.camera.getAllUnreadResults()) {
                LimelightData data = PhotonToLimelightConverter.convertPipelineResult(
                    result, instance.mapping.robotToCamera);
                instance.publisher.publish(data);

                //if (data.tid != -1) {
                //    System.out.println("Detected fiducial ID: " + data.tid);
                //}
            }
        }
    }
}
