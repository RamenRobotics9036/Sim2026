package frc.robot.simphotontolimelight;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import java.util.List;

/**
 * Pure transformation functions: PhotonVision → LimelightData.
 * Stateless, no I/O - fully unit testable.
 */
public class PhotonToLimelightConverter {

    /**
     * Convert a single PhotonTrackedTarget to basic Limelight targeting data.
     */
    public static void convertTarget(PhotonTrackedTarget target, LimelightData data) {
        if (target == null) {
            data.targetValid = false;
            return;
        }

        data.targetValid = true;
        data.tx = target.getYaw();
        data.ty = target.getPitch();
        data.txnc = target.getYaw();
        data.tync = target.getPitch();
        data.ta = target.getArea();
        data.tid = target.getFiducialId();
    }

    /**
     * Convert target 3D pose data to Limelight format.
     */
    public static void convertTargetPose3d(PhotonTrackedTarget target, LimelightData data) {
        Transform3d camToTarget = target.getBestCameraToTarget();

        data.targetPoseCameraSpace = transform3dToArray(camToTarget);
        data.cameraPoseTargetSpace = transform3dToArray(camToTarget.inverse());
    }

    /**
     * Convert list of targets to rawfiducials array format.
     */
    public static void convertRawFiducials(
            List<PhotonTrackedTarget> targets,
            Transform3d robotToCamera,
            LimelightData data) {

        if (targets.isEmpty()) {
            data.rawFiducials = new double[0];
            return;
        }

        data.rawFiducials = new double[targets.size() * 7];

        for (int i = 0; i < targets.size(); i++) {
            PhotonTrackedTarget target = targets.get(i);
            int baseIndex = i * 7;

            Transform3d camToTarget = target.getBestCameraToTarget();
            double distToCamera = camToTarget.getTranslation().getNorm();

            Transform3d robotToTarget = robotToCamera.plus(camToTarget);
            double distToRobot = robotToTarget.getTranslation().getNorm();

            data.rawFiducials[baseIndex + 0] = target.getFiducialId();
            data.rawFiducials[baseIndex + 1] = target.getYaw();
            data.rawFiducials[baseIndex + 2] = target.getPitch();
            data.rawFiducials[baseIndex + 3] = target.getArea();
            data.rawFiducials[baseIndex + 4] = distToCamera;
            data.rawFiducials[baseIndex + 5] = distToRobot;
            data.rawFiducials[baseIndex + 6] = target.getPoseAmbiguity();
        }
    }

    /**
     * Convert latency data from pipeline result.
     */
    public static void convertLatency(PhotonPipelineResult result, LimelightData data) {
        data.pipelineLatencyMs = result.metadata.getLatencyMillis();

        // Capture latency is typically small; use a nominal value for simulation
        data.captureLatencyMs = 5.0;    }

    /**
     * Build the t2d array from targets and latency.
     */
    public static void convertT2D(
            List<PhotonTrackedTarget> targets,
            double latency,
            double captureLatency,
            LimelightData data) {

        data.t2d = new double[17];

        if (targets.isEmpty()) {
            data.t2d[0] = 0;
            return;
        }

        PhotonTrackedTarget primary = targets.get(0);

        data.t2d[0] = 1;
        data.t2d[1] = targets.size();
        data.t2d[2] = latency;
        data.t2d[3] = captureLatency;
        data.t2d[4] = primary.getYaw();
        data.t2d[5] = primary.getPitch();
        data.t2d[6] = primary.getYaw();
        data.t2d[7] = primary.getPitch();
        data.t2d[8] = primary.getArea();
        data.t2d[9] = primary.getFiducialId();
        data.t2d[10] = 0;
        data.t2d[11] = 0;
        data.t2d[12] = 0;
        data.t2d[13] = 0;
        data.t2d[14] = 0;
        data.t2d[15] = 0;
        data.t2d[16] = primary.getSkew();
    }

    /**
     * Convenience: Convert entire pipeline result to LimelightData.
     */
    public static LimelightData convertPipelineResult(
            PhotonPipelineResult result,
            Transform3d robotToCamera) {

        LimelightData data = new LimelightData();
        List<PhotonTrackedTarget> targets = result.getTargets();

        convertLatency(result, data);

        if (!targets.isEmpty()) {
            PhotonTrackedTarget primary = targets.get(0);
            convertTarget(primary, data);
            convertTargetPose3d(primary, data);
        }

        convertRawFiducials(targets, robotToCamera, data);
        convertT2D(targets, data.pipelineLatencyMs, data.captureLatencyMs, data);

        return data;
    }

    // Helper: Transform3d → double[6] array
    private static double[] transform3dToArray(Transform3d transform) {
        return new double[] {
            transform.getX(),
            transform.getY(),
            transform.getZ(),
            Units.radiansToDegrees(transform.getRotation().getX()),
            Units.radiansToDegrees(transform.getRotation().getY()),
            Units.radiansToDegrees(transform.getRotation().getZ())
        };
    }
}
