package frc.robot.simphotontolimelight;

import edu.wpi.first.math.geometry.Transform3d;

/**
 * Configuration for mapping a PhotonVision camera to a Limelight NetworkTables output.
 */
public class CameraMapping {
    public final String photonCameraName;
    public final String limelightTableName;
    public final Transform3d robotToCamera;

    public CameraMapping(String photonCameraName, String limelightTableName, Transform3d robotToCamera) {
        this.photonCameraName = photonCameraName;
        this.limelightTableName = limelightTableName;
        this.robotToCamera = robotToCamera;
    }
}
