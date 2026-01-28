package frc.robot.simphotontolimelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Publishes LimelightData to NetworkTables.
 * This is the only class with NetworkTables dependency.
 */
public class LimelightTablePublisher {
    private final NetworkTable table;

    public LimelightTablePublisher(String limelightName) {
        String tableName = (limelightName == null || limelightName.isEmpty())
            ? "limelight" : limelightName;
        this.table = NetworkTableInstance.getDefault().getTable(tableName);
    }

    public void publish(LimelightData data) {
        // Basic targeting
        table.getEntry("tv").setDouble(data.targetValid ? 1 : 0);
        table.getEntry("tx").setDouble(data.tx);
        table.getEntry("ty").setDouble(data.ty);
        table.getEntry("txnc").setDouble(data.txnc);
        table.getEntry("tync").setDouble(data.tync);
        table.getEntry("ta").setDouble(data.ta);
        table.getEntry("tid").setDouble(data.tid);

        // Latency
        table.getEntry("tl").setDouble(data.pipelineLatencyMs);
        table.getEntry("cl").setDouble(data.captureLatencyMs);

        // 3D poses
        table.getEntry("targetpose_cameraspace").setDoubleArray(data.targetPoseCameraSpace);
        table.getEntry("camerapose_targetspace").setDoubleArray(data.cameraPoseTargetSpace);

        // Raw fiducials
        table.getEntry("rawfiducials").setDoubleArray(data.rawFiducials);

        // t2d array
        table.getEntry("t2d").setDoubleArray(data.t2d);
    }
}
