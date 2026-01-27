# PhotonToLimelight Enhancement Plan

## Overview

Add new methods to `PhotonToLimelight` that take `PhotonTrackedTarget` objects and write them to NetworkTables in the exact format that `LimelightHelpers.java` reads from.

## Goals

1. Enable seamless simulation of Limelight data using PhotonVision simulation
2. Allow existing code that uses `LimelightHelpers` to work transparently in simulation
3. Support AprilTag/fiducial tracking data format
4. **Testable design**: Separate data transformation from NetworkTables I/O for unit testing
5. **Multi-camera support**: Map multiple PhotonVision cameras to multiple Limelight tables

---

## Architecture: Separation of Concerns

To enable unit testing without NetworkTables dependencies, we split the code into:

1. **`LimelightData`** (Data class) - Pure data container holding Limelight-formatted values
2. **`PhotonToLimelightConverter`** (Pure functions) - Transforms PhotonVision data → `LimelightData`
3. **`LimelightTablePublisher`** (I/O layer) - Writes `LimelightData` to NetworkTables

```
                                                               ┌────────────────────────┐
┌─────────────────────┐      ┌──────────────────────────┐      │    LimelightData       │
│ PhotonTrackedTarget │─────▶│ PhotonToLimelightConverter│─────▶│   (POJO, testable)     │
│ PhotonPipelineResult│      │   (pure transformation)   │      └───────────┬────────────┘
└─────────────────────┘      └──────────────────────────┘                    │
                                                                             ▼
┌─────────────────────┐      ┌────────────────────────┐      ┌────────────────────────┐
│   CameraMapping     │─────▶│   PhotonToLimelight    │─────▶│ LimelightTablePublisher │
│ (photon → limelight)│      │    (orchestrator)      │      │  (NetworkTables I/O)   │
└─────────────────────┘      └────────────────────────┘      └────────────────────────┘
        × N cameras
```

### Multi-Camera Flow

```
┌──────────────────┐     ┌─────────────────┐
│ PhotonCamera     │────▶│ LimelightTable  │
│ "photonvision1"  │     │ "limelight"     │
└──────────────────┘     └─────────────────┘

┌──────────────────┐     ┌─────────────────┐
│ PhotonCamera     │────▶│ LimelightTable  │
│ "photonvision2"  │     │ "limelight-back"│
└──────────────────┘     └─────────────────┘

┌──────────────────┐     ┌─────────────────┐
│ PhotonCamera     │────▶│ LimelightTable  │
│ "photonvision3"  │     │ "limelight-left"│
└──────────────────┘     └─────────────────┘
```

### Benefits

- **Unit testable**: Test `PhotonToLimelightConverter` with mock inputs, assert on `LimelightData` outputs
- **No WPILib dependencies in tests**: Pure Java data classes and transformation logic
- **Swappable publisher**: Could write to different outputs (logging, mock, etc.)
- **Clear contracts**: Data class documents exact Limelight format

---

## Data Mapping: PhotonTrackedTarget → Limelight NetworkTables

### Source: PhotonTrackedTarget Methods

| Method                        | Returns              | Description                                          |
|-------------------------------|----------------------|------------------------------------------------------|
| `getYaw()`                    | `double`             | Horizontal offset in degrees                         |
| `getPitch()`                  | `double`             | Vertical offset in degrees                           |
| `getArea()`                   | `double`             | Target area (0-100% of image)                        |
| `getSkew()`                   | `double`             | Target skew/rotation                                 |
| `getFiducialId()`             | `int`                | AprilTag ID                                          |
| `getPoseAmbiguity()`          | `double`             | Pose ambiguity (0-1, lower is better)                |
| `getBestCameraToTarget()`     | `Transform3d`        | Best estimate of camera→target transform             |
| `getAlternateCameraToTarget()`| `Transform3d`        | Alternate camera→target transform                    |
| `getDetectedCorners()`        | `List<TargetCorner>` | 4 corner points of detected target                   |
| `getMinAreaRectCorners()`     | `List<TargetCorner>` | Min-area bounding rectangle corners                  |

### Target: Limelight NetworkTables Format

#### Basic Targeting Data (Single Target)

| NT Key   | Type     | Source from PhotonTrackedTarget                     |
|----------|----------|-----------------------------------------------------|
| `tv`     | `int`    | `1` if target exists, `0` if not                    |
| `tx`     | `double` | `target.getYaw()`                                   |
| `ty`     | `double` | `target.getPitch()`                                 |
| `txnc`   | `double` | `target.getYaw()` (same, no crosshair offset)       |
| `tync`   | `double` | `target.getPitch()` (same, no crosshair offset)     |
| `ta`     | `double` | `target.getArea()`                                  |
| `tid`    | `int`    | `target.getFiducialId()`                            |
| `tl`     | `double` | Pipeline latency (from PhotonPipelineResult)        |
| `cl`     | `double` | Capture latency (from PhotonPipelineResult)         |

#### 3D Pose Data (for AprilTags)

| NT Key                    | Type           | Description                                      |
|---------------------------|----------------|--------------------------------------------------|
| `targetpose_cameraspace`  | `double[6]`    | `[tx, ty, tz, pitch, yaw, roll]` from `getBestCameraToTarget()` |
| `camerapose_targetspace`  | `double[6]`    | Inverse of camera→target transform               |
| `botpose_wpiblue`         | `double[11+]`  | Robot pose in WPILib Blue origin coordinates     |

#### Raw Fiducials Array

| NT Key         | Type           | Format                                                    |
|----------------|----------------|-----------------------------------------------------------|
| `rawfiducials` | `double[]`     | `[id, txnc, tync, ta, distToCamera, distToRobot, ambiguity, ...]` |

Each fiducial entry has 7 values:
1. `id` - Fiducial ID
2. `txnc` - Horizontal offset (degrees)
3. `tync` - Vertical offset (degrees)
4. `ta` - Target area (%)
5. `distToCamera` - Distance from camera to target (meters)
6. `distToRobot` - Distance from robot to target (meters)
7. `ambiguity` - Pose ambiguity (0-1)

#### t2d Array (17 elements)

| Index | Value                        |
|-------|------------------------------|
| 0     | targetValid (0 or 1)         |
| 1     | targetCount                  |
| 2     | targetLatency                |
| 3     | captureLatency               |
| 4     | tx                           |
| 5     | ty                           |
| 6     | txnc                         |
| 7     | tync                         |
| 8     | ta                           |
| 9     | tid                          |
| 10    | targetClassIndexDetector     |
| 11    | targetClassIndexClassifier   |
| 12    | targetLongSidePixels         |
| 13    | targetShortSidePixels        |
| 14    | targetHorizontalExtentPixels |
| 15    | targetVerticalExtentPixels   |
| 16    | targetSkewDegrees            |

---

## Design: Class Structure

### 1. LimelightData (Data Container)

Pure Java data class with no external dependencies. Holds all Limelight-formatted values.

```java
package frc.robot.sim;

/**
 * Pure data container representing Limelight NetworkTables data format.
 * No WPILib/NetworkTables dependencies - fully unit testable.
 */
public class LimelightData {
    // Basic targeting
    public boolean targetValid = false;
    public double tx = 0;
    public double ty = 0;
    public double txnc = 0;
    public double tync = 0;
    public double ta = 0;
    public int tid = -1;

    // Latency
    public double pipelineLatencyMs = 0;
    public double captureLatencyMs = 0;

    // 3D pose arrays (6 elements each: x, y, z, pitch, yaw, roll)
    public double[] targetPoseCameraSpace = new double[6];
    public double[] cameraPoseTargetSpace = new double[6];

    // Raw fiducials array (7 values per fiducial)
    public double[] rawFiducials = new double[0];

    // t2d array (17 elements)
    public double[] t2d = new double[17];
}
```

### 2. PhotonToLimelightConverter (Pure Transformation)

Static methods that transform PhotonVision data to `LimelightData`. No I/O, no side effects.

```java
package frc.robot.sim;

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
        data.pipelineLatencyMs = result.metadata().getLatencyMillis();
        data.captureLatencyMs = result.metadata().getCaptureTimestampMicros() / 1000.0;
    }

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
```

### 3. LimelightTablePublisher (NetworkTables I/O)

Writes `LimelightData` to NetworkTables. This is the only class with NT dependencies.

```java
package frc.robot.sim;

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
```

### 4. CameraMapping (Configuration)

Defines the mapping between a PhotonVision camera and its corresponding Limelight table.

```java
package frc.robot.sim;

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
```

### 5. PhotonToLimelight (Multi-Camera Orchestrator)

Updated main class that manages multiple camera-to-limelight mappings.

```java
package frc.robot.sim;

import org.photonvision.PhotonCamera;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;

/**
 * Manages multiple PhotonVision cameras, publishing each to its own Limelight table.
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
            }
        }
    }
}
```

### Usage Example

```java
// Single camera (simple)
PhotonToLimelight bridge = new PhotonToLimelight(
    new CameraMapping("photonvision", "limelight", kFrontCameraTransform)
);

// Multiple cameras
PhotonToLimelight bridge = new PhotonToLimelight(List.of(
    new CameraMapping("front_camera", "limelight", kFrontCameraTransform),
    new CameraMapping("back_camera", "limelight-back", kBackCameraTransform),
    new CameraMapping("left_camera", "limelight-left", kLeftCameraTransform)
));

// In periodic:
bridge.periodic();  // Updates all cameras
```

---

## Implementation Tasks

### New Classes
1. [ ] Create `LimelightData.java` - pure data container
2. [ ] Create `PhotonToLimelightConverter.java` - static transformation methods
3. [ ] Create `LimelightTablePublisher.java` - NetworkTables I/O
4. [ ] Create `CameraMapping.java` - per-camera configuration record

### Converter Methods
5. [ ] Implement `convertTarget(PhotonTrackedTarget, LimelightData)`
6. [ ] Implement `convertTargetPose3d(PhotonTrackedTarget, LimelightData)`
7. [ ] Implement `convertRawFiducials(List<PhotonTrackedTarget>, Transform3d, LimelightData)`
8. [ ] Implement `convertLatency(PhotonPipelineResult, LimelightData)`
9. [ ] Implement `convertT2D(...)`
10. [ ] Implement `convertPipelineResult(...)` convenience method

### Multi-Camera Support
11. [ ] Update `PhotonToLimelight` to accept `List<CameraMapping>`
12. [ ] Implement `CameraInstance` inner class to hold camera + publisher pairs
13. [ ] Update `periodic()` to iterate over all camera instances

### Integration
14. [ ] Update existing usage to use new `CameraMapping` constructor
15. [ ] Test with existing LimelightHelpers usage to verify compatibility
16. [ ] Test multi-camera scenario with different table names

---

## Optional Future Enhancements

- **Corner data**: Write `tcornxy` with detected corner coordinates from `getDetectedCorners()`
- **Bot pose**: Calculate and write `botpose_wpiblue` using pose estimation data
- **JSON output**: Generate JSON structure matching Limelight's `json` dump
- **Heartbeat**: Implement incrementing `hb` (heartbeat) counter
- **Dynamic camera add/remove**: Add cameras at runtime

---

## Testing Strategy

### Unit Tests (No NetworkTables Required)

Test `PhotonToLimelightConverter` with mock `PhotonTrackedTarget` objects:

```java
@Test
void convertTarget_setsCorrectValues() {
    // Arrange
    PhotonTrackedTarget mockTarget = createMockTarget(
        yaw: 15.5, pitch: -3.2, area: 0.5, fiducialId: 7);
    LimelightData data = new LimelightData();

    // Act
    PhotonToLimelightConverter.convertTarget(mockTarget, data);

    // Assert
    assertTrue(data.targetValid);
    assertEquals(15.5, data.tx, 0.001);
    assertEquals(-3.2, data.ty, 0.001);
    assertEquals(7, data.tid);
}

@Test
void convertRawFiducials_formatsArrayCorrectly() {
    // Arrange
    List<PhotonTrackedTarget> targets = List.of(
        createMockTarget(id: 1, yaw: 10, pitch: 5, area: 0.3),
        createMockTarget(id: 4, yaw: -8, pitch: 2, area: 0.2)
    );
    LimelightData data = new LimelightData();

    // Act
    PhotonToLimelightConverter.convertRawFiducials(targets, robotToCamera, data);

    // Assert
    assertEquals(14, data.rawFiducials.length); // 7 * 2 targets
    assertEquals(1, data.rawFiducials[0]);      // first fiducial ID
    assertEquals(4, data.rawFiducials[7]);      // second fiducial ID
}

@Test
void convertT2D_builds17ElementArray() {
    // ...
}
```

### Integration Tests

Verify `LimelightTablePublisher` writes correct values that `LimelightHelpers` can read:

```java
@Test
void publishedData_readableByLimelightHelpers() {
    // Arrange
    LimelightData data = new LimelightData();
    data.targetValid = true;
    data.tx = 12.5;
    data.ty = -4.0;
    data.tid = 3;

    LimelightTablePublisher publisher = new LimelightTablePublisher("limelight");
    publisher.publish(data);

    // Act & Assert (using LimelightHelpers to read back)
    assertTrue(LimelightHelpers.getTV("limelight"));
    assertEquals(12.5, LimelightHelpers.getTX("limelight"), 0.001);
    assertEquals(-4.0, LimelightHelpers.getTY("limelight"), 0.001);
    assertEquals(3, LimelightHelpers.getFiducialID("limelight"), 0.001);
}
```

### Mock Helper for Unit Tests

```java
// Test utility to create mock PhotonTrackedTarget objects
public class MockPhotonTarget {
    public static PhotonTrackedTarget create(
            double yaw, double pitch, double area,
            int fiducialId, Transform3d cameraToTarget) {
        // Use Mockito or manual stub implementation
    }
}
```
