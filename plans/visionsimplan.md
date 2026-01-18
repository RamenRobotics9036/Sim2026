# PhotonVision Simulation Integration Plan

## Overview
Add PhotonVision simulation support to the robot project, enabling AprilTag detection and pose estimation in the WPILib simulator.

---

## Prerequisites

### 1. ✅ Add PhotonLib Vendor Dependency
Download the PhotonLib JSON file to `vendordeps/`:
```bash
cd vendordeps
curl -o photonlib.json https://maven.photonvision.org/repository/internal/org/photonvision/photonlib-json/1.0/photonlib-json-1.0.json
```

Or use VS Code: `Ctrl+Shift+P` → "WPILib: Manage Vendor Libraries" → "Install new libraries (online)" → paste the PhotonLib URL.

---

## Files to Create

### 2. ✅ Create `src/main/java/frc/robot/Vision.java`

Based on `src/photonexample/poseest/src/main/java/frc/robot/Vision.java`:

**Key Components:**
- `PhotonCamera` - Interface to the camera (real or simulated)
- `PhotonPoseEstimator` - Estimates robot pose from AprilTag detections
- `VisionSystemSim` - Simulates the vision system (AprilTags on field)
- `PhotonCameraSim` - Simulates camera behavior with configurable properties
- `EstimateConsumer` - Functional interface to pass pose estimates to drivetrain

**Simulation Setup (in constructor):**
```java
if (Robot.isSimulation()) {
    visionSim = new VisionSystemSim("main");
    visionSim.addAprilTags(kTagLayout);

    var cameraProp = new SimCameraProperties();
    cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
    cameraProp.setCalibError(0.35, 0.10);
    cameraProp.setFPS(15);
    cameraProp.setAvgLatencyMs(50);
    cameraProp.setLatencyStdDevMs(15);

    cameraSim = new PhotonCameraSim(camera, cameraProp);
    visionSim.addCamera(cameraSim, kRobotToCam);
    cameraSim.enableDrawWireframe(true);
}
```

**Methods to implement:**
- `periodic()` - Process camera results and send pose estimates
- `simulationPeriodic(Pose2d robotSimPose)` - Update simulation with robot pose
- `resetSimPose(Pose2d pose)` - Reset simulation pose
- `getSimDebugField()` - Get Field2d for visualization
- `updateEstimationStdDevs()` - Dynamic std devs based on tag count/distance

---

### 3. ✅ Add Vision Constants to `src/main/java/frc/robot/Constants.java`

Create new file or add to existing constants:

```java
public class Constants {
    public static class Vision {
        // Camera name (must match PhotonVision camera name)
        public static final String kCameraName = "photonvision";

        // Camera position relative to robot center
        // Example: mounted facing forward, 0.5m forward, 0.5m up
        public static final Transform3d kRobotToCam = new Transform3d(
            new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(0, 0, 0)
        );

        // AprilTag field layout (use current game's field)
        public static final AprilTagFieldLayout kTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // Standard deviations for pose estimation
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }
}
```

---

## Files to Modify

### 4. ✅ Update `src/main/java/frc/robot/Robot.java`

**Add imports:**
```java
import edu.wpi.first.math.geometry.Pose2d;
```

**Add Vision field:**
```java
private Vision m_vision;
```

**Initialize in constructor (after RobotContainer):**
```java
public Robot() {
    m_robotContainer = new RobotContainer();
    m_vision = new Vision(m_robotContainer.drivetrain::addVisionMeasurement);
}
```

**Add to `robotPeriodic()`:**
```java
// Update vision (processes camera results and updates pose estimator)
m_vision.periodic();
```

**Add to `simulationPeriodic()`:**
```java
@Override
public void simulationPeriodic() {
    // Update vision simulation with current robot pose
    m_vision.simulationPeriodic(m_robotContainer.drivetrain.getState().Pose);

    // Optional: visualize estimated pose on field
    var debugField = m_vision.getSimDebugField();
    if (debugField != null) {
        debugField.getObject("EstimatedRobot").setPose(
            m_robotContainer.drivetrain.getState().Pose
        );
    }
}
```

**Remove Limelight code (optional):**
- Remove `kUseLimelight` field
- Remove Limelight-related code from `robotPeriodic()`

---

### 5. ✅ Update `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java`

The drivetrain already has `addVisionMeasurement()` methods - no changes needed!

The existing methods handle:
- `addVisionMeasurement(Pose2d pose, double timestamp)`
- `addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs)`

---

## Integration Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        Robot.java                           │
│  ┌─────────────────┐         ┌─────────────────────────┐   │
│  │  RobotContainer │         │        Vision           │   │
│  │   (drivetrain)  │◄────────│  - PhotonCamera         │   │
│  └────────┬────────┘         │  - PhotonPoseEstimator  │   │
│           │                  │  - VisionSystemSim      │   │
│           │                  │  - PhotonCameraSim      │   │
│           ▼                  └───────────┬─────────────┘   │
│  addVisionMeasurement()                  │                  │
│                                          │                  │
│  simulationPeriodic() ──────────────────►│                  │
│     (passes robot pose to vision sim)    │                  │
└─────────────────────────────────────────────────────────────┘
```

---

## Testing the Integration

1. **Build the project:**
   ```bash
   ./gradlew build
   ```

2. **Run simulation:**
   ```bash
   ./gradlew simulateJava
   ```

3. **Verify in simulation:**
   - Open AdvantageScope or Shuffleboard
   - Check the Field2d widget for:
     - Robot position (from drivetrain)
     - VisionEstimation pose (from PhotonVision)
     - AprilTag positions
   - Drive robot near AprilTags to see vision corrections

---

## Optional Enhancements

### Multiple Cameras
Add additional cameras by creating multiple `PhotonCamera`/`PhotonCameraSim` pairs with different `Transform3d` offsets.

### Custom Camera Properties
Adjust `SimCameraProperties` to match your actual camera:
- Resolution (default: 960x720)
- Field of view
- FPS (default: 15)
- Latency (default: 50ms average)

### Field2d Visualization
The `VisionSystemSim` provides a debug field that shows:
- Simulated robot pose
- Visible AprilTags
- Camera frustum wireframe

---

## File Summary

| Action | File Path |
|--------|-----------|
| **Create** | `vendordeps/photonlib.json` |
| **Create** | `src/main/java/frc/robot/Vision.java` |
| **Create** | `src/main/java/frc/robot/Constants.java` |
| **Modify** | `src/main/java/frc/robot/Robot.java` |

---

## References

- [PhotonVision Docs - Simulation](https://docs.photonvision.org/en/latest/docs/simulation/simulation.html)
- [PhotonLib API](https://docs.photonvision.org/en/latest/docs/programming/photonlib/index.html)
- Example code: `src/photonexample/poseest/`
