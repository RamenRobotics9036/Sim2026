# Plan: Move SimTruePose Code from CommandSwerveDrivetrain to PhotonVisionSim

## Problem
Commit `2751dfa` added ~126 lines of simulation-specific code to `CommandSwerveDrivetrain` for tracking "true pose" and testing vision correction. This code is specific to PhotonVision simulation and pollutes the drivetrain class with concerns it shouldn't own.

## Code Added in 2751dfa (to be moved/removed)

### Fields (lines 48-63 in CommandSwerveDrivetrain)
```java
// Simulation noise parameters
private static final double kSimTranslationDriftMetersPerMeter = 0.02;
private static final double kSimRotationDriftDegreesPerDegree = 0.02;

// True pose tracking
private Pose2d m_simTruePose = new Pose2d();
private double m_simTotalDistanceTraveled = 0.0;
private double m_simTotalRotation = 0.0;
```

### Methods to Move
1. `getSimTruePose()` - Get the true simulated pose
2. `resetSimTruePose()` - Reset true pose to match estimated
3. `injectSimulatedDrift()` - Inject drift for testing vision

### Code in `periodic()` (SmartDashboard telemetry)
- Publishing Sim/TruePose/*, Sim/EstimatedPose/*, Sim/PoseErrorMeters, etc.

### Code in `startSimThread()` (true pose integration)
- The 20+ lines that integrate chassis speeds to track true pose

---

## Proposed Solution

### Option A: PhotonVisionSim Owns True Pose Tracking (Recommended)

**Concept**: `PhotonVisionSim` maintains its own true pose by periodically querying the drivetrain's state and integrating velocities.

#### Step 1: Add True Pose Tracking to PhotonVisionSim

```java
public class PhotonVisionSim {
    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;
    private Pose2d truePose = new Pose2d();
    private double totalDistanceTraveled = 0.0;
    private double lastUpdateTime;

    public PhotonVisionSim(SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain) {
        if (!Robot.isSimulation()) {
            throw new IllegalStateException("...");
        }
        this.drivetrain = drivetrain;
        this.lastUpdateTime = Utils.getCurrentTimeSeconds();
    }

    /** Call this from Robot.simulationPeriodic() */
    public void updateTruePose() {
        double currentTime = Utils.getCurrentTimeSeconds();
        double deltaTime = currentTime - lastUpdateTime;
        lastUpdateTime = currentTime;

        ChassisSpeeds speeds = drivetrain.getState().Speeds;
        // ... integrate velocities to update truePose ...
    }

    public Pose2d getTruePose() {
        return truePose;
    }

    public void resetTruePose() {
        truePose = drivetrain.getState().Pose;
        totalDistanceTraveled = 0.0;
    }

    public void injectDrift(double translationMeters, double rotationDegrees) {
        // ... inject drift into drivetrain.resetPose() ...
    }

    public void publishTelemetry() {
        // SmartDashboard.putNumber("Sim/TruePose/X", ...)
    }
}
```

#### Step 2: Update RobotContainer

```java
public class RobotContainer {
    public PhotonVisionSim visionSim = null;
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        if (Robot.isSimulation()) {
            visionSim = new PhotonVisionSim(drivetrain);
            visionSim.setDrivetrainToTrustVisionMore(drivetrain);
        }
    }
}
```

#### Step 3: Update Robot.java

```java
@Override
public void simulationPeriodic() {
    if (m_robotContainer.visionSim != null) {
        m_robotContainer.visionSim.updateTruePose();
        m_robotContainer.visionSim.publishTelemetry();
    }
}
```

#### Step 4: Remove from CommandSwerveDrivetrain

1. Remove fields: `m_simTruePose`, `m_simTotalDistanceTraveled`, `m_simTotalRotation`, `kSimTranslationDriftMetersPerMeter`, `kSimRotationDriftDegreesPerDegree`
2. Remove methods: `getSimTruePose()`, `resetSimTruePose()`, `injectSimulatedDrift()`
3. Remove SmartDashboard code from `periodic()`
4. Remove true pose integration from `startSimThread()` (keep only the `updateSimState` call)

---

### Option B: Keep Minimal Hook in CommandSwerveDrivetrain

If `startSimThread()` timing is critical (4ms loop), keep a callback hook:

```java
// In CommandSwerveDrivetrain
private Consumer<Double> simUpdateCallback = null;

public void setSimUpdateCallback(Consumer<Double> callback) {
    this.simUpdateCallback = callback;
}

// In startSimThread():
if (simUpdateCallback != null) {
    simUpdateCallback.accept(deltaTime);
}
```

Then `PhotonVisionSim` registers itself to receive deltaTime updates.

---

## Recommended Approach: Option A

**Rationale**:
1. `Robot.simulationPeriodic()` runs at 20ms (50Hz) which is sufficient for pose tracking
2. Keeps CommandSwerveDrivetrain clean and focused on drivetrain concerns
3. PhotonVisionSim becomes a self-contained simulation helper
4. Easier to add/remove simulation features without touching core drivetrain code

---

## Migration Checklist

- [ ] Add `drivetrain` field to PhotonVisionSim constructor
- [ ] Move `truePose`, `totalDistanceTraveled`, `totalRotation` fields to PhotonVisionSim
- [ ] Create `updateTruePose()` method in PhotonVisionSim
- [ ] Move `injectSimulatedDrift()` logic to PhotonVisionSim (calling `drivetrain.resetPose()`)
- [ ] Create `publishTelemetry()` method for SmartDashboard output
- [ ] Update RobotContainer to pass drivetrain to PhotonVisionSim constructor
- [ ] Add `visionSim.updateTruePose()` call in Robot.simulationPeriodic()
- [ ] Remove simulation-specific fields from CommandSwerveDrivetrain
- [ ] Remove simulation-specific methods from CommandSwerveDrivetrain
- [ ] Remove SmartDashboard code from CommandSwerveDrivetrain.periodic()
- [ ] Simplify startSimThread() to only call updateSimState()
- [ ] Test that simulation still works correctly
- [ ] Verify vision correction still functions

---

## Files to Modify

1. **PhotonVisionSim.java** - Add all simulation tracking logic
2. **CommandSwerveDrivetrain.java** - Remove ~80 lines of simulation code
3. **RobotContainer.java** - Update PhotonVisionSim instantiation
4. **Robot.java** - Add simulationPeriodic() calls if needed
