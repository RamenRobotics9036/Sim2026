# Pose Reset Analysis

## Overview

This document describes the reset chain for synchronizing all simulation state when resetting the robot's pose.

---

## Reset Methods

### 1. `PhotonVisionSim.resetAllPoses(Pose2d pose)`

**Location:** [PhotonVisionSim.java](../src/main/java/frc/robot/PhotonVisionSim.java)

**What it resets:**
- `groundTruthPose` - The simulated "actual" robot position (used by cameras)
- `drivetrain.resetPose(pose)` - The drivetrain's pose estimator
- `vision.resetSimPose(pose)` - The VisionSystemSim pose history
- `totalDistanceTraveled` / `totalRotation` - Telemetry counters

**Called by:**
- `resetAllPosesToSelectedAutoPos()` → `cycleResetPosition()` (triggered by left bumper in sim)

---

### 2. `Vision.resetSimPose(Pose2d pose)`

**Location:** [Vision.java](../src/main/java/frc/robot/Vision.java#L187)

**What it resets:**
- `visionSim.resetRobotPose(pose)` - Clears the VisionSystemSim's internal pose history

**Purpose:** The VisionSystemSim maintains a pose history for latency compensation. When you teleport the robot, old pose history becomes invalid and can cause erratic vision behavior. This method clears that history.

**Called by:**
- `PhotonVisionSim.resetAllPoses()` (via `setVision()` wiring in `Robot.java`)

---

## Current Reset Chain

```
PhotonVisionSim.resetAllPoses(pose)
    ├── groundTruthPose = pose                     // ✓ Ground truth
    ├── drivetrain.resetPose(pose)                 // ✓ Pose estimator
    │       └── (CTRE internally handles sim)      // ✓ Handled by CTRE
    └── vision.resetSimPose(pose)                  // ✓ Vision sim history
```

---

## Wiring

In `Robot.java`, the Vision instance is connected to PhotonVisionSim:

```java
public Robot() {
    m_robotContainer = new RobotContainer();
    m_vision = new Vision(m_robotContainer.drivetrain::addVisionMeasurement);

    // Connect vision to PhotonVisionSim so pose resets also reset the vision system
    if (m_robotContainer.visionSim != null) {
        m_robotContainer.visionSim.setVision(m_vision);
    }
}
```

---

## Summary Table

| Component | Reset Method | What It Resets | Called on Reset? |
|-----------|--------------|----------------|------------------|
| PhotonVisionSim | `resetAllPoses()` | Ground truth pose | ✓ Entry point |
| CommandSwerveDrivetrain | `resetPose()` | Pose estimator + CTRE sim | ✓ Yes |
| Vision | `resetSimPose()` | VisionSystemSim pose history | ✓ Yes |
