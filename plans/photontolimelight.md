- Photon vision stores camera datapoints in this structure: PhotonTrackedTarget
- Limelighthelpers class stores camera datapoints in this structure: RawDetection


I'd like in PhotonToLimelight class to add code so that each PhotonTrackedTarget is written back to NetworkTables in the exact place and format that limelighthelpers.java reads it in.






# NetworkTables API

Limelight OS features a NetworkTables 4 Client. It auto-connects to the NetworkTables 4 Server running on FRC Robots based on the Team Number / ID configured in the Settings UI.

All data is published to a table that matches the device name (e.g. "limelight"). If a hostname / nickname is assigned to your camera, the table name will match the full limelight name (e.g. "limelight-top").

LimelightLib WPIJava and LimelightLib WPICPP interact with Limelight devices via NetworkTables.

## Basic Targeting Data

Use the following code:

```java
NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDouble(0);
```

to retrieve this data:

| Key | Type | Description |
|-----|------|-------------|
| tv | int | 1 if valid target exists. 0 if no valid targets exist |
| tx | double | Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees) |
| ty | double | Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees) |
| txnc | double | Horizontal Offset From Principal Pixel To Target (degrees) |
| tync | double | Vertical Offset From Principal Pixel To Target (degrees) |
| ta | double | Target Area (0% of image to 100% of image) |
| tl | double | The pipeline's latency contribution (ms). Add to "cl" to get total latency. |
| cl | double | Capture pipeline latency (ms). Time between the end of the exposure of the middle row of the sensor to the beginning of the tracking pipeline. |
| t2d | double | Array containing several values for matched-timestamp statistics: [targetValid, targetCount, targetLatency, captureLatency, tx, ty, txnc, tync, ta, tid, targetClassIndexDetector, targetClassIndexClassifier, targetLongSidePixels, targetShortSidePixels, targetHorizontalExtentPixels, targetVerticalExtentPixels, targetSkewDegrees] |
| getpipe | int | True active pipeline index of the camera (0 .. 9) |
| getpipetype | string | Pipeline Type e.g. "pipe_color" |
| json | string | Full JSON dump of targeting results. Must be enabled per-pipeline in the 'output' tab |
| tc | doubleArray | Get the average BGR color underneath the crosshair region as a NumberArray [B, G, R] |
| hb | double | Heartbeat value. Increases once per frame, resets at 2 billion |
| hw | doubleArray | Hardware metrics [cpu_temp_celsius, cpu_usage, ram_usage_percent, fps] |
| crosshairs | doubleArray | 2D Crosshairs [cx0, cy0, cx1, cy1] |
| tcclass | string | Name of classifier pipeline's computed class |
| tdclass | string | Name of detector pipeline's primary detection |

## AprilTag and 3D Data

Use the following code:

```java
NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDoubleArray(new double[6]);
```

to retrieve this data:

| Key | Type | Description |
|-----|------|-------------|
| botpose | doubleArray | Robot transform in field-space. Translation (X,Y,Z) in meters, Rotation(Roll,Pitch,Yaw) in degrees, total latency (cl+tl), tag count, tag span, average tag distance from camera, average tag area (percentage of image) |
| botpose_wpiblue | doubleArray | Robot transform in field-space (blue driverstation WPILIB origin). Translation (X,Y,Z) in meters, Rotation(Roll,Pitch,Yaw) in degrees, total latency (cl+tl), tag count, tag span, average tag distance from camera, average tag area (percentage of image) |
| botpose_wpired | doubleArray | Robot transform in field-space (red driverstation WPILIB origin). Translation (X,Y,Z) in meters, Rotation(Roll,Pitch,Yaw) in degrees, total latency (cl+tl), tag count, tag span, average tag distance from camera, average tag area (percentage of image) |
| botpose_orb | doubleArray | Robot transform in field-space (Megatag2). Translation (X,Y,Z) in meters, Rotation(Roll,Pitch,Yaw) in degrees, total latency (cl+tl), tag count, tag span, average tag distance from camera, average tag area (percentage of image) |
| botpose_orb_wpiblue | doubleArray | Robot transform in field-space (Megatag2) (blue driverstation WPILIB origin). Translation (X,Y,Z) in meters, Rotation(Roll,Pitch,Yaw) in degrees, total latency (cl+tl), tag count, tag span, average tag distance from camera, average tag area (percentage of image) |
| botpose_orb_wpired | doubleArray | Robot transform in field-space (Megatag2) (red driverstation WPILIB origin). Translation (X,Y,Z) in meters, Rotation(Roll,Pitch,Yaw) in degrees, total latency (cl+tl), tag count, tag span, average tag distance from camera, average tag area (percentage of image) |
| camerapose_targetspace | doubleArray | 3D transform of the camera in the coordinate system of the primary in-view AprilTag (array (6)) [tx, ty, tz, pitch, yaw, roll] (meters, degrees) |
| targetpose_cameraspace | doubleArray | 3D transform of the primary in-view AprilTag in the coordinate system of the Camera (array (6)) [tx, ty, tz, pitch, yaw, roll] (meters, degrees) |
| targetpose_robotspace | doubleArray | 3D transform of the primary in-view AprilTag in the coordinate system of the Robot (array (6)) [tx, ty, tz, pitch, yaw, roll] (meters, degrees) |
| botpose_targetspace | doubleArray | 3D transform of the robot in the coordinate system of the primary in-view AprilTag (array (6)) [tx, ty, tz, pitch, yaw, roll] (meters, degrees) |
| camerapose_robotspace | doubleArray | 3D transform of the camera in the coordinate system of the robot (array (6)) |
| tid | int | ID of the primary in-view AprilTag |
| stddevs | doubleArray | MegaTag Standard Deviations [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x, MT2y, MT2z, MT2roll, MT2pitch, MT2yaw] |
| camerapose_robotspace_set | doubleArray | SET the camera's pose in the coordinate system of the robot. |
| priorityid | int | SET the required ID for tx/ty targeting. Ignore other targets. Does not affect localization |
| robot_orientation_set | doubleArray | SET Robot Orientation and angular velocities in degrees and degrees per second [yaw,yawrate,pitch,pitchrate,roll,rollrate] |
| fiducial_id_filters_set | doubleArray | Override valid fiducial ids for localization (array) |
| fiducial_offset_set | doubleArray | SET the 3D Point of Interest Offset [x,y,z] |
| fiducial_downscale_set | int | Override AprilTag detection downscale. 0=pipeline control, 1=1x (no downscale), 2=1.5x, 3=2x, 4=3x, 5=4x |

## Camera Controls

Use the following code:

```java
NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").setNumber(<value>);
```

to set this data:

| Key | Value | Description |
|-----|-------|-------------|
| ledMode | 0 | Use the LED Mode set in the current pipeline |
| | 1 | Force off |
| | 2 | Force blink |
| | 3 | Force on |
| | 4 | Force on (left LEDs only) |
| | 5 | Force on (right LEDs only) |
| | 6 | Bounce halves |
| | 7 | Force blink (left LEDs only) |
| | 8 | Force blink (right LEDs only) |
| pipeline | 0..9 | Select pipeline 0..9 |
| stream | 0 | Standard - Side-by-side streams if a webcam is attached to Limelight |
| | 1 | PiP Main - The secondary camera stream is placed in the lower-right corner of the primary camera stream |
| | 2 | PiP Secondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream |
| snapshot | | Takes a snapshot. Increment this value to trigger a capture (e.g., 0→1→2→3). Rate-limited to once every 10 frames. |
| crop | [0] | X0 - Min or Max X value of crop rectangle (-1 to 1) |
| | [1] | X1 - Min or Max X value of crop rectangle (-1 to 1) |
| | [2] | Y0 - Min or Max Y value of crop rectangle (-1 to 1) |
| | [3] | Y1 - Min or Max Y value of crop rectangle (-1 to 1) |
| keystone_set | [0] | Horizontal keystone (-0.95 to 0.95) |
| | [1] | Vertical keystone (-0.95 to 0.95) |
| throttle_set | int | We recommend setting this to 100-200 while disabled. Sets number of frames to skip between processed frames to reduce temperature rise. Outputs are not zeroed during skipped frames. |

## Video Recording Controls

| Key | Value | Description |
|-----|-------|-------------|
| rewind_enable_set | double | Controls rewind buffer recording. 1 = recording enabled, 0 = recording paused. |
| capture_rewind | [0] | Counter - increment this value to trigger a capture |
| | [1] | Duration in seconds (max 165 via NT) |

Example:

```java
double[] cropValues = new double[4];
cropValues[0] = -1.0;
cropValues[1] = 1.0;
cropValues[2] = -1.0;
cropValues[3] = 1.0;
NetworkTableInstance.getDefault().getTable("limelight").getEntry("crop").setDoubleArray(cropValues);
```

## IMU Data

| Key | Type | Description |
|-----|------|-------------|
| imu | doubleArray | IMU data output [robot_yaw, roll, pitch, internal_yaw, roll_rate, pitch_rate, yaw_rate, accel_x, accel_y, accel_z] (10 elements). Angles in degrees, rates in deg/s. |

## IMU Controls

| Key | Type | Description |
|-----|------|-------------|
| imumode_set | int | Set the imumode. 0 - use external imu, 1 - use external imu, seed internal imu, 2 - use internal, 3 - use internal with MT1 assisted convergence, 4 - use internal IMU with external IMU assisted convergence |
| imuassistalpha_set | double | Complementary filter alpha / strength. Higher values will cause the internal imu to converge on assist source more rapidly. The default is set to a low value 0.001 because we now trust the internal IMU more than before. Assist modes are built to very gently "tug" the internal imu towards the chosen assist source. |

## Python

Python scripts allow for arbitrary inbound and outbound data.

| Key | Description |
|-----|-------------|
| llpython | NumberArray sent by python scripts. This is accessible within robot code. |
| llrobot | NumberArray sent by the robot. This is accessible within python SnapScripts. |

## Raw Data

### Corners

Enable "send contours" in the "Output" tab to stream corner coordinates:

| Key | Description |
|-----|-------------|
| tcornxy | Number array of corner coordinates [x0,y0,x1,y1......] |

### Raw Targets

Limelight posts three raw contours to NetworkTables that are not influenced by your grouping mode. That is, they are filtered with your pipeline parameters, but never grouped. X and Y are returned in normalized screen space (-1 to 1) rather than degrees.

| Key | Format |
|-----|--------|
| rawtargets | [txnc,tync,ta,txnc2,tync2,ta2....] |

### Raw Fiducials

Get all valid (unfiltered) fiducials

| Key | Format |
|-----|--------|
| rawfiducials | [id, txnc, tync, ta, distToCamera, distToRobot, ambiguity, id2.....] |

### Raw Detections

Get all valid (unfiltered) neural detection results

| Key | Format |
|-----|--------|
| rawdetections | [id, txnc, tync, ta, corner0x, corner0y, corner1x, corner1y, corner2x, corner2y, corner3x, corner3y, id2.....] |

### Raw Barcodes

Get all valid (unfiltered) barcode results

| Key | Format |
|-----|--------|
| rawbarcodes | string array of barcode data |