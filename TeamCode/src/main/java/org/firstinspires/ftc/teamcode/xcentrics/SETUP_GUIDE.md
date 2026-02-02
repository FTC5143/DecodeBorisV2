# Ball Pickup System - Setup and Configuration Guide

## Quick Start

### 1. Basic Integration (5 minutes)

Add DualCamera to your autonomous OpMode:

```java
// In your OpMode class
private DualCamera dualCamera;

@Override
public void on_init() {
    // ... existing initialization ...
    
    dualCamera = new DualCamera(robot);
    dualCamera.registerHardware(hardwareMap);
    robot.registerComponent(dualCamera);
    
    // Configure for yellow balls
    dualCamera.setBallColorRange(15, 40, 100, 255, 100, 255);
    dualCamera.setMinBallArea(150);
}

@Override
public void on_start() {
    // ... existing startup ...
    dualCamera.startup();
}

@Override
public void on_loop() {
    robot.update();
    dualCamera.update(this);
    
    // Use detected balls
    if (dualCamera.hasBallDetections()) {
        BallTarget closestBall = dualCamera.getClosestBall();
        telemetry.addData("Ball", closestBall.toString());
    }
}

@Override
public void on_stop() {
    dualCamera.shutdown();
}
```

### 2. Tune Ball Detection (10 minutes)

1. Load **CameraDebugTeleOp** OpMode
2. Connect to RC phone
3. Initialize OpMode
4. Press START
5. Use gamepad controls to adjust:
   - X: Cycle color presets (Yellow → Orange → Red)
   - DPAD UP/DOWN: Adjust brightness range
   - DPAD LEFT/RIGHT: Adjust minimum area threshold
6. Watch telemetry for ball detections
7. Note the HSV values that work best

### 3. Calibrate Camera (20 minutes)

**Option A: Use Default Values**
- Default calibration is for standard OV5648 camera
- Mounted pointing down at 12" height
- May have ±10% accuracy

**Option B: Measure Your Camera**

1. Mount camera on robot at known height (e.g., 12 inches)
2. Place target board with known dimensions at distance
3. Take calibration photos with different distances
4. Use OpenCV calibration tools to determine:
   - Focal length (fx, fy)
   - Principal point (cx, cy)
5. Update in DualCamera.java:

```java
private static final double FOCAL_LENGTH_X = YOUR_FX;
private static final double FOCAL_LENGTH_Y = YOUR_FY;
private static final double PRINCIPAL_POINT_X = YOUR_CX;
private static final double PRINCIPAL_POINT_Y = YOUR_CY;
private static final double CAMERA_HEIGHT_INCHES = YOUR_HEIGHT;
```

### 4. Test in Autonomous (15 minutes)

1. Load **AdvancedBallPickupAuto** OpMode
2. Place robot in field with AprilTags visible
3. Place 1 ball near robot
4. Start OpMode
5. Watch telemetry:
   - Should see "AprilTag Lock"
   - Should detect ball position
   - Should navigate to ball
6. Verify ball pickup mechanism engages
7. Verify return to scoring position

## Configuration Parameters

### Ball Detection

```java
// Color range (HSV)
dualCamera.setBallColorRange(
    15, 40,      // Hue min/max (0-180 in OpenCV HSV)
    100, 255,    // Saturation min/max
    100, 255     // Value (brightness) min/max
);

// Minimum detection area (filters noise)
dualCamera.setMinBallArea(150); // pixels²
```

### Common Color Ranges

```java
// Yellow balls
dualCamera.setBallColorRange(15, 40, 100, 255, 100, 255);

// Orange balls
dualCamera.setBallColorRange(5, 25, 130, 255, 100, 255);

// Red balls
dualCamera.setBallColorRange(0, 15, 100, 255, 100, 255);

// Blue balls
dualCamera.setBallColorRange(100, 130, 100, 255, 100, 255);

// Green balls
dualCamera.setBallColorRange(40, 80, 100, 255, 100, 255);
```

### Path Planner

```java
BallPathPlanner planner = new BallPathPlanner(robot.follower);

// Set starting position
planner.setStartPose(new Pose(27.5, 131.6, Math.toRadians(144)));

// Set scoring position
planner.setScoringPose(new Pose(50, 110, 0));

// Set how close to get before pickup
planner.setApproachDistance(6.0); // inches
```

### Autonomous OpMode

In **AdvancedBallPickupAuto.java**, configure:

```java
// Maximum balls to collect
private final int MAX_BALLS_TO_COLLECT = 3;

// Maximum detection time before moving on
private final double MAX_DETECTION_TIME = 4.0; // seconds

// Starting positions
private final Pose BLUE_START = new Pose(27.5, 131.6, Math.toRadians(144));
private final Pose RED_START = new Pose(-27.5, 131.6, Math.toRadians(36));
private final Pose SCORING_POSITION = new Pose(50, 110, 0);
```

## Camera Hardware Setup

### Physical Mount

Optimal configuration:
- **Position**: Center of robot, front-facing down
- **Height**: 10-15 inches above field
- **Angle**: 90° (perpendicular to field)
- **Focus Distance**: 12-36 inches
- **Protection**: Clear enclosure to prevent dust

### Hardware Configuration

In your `hardwareMap` configuration:

```json
{
  "cameraName": "Webcam 1",
  "hardwareType": "WebcamName",
  "note": "OV5648 or similar",
  "width": 640,
  "height": 480
}
```

### Lighting

- **Best Practice**: Consistent, overhead lighting
- **Avoid**: Direct sunlight, glare, shadows
- **Test**: Run detection in actual competition lighting
- **Fallback**: Use ball detection with adjustable brightness threshold

## Troubleshooting

### AprilTag Detection Not Working

**Symptom**: "Waiting for AprilTag lock" - never locks

**Solutions**:
1. Verify AprilTags are in view of camera
2. Ensure good lighting on AprilTags
3. Check camera focus (adjust manually if needed)
4. Verify robot is within 4 feet of tags
5. Ensure AprilTag library is correct for your game

**Debug**:
```java
telemetry.addData("AprilTag Detections", dualCamera.currentDetections.size());
for (AprilTagDetection det : dualCamera.currentDetections) {
    telemetry.addData("Tag ID", det.id);
}
```

### Ball Detection Not Working

**Symptom**: "Balls Detected: 0" even when balls visible

**Solutions**:
1. Run CameraDebugTeleOp to verify color range
2. Check lighting is adequate
3. Verify ball color matches preset
4. Try adjusting minimum area threshold
5. Test in actual competition field lighting

**Debug**:
```java
int rawBallCount = dualCamera.ballDetectionProcessor.getDetectedBalls().size();
telemetry.addData("Raw Detections", rawBallCount);

for (var ball : dualCamera.ballDetectionProcessor.getDetectedBalls()) {
    telemetry.addData("Pixel Position", "%.0f, %.0f", ball.x, ball.y);
    telemetry.addData("Area", "%.0f", ball.area);
}
```

### Coordinate Conversion Errors

**Symptom**: Ball position is consistently wrong

**Solutions**:
1. Verify camera calibration values
2. Check camera height measurement
3. Ensure camera is perpendicular to field
4. Test with known-distance targets
5. Consider field coordinate system (red vs blue alliance)

**Calibration Check**:
```java
// Place ball at known distance (e.g., 12 inches forward)
// Check if fieldX ≈ 0, fieldY ≈ 12
BallTarget detected = dualCamera.getClosestBall();
telemetry.addData("Expected", "0, 12");
telemetry.addData("Detected", "%.1f, %.1f", detected.fieldX, detected.fieldY);
```

### Performance Issues

**Symptom**: Slow movements, laggy robot

**Solutions**:
1. Reduce frame rate in vision processor
2. Increase minimum ball area to reduce processing
3. Alternate between pipelines (less frequent updates)
4. Use lower resolution (480p instead of 720p)
5. Disable telemetry updates during autonomous

**Performance Check**:
```java
long startTime = System.nanoTime();
dualCamera.update(this);
long elapsed = System.nanoTime() - startTime;
telemetry.addData("Update Time", "%.1f ms", elapsed / 1000000.0);
```

## Testing Progression

### Phase 1: Vision System (No Movement)
- [ ] OpMode initializes without errors
- [ ] Telemetry shows AprilTag detection
- [ ] Can toggle ball detection on/off
- [ ] Ball position updates when ball moves
- [ ] Coordinates are reasonable

### Phase 2: Manual Movement
- [ ] Load CameraDebugTeleOp
- [ ] Verify AprilTag lock with manual movement
- [ ] Test ball detection as you move
- [ ] Adjust HSV values for lighting conditions
- [ ] Record optimal HSV values

### Phase 3: Automated Navigation
- [ ] Load BallPickupAuto
- [ ] Robot localizes using AprilTag
- [ ] Robot detects ball
- [ ] Robot plans path (verify in telemetry)
- [ ] Robot follows path smoothly
- [ ] Robot stops at correct position

### Phase 4: Pickup Execution
- [ ] Intake activates at correct position
- [ ] Ball is collected
- [ ] Robot returns to start
- [ ] Complete cycle works end-to-end

### Phase 5: Multi-Ball Collection
- [ ] Place 2-3 balls on field
- [ ] Robot picks up nearest ball first
- [ ] Robot reoptimizes path for remaining balls
- [ ] All balls collected in optimal sequence
- [ ] Robot returns and scores

## Performance Benchmarks

Expected performance metrics:

| Metric | Target | Acceptable |
|--------|--------|-----------|
| AprilTag Lock Time | <2 sec | <5 sec |
| Ball Detection Latency | <100 ms | <200 ms |
| Path Planning Time | <500 ms | <1 sec |
| Coordinate Accuracy | ±2 in | ±4 in |
| Single Ball Cycle | <10 sec | <15 sec |
| Multi-Ball (3x) Cycle | <25 sec | <35 sec |
| Update Frequency | >20 Hz | >10 Hz |

## Optimization Tips

### Speed Improvements
1. Reduce detection area threshold
2. Pre-calculate paths during delays
3. Use bezier curves instead of line paths
4. Pre-load intake mechanism before reaching ball

### Accuracy Improvements
1. Increase AprilTag library size
2. Multiple ball detection passes
3. Averaging ball positions over frames
4. Kalman filtering for position smoothing

### Reliability Improvements
1. Fallback to odom if AprilTag lost
2. Multiple color presets for different lighting
3. Error recovery in autonomous loop
4. Timeout handling for lost balls

## Next Steps

1. **Install System**: Copy all files to your project
2. **Configure**: Update OpMode with your field positions
3. **Calibrate**: Run calibration procedures
4. **Test**: Follow testing progression above
5. **Tune**: Optimize parameters for your robot
6. **Deploy**: Use in competition

## Support Files

- `BALL_PICKUP_SYSTEM.md` - Detailed system documentation
- `SETUP_GUIDE.md` - This file
- `BallDetectionProcessor.java` - Vision processor
- `BallTarget.java` - Data class
- `DualCamera.java` - Main camera system
- `BallPathPlanner.java` - Path optimization
- `BallPickupAuto.java` - Simple autonomous
- `AdvancedBallPickupAuto.java` - Advanced autonomous
- `CameraDebugTeleOp.java` - Debugging/tuning

## Questions?

For issues or questions:
1. Check BALL_PICKUP_SYSTEM.md documentation
2. Run CameraDebugTeleOp to verify vision
3. Check telemetry for error messages
4. Review code comments in source files
5. Test individual components in isolation
