# Ball Pickup System - Quick Reference Card

## Installation Checklist

```
[ ] Copy all Java files to appropriate directories
[ ] Update LiveRobot.java (already done)
[ ] Add imports to your OpModes
[ ] Configure your starting positions
[ ] Choose detection mode: HSV or Husky Lens
[ ] Calibrate camera (optional, defaults provided)
[ ] Test with CameraDebugTeleOp (for HSV) or test Husky Lens I2C
[ ] Run SimpleBallPickupExample (HSV) or HuskyLensBallPickupExample (Husky Lens)
[ ] Deploy to competition
```

## Essential Code Snippets

### Basic Usage

```java
// In OpMode
DualCamera camera = new DualCamera(robot);
camera.registerHardware(hardwareMap);
camera.startup();

// Choose detection mode
camera.useHuskyLensDetection();        // Use Husky Lens hardware
camera.useCameraRightMounted(6.0);     // 6 inches right of center

// OR
camera.useHSVDetection();              // Use OpenCV HSV
camera.setBallColorRange(15, 40, 100, 255, 100, 255);

// In update loop
camera.update(this);

// Get data (same for both modes)
if (camera.hasBallDetections()) {
    BallTarget ball = camera.getClosestBall();
    Pose ballPose = ball.toPose();
}

// Get pose
Pose robotPose = camera.getPose();
```

### Configure Camera Offset (Husky Lens)

```java
// Preset configurations
camera.useCameraCenterMounted();        // No offset
camera.useCameraLeftMounted(6.0);       // 6" left
camera.useCameraRightMounted(6.0);      // 6" right
camera.useCameraForwardMounted(8.0);    // 8" forward

// Manual configuration
CameraOffsetConfig config = new CameraOffsetConfig();
config.setCameraPosition(-6.0, 0, 12.0); // X, Y, Z offsets
camera.setCameraOffsetConfig(config);
```

### Path Planning

```java
BallPathPlanner planner = new BallPathPlanner(robot.follower);
planner.setStartPose(new Pose(0, 0, 0));
planner.setScoringPose(new Pose(50, 110, 0));

// Single ball
PathChain path = planner.createPathToBall(currentPose, ball);

// Multiple balls (optimized)
List<PathChain> paths = planner.createMultiBallPath(currentPose, balls);
```

## Class Reference

### DualCamera

**Startup/Shutdown:**
```java
void registerHardware(HardwareMap hardwareMap)
void startup()
void shutdown()
void update(OpMode opMode)
```

**Vision Access:**
```java
List<BallTarget> getDetectedBalls()
BallTarget getClosestBall()
int getBallCount()
Pose getPose()
List<AprilTagDetection> currentDetections
```

**Configuration:**
```java
void setBallColorRange(int hMin, int hMax, int sMin, int sMax, 
                       int vMin, int vMax)
void setMinBallArea(double minArea)
```

**Status:**
```java
boolean hasAprilTagLock()
boolean hasBallDetections()
```

### BallTarget

```java
double fieldX;           // X position (inches)
double fieldY;           // Y position (inches)
double confidence;       // 0.0 - 1.0
long timestamp;          // Detection time

Pose toPose()           // Convert to Pedro Pose
double distanceTo(Pose)  // Distance from pose
```

### BallPathPlanner

```java
void setStartPose(Pose)
void setScoringPose(Pose)
void setApproachDistance(double)

PathChain createPathToBall(Pose, BallTarget)
PathChain createCurvedBallApproach(Pose, BallTarget)
List<PathChain> createMultiBallPath(Pose, List<BallTarget>)
PathChain createReturnPath(Pose)

double calculateTotalDistance(Pose, List<BallTarget>)
double estimatePickupTime(Pose, List<BallTarget>, double speed)
```

## OpenCV HSV Value Chart

```
HUE (0-180 in OpenCV):
Red        0-15, 165-180
Orange     5-35
Yellow     15-40
Green      40-80
Cyan       80-100
Blue       100-130
Magenta    130-165

SATURATION (0-255):
Pale       0-50
Normal     100-150
Vibrant    150-255

VALUE (0-255):
Dark       0-85
Medium     85-170
Bright     170-255
```

## Telemetry Diagnostics

```java
// Check AprilTag status
telemetry.addData("AprilTag Lock", camera.hasAprilTagLock());
telemetry.addData("Tags Detected", camera.currentDetections.size());

// Check ball status
telemetry.addData("Balls Detected", camera.getBallCount());
telemetry.addData("Confidence", camera.getClosestBall().confidence);

// Check position
Pose p = camera.getPose();
telemetry.addData("Position", "%.1f, %.1f", p.getX(), p.getY());

// Check performance
telemetry.addData("Update Freq", robot.update_freq);
```

## Common Errors and Fixes

| Error | Cause | Fix |
|-------|-------|-----|
| "No AprilTag lock" | Tags not visible | Position robot to see tags |
| "No balls detected" | Wrong HSV range | Run CameraDebugTeleOp to tune |
| "Ball position wrong" | Bad calibration | Verify camera height/focal length |
| "Robot jerky movement" | High latency | Increase min ball area threshold |
| "Mission incomplete" | Path planning issue | Check scoring position coordinates |

## Performance Targets

```
AprilTag Detection:     30 FPS, <20ms latency
Ball Detection:         30 FPS, <15ms latency
Overall Latency:        <40ms
Single Ball Cycle:      <10 seconds
Multi-Ball Cycle:       <25 seconds
Accuracy:               ±2-4 inches
CPU Usage:              <20%
```

## Field Coordinates Setup

```java
// Blue Alliance
Pose BLUE_START = new Pose(27.5, 131.6, Math.toRadians(144));
Pose BLUE_SCORE = new Pose(50, 110, 0);

// Red Alliance
Pose RED_START = new Pose(-27.5, 131.6, Math.toRadians(36));
Pose RED_SCORE = new Pose(-50, 110, 180);
```

## OpMode Selection Guide

| OpMode | Complexity | Use Case |
|--------|-----------|----------|
| **SimpleBallPickupExample** | Low | Learning, first test |
| **BallPickupAuto** | Medium | Simple pickup scoring |
| **AdvancedBallPickupAuto** | High | Multi-ball, optimized |
| **CameraDebugTeleOp** | Low | Tuning HSV values |

## State Machine States

### BallPickupAuto
```
INITIALIZE → LOCALIZE → SEARCH → APPROACH → PICKUP → RETURN → COMPLETE
```

### AdvancedBallPickupAuto
```
INITIALIZE → LOCALIZE → DETECT → PLAN → EXECUTE → SCORE → COMPLETE
```

## Debugging Workflow

1. **Start with CameraDebugTeleOp**
   - Verify AprilTag lock
   - Tune HSV ranges
   - Confirm ball detection

2. **Run SimpleBallPickupExample**
   - Test basic autonomous
   - Verify localization
   - Confirm pickup execution

3. **Deploy AdvancedBallPickupAuto**
   - Test multi-ball collection
   - Verify path optimization
   - Confirm scoring

## Memory Usage

```
DualCamera:           ~2 MB (vision buffers)
Detections List:      ~100 KB (typically)
Path Planning:        ~500 KB
Telemetry:            ~1 MB
Total Overhead:       ~4-5 MB
```

## Network and Hardware

```
USB Bandwidth:        Webcam ~10 Mbps
Processing:           2 threads (vision, main)
Vision Update Rate:   30 FPS (33ms per frame)
Decision Rate:        100 Hz (10ms loop)
Motor Update Rate:    100 Hz (10ms loop)
```

## Key Imports

```java
import org.firstinspires.ftc.teamcode.xcentrics.components.live.DualCamera;
import org.firstinspires.ftc.teamcode.xcentrics.vision.BallTarget;
import org.firstinspires.ftc.teamcode.xcentrics.vision.BallDetectionProcessor;
import org.firstinspires.ftc.teamcode.xcentrics.util.navigation.BallPathPlanner;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
```

## File Locations (Relative to TeamCode)

```
src/main/java/org/firstinspires/ftc/teamcode/xcentrics/
├── components/live/DualCamera.java
├── vision/BallDetectionProcessor.java
├── vision/BallTarget.java
├── util/navigation/BallPathPlanner.java
├── OpModes/Auto/BallPickupAuto.java
├── OpModes/Auto/AdvancedBallPickupAuto.java
├── OpModes/Auto/examples/SimpleBallPickupExample.java
└── OpModes/TeleOp/CameraDebugTeleOp.java
```

## Default Camera Calibration

```java
FOCAL_LENGTH_X        = 578.272 pixels
FOCAL_LENGTH_Y        = 578.272 pixels
PRINCIPAL_POINT_X     = 402.145 pixels
PRINCIPAL_POINT_Y     = 221.506 pixels
CAMERA_HEIGHT         = 12.0 inches
FRAME_WIDTH           = 640 pixels
FRAME_HEIGHT          = 480 pixels
```

## Testing Sequence

```
Day 1: Setup
  □ Copy files to project
  □ Compile without errors
  □ Load CameraDebugTeleOp

Day 2: Calibration
  □ Position robot near AprilTag
  □ Verify AprilTag lock
  □ Place ball in view
  □ Adjust HSV range
  □ Record optimal values

Day 3: Basic Testing
  □ Load SimpleBallPickupExample
  □ Run with 1 ball on field
  □ Verify complete cycle

Day 4: Advanced Testing
  □ Load AdvancedBallPickupAuto
  □ Run with 2-3 balls
  □ Verify multi-ball collection
  □ Confirm scoring

Day 5: Competition Ready
  □ Fine-tune for field conditions
  □ Test in final position
  □ Backup configuration values
```

## Emergency Options

**If AprilTag lock is unavailable:**
```java
// Use last known pose + odometry
Pose fallbackPose = robot.lastPose;
robot.follower.setPose(fallbackPose);
```

**If ball detection fails:**
```java
// Use manual position or timeout
if (!camera.hasBallDetections() && timer.seconds() > timeout) {
    currentState = State.SCORE;
}
```

**If path planning fails:**
```java
// Use simple linear path
PathChain simplePath = robot.follower.pathBuilder()
    .addPath(new BezierLine(current, target))
    .setLinearHeadingInterpolation(current.getHeading(), target.getHeading())
    .build();
```

## Best Practices

✓ Always call `camera.update(this)` in update loop
✓ Check `hasBallDetections()` before `getClosestBall()`
✓ Verify `hasAprilTagLock()` before critical operations
✓ Use `CameraDebugTeleOp` to tune vision before autonomous
✓ Test on actual field with competition lighting
✓ Log configuration values for reproducibility
✓ Implement timeout handling for lost detections
✓ Use telemetry for real-time debugging

## Common Customizations

**Change camera mount position:**
```java
private final Position cameraPosition = new Position(DistanceUnit.MM,
    X_OFFSET, Y_OFFSET, Z_OFFSET, 0);
```

**Adjust pickup sequence:**
```java
robot.intake.setPower(1.0);      // Start
halt(1.5);                         // Run for time
robot.intake.setPower(0.0);       // Stop
```

**Modify field coordinates:**
```java
Pose START = new Pose(x, y, heading);
Pose SCORE = new Pose(x, y, heading);
```

---

**Print this quick reference for easy competition pit access!**
