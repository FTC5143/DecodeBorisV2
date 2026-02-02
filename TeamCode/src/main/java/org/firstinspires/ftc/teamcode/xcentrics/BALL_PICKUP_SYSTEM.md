# Autonomous Ball Pickup with Dual Camera Vision System

## Overview

This system implements an advanced autonomous ball pickup functionality for FTC robots using a sophisticated dual-camera pipeline architecture. It combines AprilTag-based localization with real-time ball detection for accurate navigation and pickup.

## System Architecture

### Dual Pipeline Design

```
┌─────────────────────────────────────────────────────┐
│          Webcam Input (Single Hardware)              │
└─────────────────────────────────────────────────────┘
         │                                │
         ▼                                ▼
┌──────────────────────┐      ┌──────────────────────┐
│  AprilTag Pipeline   │      │ Ball Detection Pipe  │
│  (Continuous Pose)   │      │ (Color HSV-based)    │
└──────────────────────┘      └──────────────────────┘
         │                                │
         ▼                                ▼
┌──────────────────────┐      ┌──────────────────────┐
│  Pose Estimation     │      │  Field Coordinate    │
│  (Robot Location)    │      │  Conversion          │
└──────────────────────┘      └──────────────────────┘
         │                                │
         └────────────┬───────────────────┘
                      ▼
         ┌──────────────────────────┐
         │  Pathfinding & Control   │
         │  (Pedro Pathing)         │
         └──────────────────────────┘
```

## Key Components

### 1. DualCamera (`DualCamera.java`)

Advanced camera management system that runs two concurrent vision pipelines:

**Features:**
- Simultaneous AprilTag detection and ball detection
- Concurrent VisionPortal instances for reduced latency
- Automatic pipeline switching based on processing load
- Real-time coordinate transformation (pixel → field coordinates)
- Integrated telemetry for debugging

**Methods:**
```java
// Get detected balls in field coordinates
List<BallTarget> getDetectedBalls()

// Get robot pose from AprilTag
Pose getPose()

// Find closest ball
BallTarget getClosestBall()

// Configure ball detection color range (HSV)
void setBallColorRange(int hMin, int hMax, int sMin, int sMax, int vMin, int vMax)

// Check system status
boolean hasAprilTagLock()
boolean hasBallDetections()
```

### 2. BallDetectionProcessor (`BallDetectionProcessor.java`)

OpenCV-based real-time ball detection using HSV color space:

**Algorithm:**
1. Convert frame from BGR to HSV color space
2. Create mask for target color range
3. Find contours using edge detection
4. Filter by area threshold (eliminates noise)
5. Calculate center and confidence for each detection

**Output:**
```java
public class BallDetection {
    double x;           // Center X (pixels)
    double y;           // Center Y (pixels)
    double width;       // Width (pixels)
    double height;      // Height (pixels)
    double area;        // Area (square pixels)
    double confidence;  // 0.0 - 1.0
}
```

### 3. BallTarget (`BallTarget.java`)

Represents detected balls in field coordinates:

```java
double fieldX;      // Field X (inches)
double fieldY;      // Field Y (inches)
double confidence;  // Detection confidence
```

### 4. BallPathPlanner (`BallPathPlanner.java`)

Optimized multi-ball pathfinding using nearest-neighbor algorithm:

**Features:**
- Optimal ball pickup sequence (minimizes travel distance)
- Smooth bezier curve generation
- Distance and time estimation
- Return path generation

**Key Methods:**
```java
// Create path to single ball
PathChain createPathToBall(Pose currentPose, BallTarget target)

// Create optimized multi-ball path
List<PathChain> createMultiBallPath(Pose currentPose, List<BallTarget> balls)

// Estimate total operation time
double estimatePickupTime(Pose startPose, List<BallTarget> balls, double robotSpeed)
```

## Autonomous OpModes

### 1. BallPickupAuto (`BallPickupAuto.java`)

Basic autonomous mode with state machine control:

**States:**
```
INITIALIZE → LOCALIZE → SEARCH_FOR_BALL → APPROACH_BALL → PICKUP → RETURN_TO_START → COMPLETE
```

**Features:**
- Simple, easy-to-understand logic
- Single ball pickup capability
- Configurable target count
- AprilTag-based pose correction

### 2. AdvancedBallPickupAuto (`AdvancedBallPickupAuto.java`)

Advanced autonomous mode with optimized pathfinding:

**States:**
```
INITIALIZE → LOCALIZE → DETECT_BALLS → PLAN_ROUTE → EXECUTE_PICKUPS → SCORE → COMPLETE
```

**Features:**
- Multi-ball pickup with optimal path planning
- Continuous pose estimation during movement
- Performance telemetry
- Curved bezier path generation for smooth approach

## Coordinate Systems and Calibration

### Pixel to Field Conversion

The system converts camera pixel coordinates to field coordinates using camera calibration:

```
Calibration Parameters:
- Focal Length X (fx): 578.272 pixels
- Focal Length Y (fy): 578.272 pixels
- Principal Point X (cx): 402.145 pixels
- Principal Point Y (cy): 221.506 pixels
- Camera Height: 12.0 inches (mounted pointing down)
- Image Frame: 640x480 (OV5648)
```

**Conversion Formula:**
```
normalizedX = (pixelX - cx) / fx
normalizedY = (pixelY - cy) / fy
groundDistance = cameraHeight / tan(atan(normalizedY))
fieldX = groundDistance * normalizedX
fieldY = groundDistance
```

### Adjusting Calibration

To improve accuracy, measure your camera's actual parameters:

1. **Mount camera at known height**
2. **Place calibration pattern (checkerboard) at known distance**
3. **Capture images and use OpenCV calibration tools**
4. **Update values in DualCamera class:**

```java
private static final double FOCAL_LENGTH_X = 578.272;
private static final double FOCAL_LENGTH_Y = 578.272;
private static final double PRINCIPAL_POINT_X = 402.145;
private static final double PRINCIPAL_POINT_Y = 221.506;
private static final double CAMERA_HEIGHT_INCHES = 12.0;
```

## Ball Detection Tuning

### Color Range Presets

The system includes HSV color range presets:

```java
// Yellow (15-40, 100-255, 100-255)
dualCamera.setBallColorRange(15, 40, 100, 255, 100, 255);

// Orange (5-25, 130-255, 100-255)
dualCamera.setBallColorRange(5, 25, 130, 255, 100, 255);

// Red (0-15, 100-255, 100-255)
dualCamera.setBallColorRange(0, 15, 100, 255, 100, 255);
```

### Fine-Tuning

Use **CameraDebugTeleOp** to adjust HSV values in real-time:

```
Controls:
- X: Cycle through color presets
- DPAD UP/DOWN: Adjust V (brightness) range
- DPAD LEFT/RIGHT: Adjust minimum ball area threshold
```

**Tips:**
1. Start with preset for your ball color
2. Reduce `vMin` if balls appear dark
3. Increase `vMax` if lighting is bright
4. Increase `minBallArea` to reduce noise
5. Test in actual field lighting conditions

## Integration with Existing Systems

### Adding to Your Robot

1. **Import DualCamera in LiveRobot:**
```java
public DualCamera dualCamera;

// In constructor
dualCamera = new DualCamera(this);
```

2. **Register hardware in OpMode:**
```java
dualCamera = new DualCamera(robot);
dualCamera.registerHardware(hardwareMap);
robot.registerComponent(dualCamera);
```

3. **Startup in OpMode:**
```java
dualCamera.startup();
// In update loop:
dualCamera.update(this);
```

4. **Shutdown in OpMode:**
```java
dualCamera.shutdown();
```

### Pedro Pathing Integration

The system integrates seamlessly with Pedro Pathing:

```java
// Get optimized path
PathChain path = pathPlanner.createPathToBall(currentPose, ballTarget);

// Follow path
robot.follower.followPath(path, true);

// Check completion
while(robot.follower.isBusy()) {
    // Poll ball detections
}
```

## Performance Considerations

### Processing Load

- **AprilTag Pipeline**: ~15-20 ms per frame
- **Ball Detection Pipeline**: ~10-15 ms per frame
- **Total Latency**: ~30-40 ms with concurrent processing

### Optimization Strategies

1. **Frame Rate Management**
   - Lower resolution for faster processing
   - Adjust processor frame rate settings

2. **Pipeline Scheduling**
   - Alternate between pipelines to reduce peak load
   - Use DualCamera's built-in scheduling

3. **Detection Thresholds**
   - Increase `minBallArea` to reduce processing
   - Limit `maxBalls` tracked simultaneously

## Troubleshooting

### AprilTag Detection Issues

**No AprilTag Lock**
- Ensure AprilTags are visible to camera
- Verify lighting conditions
- Check camera focus
- Adjust camera pose calibration

**Inconsistent Pose**
- Multiple AprilTags improves accuracy
- Increase detection threshold
- Verify AprilTag spacing

### Ball Detection Issues

**Too Many False Detections**
- Increase `minBallArea` threshold
- Narrow HSV color range
- Verify lighting is consistent
- Use CameraDebugTeleOp to tune

**Balls Not Detected**
- Check HSV color range using CameraDebugTeleOp
- Ensure adequate lighting
- Verify camera focus
- Check ball color matches preset

**Jittery Ball Position**
- Increase filtering/averaging
- Reduce noise in HSV range
- Improve lighting consistency

## Usage Examples

### Basic Ball Pickup

```java
@Autonomous(name = "Simple Pickup")
public class SimplePickupAuto extends LiveAutoBase {
    private DualCamera camera;
    
    @Override
    public void on_init() {
        camera = new DualCamera(robot);
        camera.registerHardware(hardwareMap);
    }
    
    @Override
    public void on_start() {
        camera.startup();
    }
    
    @Override
    public void on_loop() {
        robot.update();
        camera.update(this);
        
        if (camera.hasBallDetections()) {
            BallTarget closest = camera.getClosestBall();
            // Navigate to ball...
        }
    }
}
```

### Multi-Ball Optimized Pickup

```java
BallPathPlanner planner = new BallPathPlanner(robot.follower);
planner.setStartPose(startPosition);
planner.setScoringPose(scorePosition);

List<BallTarget> balls = camera.getDetectedBalls();
List<PathChain> paths = planner.createMultiBallPath(
    robot.follower.getPose(), 
    balls
);

// Follow optimized paths...
for (PathChain path : paths) {
    robot.follower.followPath(path, true);
    // Execute pickup
}
```

## Advanced Features

### Custom Color Detection

Implement custom ball colors:

```java
// For purple balls
dualCamera.setBallColorRange(
    280,  // Hue min (in degrees, then scaled)
    320,  // Hue max
    100,  // Saturation min
    255,  // Saturation max
    100,  // Value min
    255   // Value max
);
```

### Confidence-Based Filtering

Filter detections by confidence:

```java
List<BallTarget> highConfidenceBalls = camera.getDetectedBalls()
    .stream()
    .filter(ball -> ball.confidence > 0.7)
    .collect(Collectors.toList());
```

### Distance-Based Filtering

Only track balls within range:

```java
Pose robotPose = robot.getRobotPose();
List<BallTarget> nearbyBalls = camera.getDetectedBalls()
    .stream()
    .filter(ball -> ball.distanceTo(robotPose) < 24.0) // 24 inches
    .collect(Collectors.toList());
```

## File Structure

```
xcentrics/
├── components/
│   └── live/
│       ├── Camera.java          (Original AprilTag-only)
│       └── DualCamera.java       (New dual-pipeline system)
├── vision/
│   ├── BallDetectionProcessor.java
│   └── BallTarget.java
├── util/
│   └── navigation/
│       └── BallPathPlanner.java
└── OpModes/
    ├── Auto/
    │   ├── BallPickupAuto.java
    │   └── AdvancedBallPickupAuto.java
    └── TeleOp/
        └── CameraDebugTeleOp.java
```

## Testing Checklist

- [ ] AprilTag detection working reliably
- [ ] Ball detection tuned for your field lighting
- [ ] Camera calibration values verified
- [ ] Coordinate conversion accurate
- [ ] Single ball pickup working
- [ ] Multi-ball optimization functional
- [ ] Path planning smooth and efficient
- [ ] Intake mechanism properly synchronized
- [ ] Return navigation working
- [ ] Full autonomous cycle tested

## References

- OpenCV Documentation: https://docs.opencv.org/
- FTC Vision Portal: https://ftc-docs.firstinspires.org/
- AprilTag Detection: https://ftc-docs.firstinspires.org/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
- Pedro Pathing: https://github.com/pedropathing/Pedro-Pathing
