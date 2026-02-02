# Vision System Tuning Guide

Complete practical guide for tuning and optimizing the autonomous ball pickup vision system.

## Table of Contents

1. [Quick Start Tuning](#quick-start-tuning)
2. [HSV Color Detection Tuning](#hsv-color-detection-tuning)
3. [Husky Lens Configuration](#husky-lens-configuration)
4. [Camera Calibration](#camera-calibration)
5. [Performance Optimization](#performance-optimization)
6. [Testing and Validation](#testing-and-validation)
7. [Troubleshooting](#troubleshooting)
8. [Advanced Tuning](#advanced-tuning)

---

## Quick Start Tuning

### 5-Minute Basic Setup

**Step 1: Load CameraDebugTeleOp**
- Deploy CameraDebugTeleOp to RC phone
- Initialize OpMode (press INIT)
- Check telemetry for AprilTag detection (should show pose)

**Step 2: Test Ball Detection**
- Place ball in camera view
- Watch telemetry for ball detection count
- If 0 balls detected, HSV range needs tuning

**Step 3: Quick HSV Tuning**
```java
// Default yellow (might need adjustment)
H: 15-40 (hue)
S: 100-255 (saturation)
V: 100-255 (value)
```

Use gamepad controls in CameraDebugTeleOp:
- **X**: Cycle color presets (Yellow → Orange → Red)
- **DPAD UP/DOWN**: Adjust brightness (V range)
- **DPAD LEFT/RIGHT**: Adjust min area threshold

**Step 4: Record Values**
Write down HSV range when ball detection works:
```java
camera.setBallColorRange(H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX);
```

### 15-Minute Standard Setup

1. Follow Quick Start steps above
2. Fine-tune HSV range (next section)
3. Adjust minimum ball area threshold
4. Test with multiple ball positions
5. Record final values

---

## HSV Color Detection Tuning

### Understanding HSV Color Space

**Hue (H)**: 0-180° (color type)
- Red: 0-10, 170-180
- Orange: 5-25
- Yellow: 15-40
- Green: 35-85
- Cyan: 85-95
- Blue: 95-130
- Magenta: 130-170

**Saturation (S)**: 0-255 (color intensity)
- Pure color: 255
- Whitish color: 0-100
- Pale color: 100-150
- Vivid color: 200-255

**Value (V)**: 0-255 (brightness)
- Dark: 0-50
- Medium: 100-150
- Bright: 200-255

### HSV Tuning Workflow

#### Phase 1: Find Hue Range (5 minutes)

1. **Open CameraDebugTeleOp**
2. **Place ball in good lighting**
3. **Adjust H_MIN and H_MAX** until ball highlights in blue

Test HSV ranges:

```java
// Yellow balls (most common)
H: 15-40    S: 100-255    V: 100-255

// Orange balls
H: 5-25     S: 130-255    V: 100-255

// Red balls
H: 0-15     S: 100-255    V: 100-255

// Purple balls
H: 130-160  S: 80-255     V: 100-255
```

**Tip**: If ball is on border of hue range, widen by ±5 degrees

#### Phase 2: Fine-tune Saturation (5 minutes)

1. **Keep Hue locked** (from Phase 1)
2. **Slowly decrease S_MIN** until noise appears
3. **Increase S_MIN** slightly above noise threshold
4. **Keep S_MAX at 255**

**Example progression**:
```
Start: S: 100-255
Too much noise at 80? → S: 100-255
Try S: 120-255
Still too noisy? → S: 150-255
Perfect at: S: 130-255
```

#### Phase 3: Adjust Brightness Range (5 minutes)

1. **Keep H and S locked**
2. **Test in actual lighting conditions**
3. **Increase V_MIN** for bright conditions
4. **Decrease V_MIN** for low light

**Common V ranges**:
```
Bright field lights:    V: 150-255
Normal lighting:        V: 100-255
Low/variable lighting:  V: 80-255
```

**Test at different distances**:
- Close (2 feet): Ball might be overexposed (increase V_MIN)
- Far (15 feet): Ball might be dim (decrease V_MIN)

#### Phase 4: Optimize Area Threshold (3 minutes)

1. **Use final HSV range** from Phase 3
2. **Decrease setMinBallArea()** to detect smaller balls
3. **Stop when noise increases significantly**
4. **Record final area value**

```java
// Balance between sensitivity and noise:
setMinBallArea(150);  // Ignore small noise
setMinBallArea(100);  // More sensitive
setMinBallArea(200);  // Less sensitive, ignores far balls
```

### HSV Tuning Tips

✅ **DO**:
- Tune in actual field lighting
- Test at multiple distances (5ft, 10ft, 15ft)
- Test with multiple balls simultaneously
- Start with wide range, then narrow
- Record working values in code comments

❌ **DON'T**:
- Try to optimize for ALL lighting conditions
- Make H range wider than 30 degrees (too much noise)
- Set S_MIN below 80 (detects similar colors)
- Ignore shadows/reflections (they're part of reality)

### Preset HSV Values

**Yellow Balls** (FTC PowerPlay standard):
```java
camera.setBallColorRange(15, 40, 100, 255, 100, 255);
```

**Orange Balls**:
```java
camera.setBallColorRange(5, 25, 130, 255, 100, 255);
```

**Red Balls**:
```java
camera.setBallColorRange(0, 15, 100, 255, 100, 255);
camera.setBallColorRange(160, 180, 100, 255, 100, 255);  // Second red range
```

**Bright Yellow** (in harsh lighting):
```java
camera.setBallColorRange(15, 40, 100, 255, 150, 255);
```

**Dim Yellow** (in low light):
```java
camera.setBallColorRange(15, 40, 80, 255, 50, 200);
```

---

## Husky Lens Configuration

### Pre-Deployment Algorithm Setup

**Required: Complete BEFORE deploying to robot**

#### Step 1: Connect Husky Lens to Computer
- Use USB cable (not I2C)
- Install "HUSKYLENS for Arduino" IDE extension
- Or use mobile app (DFRobot HuskyLens app)

#### Step 2: Select Algorithm
1. Power on Husky Lens
2. Navigate to "Algorithm"
3. Choose detection type:
   - **Color Recognition**: Best for solid color balls
   - **Object Tracking**: Best for any shaped ball
   - **Line Tracking**: Not recommended for balls

#### Step 3: Train Algorithm

**For Color Recognition**:
1. Select "Train" mode
2. Show Husky Lens a ball
3. Let it learn for 1-2 seconds
4. Press confirm
5. Test with different lighting

**For Object Tracking**:
1. Select "Train" mode
2. Show Husky Lens a clear image of the ball
3. Let it learn shape/color features
4. Press confirm
5. Test recognition from different angles/distances

#### Step 4: Test Detection
1. Move balls around
2. Verify detection works at various distances
3. Test with multiple balls
4. Adjust sensitivity if needed

#### Step 5: Configure Parameters (if available)

```
Confidence Threshold: 50-70  (higher = more strict)
Block Size Min: 10-30        (ignore very small detections)
Block Size Max: 200-300      (ignore very large detections)
```

### Husky Lens Field Deployment

#### At Competition

```java
// In on_init()
camera.useHuskyLensDetection();

// Configure camera offset
camera.useCameraRightMounted(6.0);  // or your mounting

// Optional: verify connection
if (camera.hasHuskyLensConnection()) {
    telemetry.addLine("Husky Lens connected ✓");
}
```

#### Algorithm Not Detecting?

1. **Verify configuration saved** on Husky Lens device
2. **Check ball color matches** training
3. **Check lighting** (should be similar to training)
4. **Verify I2C connection** (see Troubleshooting)
5. **Re-train algorithm** with current conditions

### Husky Lens vs HSV Comparison

| Factor | Husky Lens | HSV |
|--------|-----------|-----|
| **Setup Time** | 10 min (training) | 5 min (tuning) |
| **CPU Usage** | 2% | 15% |
| **Accuracy** | ±1-2 inches | ±2-4 inches |
| **Speed** | <10ms | 10-15ms |
| **Lighting Sensitivity** | High (needs training) | Medium (needs tuning) |
| **Flexibility** | Low (pre-trained only) | High (tunable) |
| **Offset Support** | Full (automatic) | Limited |
| **Cost** | Higher | Lower |

**Choose HSV if**: You need maximum flexibility or only have a webcam
**Choose Husky Lens if**: You want best performance and have the hardware

---

## Camera Calibration

### Why Calibrate?

Better camera calibration = More accurate ball detection = Better autonomous performance

**Impact**:
- Default calibration: ±3-4 inches error
- Calibrated: ±1-2 inches error

### Quick Calibration (15 minutes)

**No special equipment needed**

1. **Mount camera** at fixed height (e.g., 12 inches)
2. **Measure to known reference** (e.g., field tile)
3. **Take photo** with camera
4. **Calculate** focal length from photo
5. **Update** DualCamera.java constants

```java
// Test procedure
private static final double CAMERA_HEIGHT_INCHES = 12.0;  // Your height
private static final double TEST_DISTANCE_INCHES = 60.0;  // Known distance

// Measure pixel coordinates of known object
// Use formula: focal_length = (pixel_distance * test_distance) / real_distance
```

### Precise Calibration (30 minutes)

**For competition-ready accuracy**

1. **Print calibration target** (checkerboard pattern)
2. **Mount at known distance** and height
3. **Take multiple photos** at different angles
4. **Use OpenCV calibration tools** or online calculator
5. **Record focal length and principal point**

```java
// Update in DualCamera.java
private static final double FOCAL_LENGTH_X = 578.272;   // Your value
private static final double FOCAL_LENGTH_Y = 578.272;   // Your value
private static final double PRINCIPAL_POINT_X = 402.145; // Your value
private static final double PRINCIPAL_POINT_Y = 221.506; // Your value
```

### Measure Camera Height

```
┌─────────────────┐
│  Robot frame    │
│  ┌───────────┐  │
│  │ Camera ◄──┼──┼─→ CAMERA_HEIGHT
│  └───────────┘  │
└─────────────────┘
```

**How to measure**:
1. Place robot on level surface
2. Measure from ground to camera lens center
3. Add 1-2 inches if camera points downward at angle
4. Test with known distance: if readings are off, adjust

---

## Performance Optimization

### Reducing Latency

**Current Performance**:
- HSV Detection: 10-15ms
- Husky Lens Detection: <10ms
- AprilTag Detection: 15-20ms
- Total (concurrent): 30-40ms

**To improve further**:

#### For HSV Mode
```java
// Reduce frame rate (trades responsiveness for speed)
visionPortal.setProcessorEnabled(ballDetector, true);
// Process every other frame
if (frameCount % 2 == 0) {
    // Process frame
}

// Lower resolution (faster but less accurate)
visionPortal.setResolution(320, 240);  // Instead of 640x480
```

#### For Husky Lens Mode
```java
// I2C bus speed optimization
// In hardware config, set I2C bus to 400 kHz (already default)

// Reduce detection block size if possible
// (Configure on device)
```

### Improving Accuracy

**Current accuracy**: ±2-4 inches

**To improve**:

1. **Better camera calibration** (-0.5 inches)
2. **Optimize HSV range** (-0.5 inches)
3. **Larger detection area** (-0.5 inches)
4. **Stable AprilTag** (-0.5 inches)

**Final potential**: ±0.5-1 inch

### Reducing CPU Usage

**HSV Mode** uses 15-20% CPU

**To reduce**:
```java
// Increase min area threshold
camera.setMinBallArea(200);  // Was 150

// Reduce resolution
camera.setResolution(320, 240);  // Was 640x480

// Reduce frame rate
camera.setFrameRate(20);  // Was 30

// Use Husky Lens instead (uses 2-3% CPU)
```

---

## Testing and Validation

### Vision-Only Test (10 minutes)

Tests just the camera system without autonomous:

```java
@TeleOp(name = "Vision Test", group = "Debug")
public class VisionTest extends OpMode {
    private DualCamera camera;
    
    @Override
    public void init() {
        camera = new DualCamera(robot);
        camera.registerHardware(hardwareMap);
        camera.useHuskyLensDetection();
        camera.startup();
    }
    
    @Override
    public void loop() {
        camera.update(this);
        
        // Display detected balls
        telemetry.addData("Balls detected", camera.getBallCount());
        
        if (camera.hasBallDetections()) {
            for (BallTarget ball : camera.getDetectedBalls()) {
                telemetry.addData("Ball X", ball.fieldX);
                telemetry.addData("Ball Y", ball.fieldY);
                telemetry.addData("Distance", ball.getDistanceTo(camera.getPose()));
            }
        }
        
        // Display pose
        telemetry.addData("Robot X", camera.getPose().getX());
        telemetry.addData("Robot Y", camera.getPose().getY());
    }
}
```

### Single Ball Test (15 minutes)

Tests autonomous pickup of one ball:

1. Place robot at known position
2. Place one ball at known distance
3. Run autonomous
4. Measure actual vs expected positions
5. Note any errors

**Success criteria**:
- Robot detects AprilTag ✓
- Robot detects ball ✓
- Robot navigates to ball ✓
- Robot position error <6 inches ✓
- Ball position error <4 inches ✓

### Multi-Ball Test (20 minutes)

Tests pickup of 3 balls in sequence:

1. Place 3 balls in known positions
2. Run AdvancedBallPickupAuto
3. Record actual path vs expected
4. Measure efficiency and accuracy

**Success criteria**:
- All 3 balls detected ✓
- Optimal path chosen ✓
- No collisions with obstacles ✓
- All balls collected ✓
- Total time <30 seconds ✓

### Accuracy Validation

**Test with known distances**:

```
Position 1: Robot at (0,0), Ball at (12,0)
Position 2: Robot at (0,0), Ball at (0,12)
Position 3: Robot at (0,0), Ball at (12,12) [diagonal]
Position 4: Robot at (12,0), Ball at (0,12) [far]
```

**Measure error**:
```
Actual distance vs Detected distance
Error = |Actual - Detected| / Actual × 100%

Target: <5% error (±0.6 inches at 12 inches)
Good: 5-10% error (±0.6-1.2 inches)
Poor: >15% error (>1.8 inches)
```

---

## Troubleshooting

### HSV Detection Not Working

**Symptom**: `camera.hasBallDetections()` returns false

**Diagnosis**:
1. Check AprilTag detection works (test with CameraDebugTeleOp)
2. Verify HSV range is reasonable
3. Test HSV range in CameraDebugTeleOp

**Solutions**:
```java
// Step 1: Verify detection is enabled
camera.useHSVDetection();
camera.setBallColorRange(15, 40, 100, 255, 100, 255);

// Step 2: Lower minimum area threshold
camera.setMinBallArea(50);  // Very sensitive, shows noise

// Step 3: Expand HSV range temporarily
camera.setBallColorRange(10, 45, 80, 255, 80, 255);  // Wider range

// Step 4: Test in bright/dark conditions
```

### Husky Lens Not Detecting Balls

**Symptom**: Husky Lens shows connected but no detections

**Diagnosis**:
1. Verify Husky Lens I2C connection
2. Verify algorithm is trained and saved
3. Test with Husky Lens mobile app

**Solutions**:
```
Step 1: Check I2C communication
  → Test with I2C scanner
  → Verify SDA/SCL tight
  → Check 5V power

Step 2: Check algorithm
  → Power on Husky Lens
  → Select algorithm (should show green checkmark)
  → Test with object in view

Step 3: Re-train algorithm
  → Training mode
  → Show Husky Lens ball
  → Take 2-3 samples at different distances
  → Save and test
```

### Low Frame Rate

**Symptom**: Telemetry shows <15 Hz update rate

**Diagnosis**:
1. HSV detection is CPU-intensive
2. Vision Portal might be overloaded
3. I2C communication slow (Husky Lens)

**Solutions**:
```java
// For HSV mode:
camera.setResolution(320, 240);  // Lower resolution
camera.setMinBallArea(200);      // Filter more noise
visionPortal.setFrameRate(20);   // Reduce frame rate

// For Husky Lens:
// Check I2C bus speed (should be 400 kHz)
// Reduce polling frequency if needed
```

### Balls Detected Far Away (>15 feet)

**Symptom**: Ball detected but very inaccurate

**Diagnosis**:
1. Lens distortion at distance
2. Insufficient resolution
3. HSV range too permissive

**Solutions**:
```java
// Filter by confidence
var highConfidenceBalls = camera.getDetectedBalls()
    .stream()
    .filter(b -> b.confidence > 0.7)
    .collect(Collectors.toList());

// Filter by distance
var nearBalls = camera.getDetectedBalls()
    .stream()
    .filter(b -> b.getDistanceTo(robot.getRobotPose()) < 24)
    .collect(Collectors.toList());

// Increase minimum area (ignores small distant balls)
camera.setMinBallArea(200);
```

### Camera Offset Compensation Not Working

**Symptom**: Ball position incorrect despite offset configuration

**Diagnosis**:
1. Offset values wrong
2. Camera calibration off
3. AprilTag pose inaccurate

**Solutions**:
```java
// Verify offset is configured
System.out.println(camera.getCameraOffset());

// Test with known position
camera.setCameraPosition(0, 6.0, 12.0);  // Right 6 inches
// Place ball directly in front
// Should detect at (0, 6) offset in field

// Adjust if needed
camera.setCameraPosition(0, 6.5, 12.0);  // Try 6.5 inches
```

---

## Advanced Tuning

### Multi-Lighting Conditions

For fields with varying lighting:

```java
// Create lighting presets
class LightingProfile {
    public int hMin, hMax, sMin, sMax, vMin, vMax;
    public int minArea;
    public String name;
    
    public static final LightingProfile[] PROFILES = {
        new Profile("bright",   15, 40, 100, 255, 150, 255, 150),
        new Profile("normal",   15, 40, 100, 255, 100, 255, 150),
        new Profile("dim",      15, 40, 80, 255, 50, 200, 100),
    };
    
    public void apply(DualCamera camera) {
        camera.setBallColorRange(hMin, hMax, sMin, sMax, vMin, vMax);
        camera.setMinBallArea(minArea);
    }
}
```

### Adaptive Tuning

Automatically adjust based on conditions:

```java
// Monitor detection success rate
if (lastFrameHadBalls && !currentFrameHasBalls) {
    // Lost detection, lighting changed?
    // Increase sensitivity
    camera.setMinBallArea(Math.max(50, currentArea - 20));
}

// Monitor AprilTag detection consistency
if (!camera.hasAprilTagLock() && previousHadLock) {
    // Lost AprilTag, but maybe can use last pose
    // Increase AprilTag detection aggressiveness if possible
}
```

### Dual Detection Fallback

Use both HSV and Husky Lens:

```java
BallTarget findBall() {
    // Try Husky Lens first (faster, more accurate)
    if (camera.useHuskyLensDetection()) {
        List<BallTarget> balls = camera.getDetectedBalls();
        if (!balls.isEmpty()) {
            return balls.get(0);
        }
    }
    
    // Fallback to HSV if Husky Lens fails
    camera.useHSVDetection();
    List<BallTarget> balls = camera.getDetectedBalls();
    if (!balls.isEmpty()) {
        return balls.get(0);
    }
    
    return null;  // No ball found
}
```

---

## Summary

### Tuning Checklist

- [ ] Load CameraDebugTeleOp
- [ ] Verify AprilTag detection works
- [ ] Test ball detection in actual lighting
- [ ] Tune HSV range (if using HSV mode)
- [ ] Calibrate camera (optional, improves accuracy)
- [ ] Measure min ball area threshold
- [ ] Test single ball pickup
- [ ] Test multi-ball sequence
- [ ] Validate accuracy at multiple distances
- [ ] Record final tuning parameters
- [ ] Document for future use

### Final Values to Document

```java
// Camera detection settings
camera.setBallColorRange(?, ?, ?, ?, ?, ?);
camera.setMinBallArea(?);

// Camera calibration (if tuned)
FOCAL_LENGTH_X = ?
FOCAL_LENGTH_Y = ?
PRINCIPAL_POINT_X = ?
PRINCIPAL_POINT_Y = ?
CAMERA_HEIGHT_INCHES = ?

// Camera offset (if using Husky Lens)
CAMERA_OFFSET_X = ?
CAMERA_OFFSET_Y = ?
CAMERA_OFFSET_Z = ?

// Performance metrics
Update frequency: ? Hz
Detection latency: ? ms
Positional accuracy: ±? inches
```

Good luck tuning! 🎯
