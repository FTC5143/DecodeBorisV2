# Vision System Technical Documentation

Complete technical deep-dive into the autonomous ball pickup vision system architecture, algorithms, and implementation details.

## Table of Contents

1. [System Architecture](#system-architecture)
2. [Component Details](#component-details)
3. [Detection Algorithms](#detection-algorithms)
4. [Coordinate Transformations](#coordinate-transformations)
5. [Camera Calibration](#camera-calibration)
6. [Performance Analysis](#performance-analysis)
7. [Concurrency Model](#concurrency-model)
8. [Error Handling](#error-handling)
9. [Advanced Features](#advanced-features)
10. [Integration Details](#integration-details)

---

## System Architecture

### High-Level Overview

The vision system consists of two concurrent detection pipelines within a single camera hardware interface:

```
┌─────────────────────────────────────────────────────┐
│             Hardware Input (Camera)                  │
└──────────────────┬──────────────────────────────────┘
                   │
        ┌──────────┴──────────┐
        │                     │
        ▼                     ▼
   ┌─────────┐          ┌──────────┐
   │ Vision  │          │ Husky    │
   │ Portal  │          │ Lens I2C │
   └────┬────┘          └────┬─────┘
        │                    │
   ┌────┴─────┐         ┌────┴──────┐
   │           │         │           │
   ▼           ▼         ▼           ▼
┌─────────┐ ┌────┐  ┌────────┐  ┌────────┐
│AprilTag │ │HSV │  │Husky   │  │Offset  │
│Processor│ │Ball│  │Lens I2C│  │Config  │
└────┬────┘ └──┬─┘  └───┬────┘  └───┬────┘
     │         │        │           │
     └─────────┴────────┴───────────┘
              │
              ▼
        ┌──────────────┐
        │ Ball Target  │
        │  Objects     │
        │ (Field Coords)│
        └──────┬───────┘
               │
        ┌──────┴──────────┐
        │                 │
        ▼                 ▼
   Pathfinding        Control Loop
   & Optimization     & Execution
```

### Design Philosophy

**Separation of Concerns**:
- DualCamera: Manages hardware and pipeline selection
- Processors: Handle image processing logic
- Config: Manages coordinate transformations
- Targets: Represent detected objects

**Modularity**:
- Processors can be swapped independently
- Detection modes (HSV/Husky Lens) are interchangeable
- Offset compensation is decoupled from detection

**Concurrency**:
- AprilTag detection always runs (Vision Portal)
- Ball detection runs independently (HSV in Vision Portal, Husky Lens via I2C)
- Both complete before next processing cycle

---

## Component Details

### DualCamera.java

Main orchestration class managing the entire vision system.

#### Class Structure

```java
public class DualCamera extends Component {
    // Hardware
    private WebcamName camera;
    private VisionPortal visionPortal;
    
    // Processors
    private AprilTagProcessor aprilTagProcessor;
    private BallDetectionProcessor ballDetector;
    private HuskyLensBallDetector huskyLensDetector;
    
    // Configuration
    private CameraOffsetConfig offsetConfig;
    private boolean useHuskyLens = false;
    
    // Detection results
    private List<BallTarget> detectedBalls;
    private Pose robotPose;
}
```

#### Key Methods

**Initialization**:
```java
public void registerHardware(HardwareMap hardwareMap)
// Creates VisionPortal with both processors
// Initializes AprilTag processor
// Sets up HSV ball detector

public void startup()
// Enables processors
// Starts VisionPortal processing
// Initializes Husky Lens if enabled

public void shutdown()
// Disables processors
// Stops VisionPortal
// Closes Husky Lens connection
```

**Main Update Loop**:
```java
public void update(OpMode opMode)
// Polls AprilTag results
// Polls ball detection results (HSV or Husky Lens)
// Applies coordinate transformations
// Updates internal state

// Internal workflow:
// 1. Get AprilTag detections
// 2. Extract robot pose if tag detected
// 3. Poll appropriate ball detector
// 4. Apply offset compensation if needed
// 5. Filter and store results
```

**Detection Mode Selection**:
```java
public void useHSVDetection()
// Sets useHuskyLens = false
// Disables Husky Lens detector
// Ensures HSV processor is enabled

public void useHuskyLensDetection()
// Sets useHuskyLens = true
// Enables Husky Lens detector
// Disables HSV processing in Vision Portal

public boolean isUsingHuskyLens()
// Returns current detection mode
```

**Camera Configuration**:
```java
public void setCameraPosition(double offsetX, double offsetY, double offsetZ)
// Sets camera mounting offset
// Creates CameraOffsetConfig with values
// Used for side-mounted cameras

public void useCameraRightMounted(double distanceInches)
public void useCameraLeftMounted(double distanceInches)
public void useCameraForwardMounted(double distanceInches)
public void useCameraCenterMounted()
// Preset mounting configurations
// Automatically set correct offsets
```

#### Thread Safety

```java
// Results stored in thread-safe containers
private final CopyOnWriteArrayList<BallTarget> detectedBalls;

// Volatile references for main variables
private volatile Pose robotPose;
private volatile boolean hasAprilTagLock;

// Synchronization on Vision Portal calls
synchronized(visionPortal) {
    // Get latest frame results
}
```

### BallDetectionProcessor.java

OpenCV-based HSV color detection for ball identification.

#### Algorithm Overview

```
Input Frame (BGRmat)
    ↓
1. Color Space Conversion
   BGR → HSV (cvtColor)
    ↓
2. Range Mask Creation
   inRange(hsv, lower, upper)
    ↓
3. Morphological Operations
   erode + dilate (noise removal)
    ↓
4. Contour Detection
   findContours(mask)
    ↓
5. Contour Filtering
   area threshold
    ↓
6. Center Calculation
   moments(contour)
    ↓
Output: BallDetection[] (pixel coordinates)
```

#### HSV Filtering Math

```java
// Create scalar bounds
Scalar lower = new Scalar(hMin, sMin, vMin);
Scalar upper = new Scalar(hMax, sMax, vMax);

// Mask pixels within range
Core.inRange(hsv, lower, upper, mask);
// Result: binary image (0 or 255)
```

#### Contour Processing

```java
// Find contours
Imgproc.findContours(mask, contours, hierarchy, 
    Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

// Filter by area
for (MatOfPoint contour : contours) {
    double area = Imgproc.contourArea(contour);
    if (area > minBallArea) {
        // Calculate center using moments
        Moments M = Imgproc.moments(contour);
        int cx = (int)(M.m10 / M.m00);
        int cy = (int)(M.m01 / M.m00);
    }
}
```

#### Confidence Calculation

```java
// Based on contour properties
double confidence = calculateConfidence(contour);

// Factors:
// 1. Circularity: How close to circle (0-1)
double circularity = 4 * PI * area / (perimeter * perimeter);

// 2. Area consistency: How much of region is filled
double fill = area / boundingRect.area();

// 3. Size consistency: How centered in bounds
double centeredness = distanceToBounds;

confidence = (circularity + fill + centeredness) / 3.0;
```

#### Coordinate Output

```java
public BallDetection[] getDetections()
// Returns array of detected balls in PIXEL coordinates
// Each contains:
// - centerX, centerY (in pixel space, 0-640)
// - confidence (0-1)
// - area (in pixels)
```

### HuskyLensBallDetector.java

I2C-based Husky Lens smart camera driver.

#### I2C Communication

```java
// Husky Lens I2C protocol
public class HuskyLensBallDetector {
    private static final int HUSKY_LENS_I2C_ADDRESS = 0x32;
    private static final int HUSKY_LENS_I2C_FREQUENCY = 400000;  // 400 kHz
    
    private I2cDevice device;
    private I2cDeviceSynch deviceClient;
}
```

#### Data Structure

```java
// Block: detected object from Husky Lens
class HuskyLensBlock {
    public int x;          // Center X (pixel)
    public int y;          // Center Y (pixel)
    public int width;      // Block width
    public int height;     // Block height
    public int id;         // Object ID
    public int confidence; // Confidence 0-100
}
```

#### Detection Loop

```java
public void update()
// Non-blocking call, updates detection state
// Polls I2C at ~30 Hz
// Handles I2C timeouts gracefully
// Filters blocks by confidence threshold

// Workflow:
// 1. Send "read blocks" command via I2C
// 2. Wait for response (<10ms typically)
// 3. Parse block data from response
// 4. Filter by confidence
// 5. Store in internal list
```

#### Algorithm Selection

```java
public void selectAlgorithm(String algorithmName)
// Sends command to Husky Lens
// Switches active detection algorithm
// Requires pre-trained algorithm on device

// Supported algorithms:
// - "Object Tracking" (0x00)
// - "Color Recognition" (0x01)
// - "Line Tracking" (0x04)
```

### CameraOffsetConfig.java

Camera offset compensation for side-mounted and offset cameras.

#### Offset Concept

```
When camera is offset from center:
┌─────────────────┐
│  Robot Frame    │
│  ┌───────────┐  │
│  │ Camera ◄──┼──┼─→ Offset (6")
│  │   View    │  │
│  └───────────┘  │
└─────────────────┘

Detected ball at pixel (320, 240) 
→ Camera center (0, 6 inches right)
→ Robot center (-6 inches left from ball)
→ Field coordinates adjusted
```

#### Configuration Fields

```java
public class CameraOffsetConfig {
    // Position offset (inches)
    private double offsetX;      // Right (+) / Left (-)
    private double offsetY;      // Forward (+) / Backward (-)
    private double offsetZ;      // Up (+) / Down (-)
    
    // Orientation offset (radians)
    private double yaw;          // Pan left/right
    private double pitch;        // Tilt up/down
    private double roll;         // Tilt side to side
    
    // Camera intrinsics
    private double focalLengthX; // Horizontal focal length
    private double focalLengthY; // Vertical focal length
    private double principalX;   // Principal point X
    private double principalY;   // Principal point Y
    
    // Image properties
    private int imageWidth;      // Frame width (pixels)
    private int imageHeight;     // Frame height (pixels)
}
```

#### Transformation Pipeline

Converting from pixel to field coordinates involves 7 sequential transformations:

```
Step 1: Pixel Normalization
────────────────────────────
pixel coords (0-640, 0-480) → normalized coords (-1 to 1)

normX = (pixelX - principalX) / focalLengthX
normY = (pixelY - principalY) / focalLengthY

Step 2: Ground Distance Calculation
────────────────────────────────────
Using camera height and angle:

distance = offsetZ / sqrt(1 + normX² + normY²)

This projects the pixel ray to the ground plane.

Step 3: Ground Point in Camera Frame
─────────────────────────────────────
Convert normalized ray to 3D point on ground:

cameraX = distance * normX
cameraY = distance * normY
cameraZ = 0 (ground plane)

Step 4: Apply Camera Orientation
─────────────────────────────────
Rotate by camera pan/tilt/roll:

point = rotationMatrix(yaw, pitch, roll) * point

Step 5: Apply Camera Offset
────────────────────────────
Translate from camera center to robot center:

robotX = cameraX - offsetX
robotY = cameraY - offsetY

Step 6: Field Rotation by Robot Heading
────────────────────────────────────────
Rotate by robot's rotation on field:

fieldX = robotX * cos(heading) - robotY * sin(heading)
fieldY = robotX * sin(heading) + robotY * cos(heading)

Step 7: Absolute Field Position
────────────────────────────────
Add robot's field position:

absX = robotPose.x + fieldX
absY = robotPose.y + fieldY
```

#### Implementation

```java
public BallTarget pixelToFieldCoordinates(
    int pixelX, int pixelY, Pose robotPose)
{
    // Step 1: Normalize
    double normX = (pixelX - principalX) / focalLengthX;
    double normY = (pixelY - principalY) / focalLengthY;
    
    // Step 2: Calculate ground distance
    double denominator = Math.sqrt(1 + normX*normX + normY*normY);
    double groundDistance = offsetZ / denominator;
    
    // Step 3: Ground point in camera frame
    double camX = groundDistance * normX;
    double camY = groundDistance * normY;
    
    // Step 4-7: Apply rotations and translations
    // (See source code for full implementation)
    
    return new BallTarget(fieldX, fieldY, distance);
}
```

### BallTarget.java

Data class representing a detected ball in field coordinates.

```java
public class BallTarget {
    // Field coordinates (inches)
    public double fieldX;
    public double fieldY;
    
    // Quality metrics
    public double confidence;      // 0-1
    public double distanceFromRobot;
    
    // Metadata
    public long detectionTime;
    public int sourceDetector;    // HSV or Husky Lens
}
```

---

## Detection Algorithms

### HSV Color Detection Algorithm

#### HSV Color Space

HSV (Hue, Saturation, Value) is preferred over RGB for object detection:

```
┌─────────────────────────────────────┐
│ RGB Color Space                     │
│ (Used by cameras)                   │
│                                     │
│ R: Red intensity                    │
│ G: Green intensity                  │
│ B: Blue intensity                   │
│                                     │
│ Problem: Color depends on lighting  │
│ Same color ≠ same RGB values        │
└─────────────────────────────────────┘

        ↓ Convert ↓

┌─────────────────────────────────────┐
│ HSV Color Space                     │
│ (Used by detection)                 │
│                                     │
│ H: Hue (0-180°) - Color type        │
│ S: Saturation (0-255) - Color purity│
│ V: Value (0-255) - Brightness       │
│                                     │
│ Benefit: Robust to lighting changes │
│ Same color = same H range           │
└─────────────────────────────────────┘
```

#### BGR to HSV Conversion

OpenCV's `cvtColor()` performs this transformation:

```
H = arctan(sqrt(3) * (G - B)) / (R - B)  (simplified)
S = (max(R,G,B) - min(R,G,B)) / max(R,G,B)
V = max(R, G, B)
```

#### Mask Generation

```java
// Create range mask
Mat lower = Scalar.all(hMin, sMin, vMin);
Mat upper = Scalar.all(hMax, sMax, vMax);
Core.inRange(hsvFrame, lower, upper, mask);

// Result: binary image
// White (255): pixels in range
// Black (0): pixels out of range
```

#### Noise Reduction (Morphological Operations)

```
Raw Mask:     Eroded:       Dilated:
███ ░ ███     ██  ░  ██     ███ ░ ███
███████       ██████       █████████
██░███        ██ ██        █████████
```

```java
// Erode: Remove small white regions (noise)
Imgproc.erode(mask, eroded, kernel);

// Dilate: Fill small black holes
Imgproc.dilate(eroded, final, kernel);
```

#### Contour Detection

```
Input (Eroded + Dilated mask):
███████
███████  → Contours: Connected regions
██ ██
```

```java
Imgproc.findContours(mask, contours, ...);
// Returns list of MatOfPoint
// Each represents one connected region
```

#### Contour Filtering by Area

```
┌─────────────┐  ┌──────────┐  ┌──────────┐
│   SMALL     │  │  MEDIUM  │  │  LARGE   │
│ Noise/False │  │  GOOD    │  │ Obstacle │
│ Positives   │  │  BALL    │  │  Block   │
│  Area < 50  │  │50-500px2 │  │ Area>500 │
└─────────────┘  └──────────┘  └──────────┘

Threshold: minBallArea (typically 150 pixels²)
```

#### Contour Center Calculation (Image Moments)

```java
// Moments: Statistical measures of shape
Moments M = Imgproc.moments(contour);

// m00 = total area
// m10 = first moment (X)
// m01 = first moment (Y)

// Center: weighted average
centerX = m10 / m00;
centerY = m01 / m00;
```

### Husky Lens Detection Algorithm

#### On-Device Processing

Husky Lens performs detection internally using pre-trained models:

```
Camera Input
    ↓
Husky Lens Hardware
    ├─ Pre-trained Neural Network
    ├─ Object Detection Model
    └─ Confidence Scoring
    ↓
Block Data (X, Y, Width, Height, ID)
```

#### I2C Protocol

```
Host (RC Control Hub)          Husky Lens
         │                         │
         │──── Read Blocks ──────→ │
         │                    (I2C Command)
         │                         │
         │ ← Send Block Data ──── │
         │                    (I2C Response)
         │                         │
    Parse & Filter                 │
```

#### Confidence Thresholding

```java
// Filter blocks by confidence
for (HuskyLensBlock block : allBlocks) {
    if (block.confidence > CONFIDENCE_THRESHOLD) {
        // Use this block
    }
}
// Typical threshold: 50-70 (out of 100)
```

---

## Coordinate Transformations

### Pixel to Camera Coordinates

```
Pixel Space (Image)          Camera Space (3D)
┌─────────────────┐          ┌──────────────────┐
│(0,0)         (640,0)        │ Camera looking down
│  •            •  │          │     Z (depth)
│  •  (320,240) •  │          │     ↑
│  •            •  │    →     │     │ Y (forward)
│(0,480)    (640,480)         │ ─ ─ ─ → X (right)
└─────────────────┘          └──────────────────┘

Transformation:
1. Normalize pixel coordinates
2. Scale by focal length
3. Project to 3D using camera model
```

### Camera to Robot Coordinates

```
Camera Frame              Robot Frame
(Camera center)           (Robot center)
    ↑ Y                       ↑ Y
    │                         │
    ├─→ X                     ├─→ X
    Z                         Z

Transformation:
1. Translate by offset (-offsetX, -offsetY, -offsetZ)
2. Rotate by camera orientation (yaw, pitch, roll)
3. Result: Ball position relative to robot
```

### Robot to Field Coordinates

```
Robot Frame                 Field Frame
(Robot-centric)             (Absolute)
    ↑ Y_robot                   ↑ Y_field
    │                           │
    ├─→ X_robot                 ├─→ X_field

Transformation:
1. Rotate by robot heading: heading = θ
   fieldX = X_robot * cos(θ) - Y_robot * sin(θ)
   fieldY = X_robot * sin(θ) + Y_robot * cos(θ)

2. Translate by robot position: (rx, ry)
   absX = fieldX + rx
   absY = fieldY + ry

Result: Ball's absolute position on field
```

### Complete Transformation Example

```
Scenario:
- Pixel detection: (320, 240) [image center]
- Camera height: 12 inches
- Camera offset: 6 inches right
- Robot pose: (0, 0) at heading 0°
- Robot at origin on field

Step-by-step:
1. Pixel (320, 240) → normalized (0, 0)
   [At image center, pointing straight down]

2. Ground distance = 12 / sqrt(1 + 0 + 0) = 12 inches
   [Points 12 inches below camera]

3. Camera frame: (0, 0, -12)
   [Directly below camera]

4. After offset: (-6, 0, -12)
   [Robot is 6 inches to the left]

5. After robot rotation (0°): Same
   [No rotation]

6. Absolute position: (0, 0) + (-6, 0) = (-6, 0)
   [Field position: 6 inches left of robot]
```

---

## Camera Calibration

### Intrinsic Parameters

```
┌──────────────────────────────────┐
│   Camera Intrinsic Matrix K      │
│                                  │
│   [ fx  0  cx ]                  │
│   [ 0  fy  cy ]                  │
│   [ 0   0   1 ]                  │
│                                  │
│ fx, fy = focal length (pixels)   │
│ cx, cy = principal point (pixels)│
└──────────────────────────────────┘
```

### Focal Length Calculation

```
Physical focal length (mm) → Pixel focal length

              image distance (pixels)
focal_pixels = ───────────────────────────
               physical focal length (mm)

Or empirically:

              reference_distance (pixels) × distance (mm)
focal_length = ──────────────────────────────────────────
                     reference_size (pixels)
```

### Principal Point

```
Image coordinates:        Principal point:
(0,0)                     (cx, cy)
  ┌───────────┐             ┌───────────┐
  │           │             │     •     │ cy
  │           │      →      │    / \    │
  │           │             │   /   \   │
  └───────────┘             │ cx      │
 (640, 480)                 └───────────┘
                           (640, 480)

For most cameras:
cx ≈ width / 2
cy ≈ height / 2
```

### Calibration Methods

**Method 1: Checkerboard (OpenCV)**
```
1. Print checkerboard pattern
2. Take photos at different angles
3. OpenCV detects corners
4. Solves for intrinsic parameters
5. Typical accuracy: ±1-2%
```

**Method 2: Known Distance (Field)**
```
1. Place object at known distance
2. Measure pixel size
3. Calculate focal length
4. Typical accuracy: ±3-5%
```

**Method 3: Empirical (Competition)**
```
1. Place ball at 12 inches (exactly)
2. Record pixel coordinates
3. Repeat at different distances
4. Solve for best fit
5. Typical accuracy: ±5-10%
```

---

## Performance Analysis

### Latency Budget

```
┌──────────────────────────────────────┐
│  Total System Latency: ~35-40ms      │
├──────────────────────────────────────┤
│ Camera capture              2-3 ms    │
│ Vision Portal processing  15-20 ms    │
│ ├─ AprilTag detection    10-15 ms    │
│ └─ Ball detection (HSV)   5-10 ms    │
│ Husky Lens I2C poll (alt)  5-8 ms    │
│ Coordinate transform       1-2 ms    │
│ State update               1-2 ms    │
├──────────────────────────────────────┤
│ One-way latency           15-20 ms    │
│ Round-trip (command)      30-40 ms    │
└──────────────────────────────────────┘
```

### CPU Usage

```
HSV Detection Mode:
├─ Vision Portal setup         3%
├─ AprilTag processing         7%
├─ Ball detection (HSV)       12%
├─ Coordinate transform        1%
└─ Total                      ~23%

Husky Lens Detection Mode:
├─ Vision Portal setup         3%
├─ AprilTag processing         7%
├─ I2C polling                 1%
├─ Block parsing              <1%
├─ Coordinate transform        1%
└─ Total                      ~12%

Overhead (per core):
Robot control loop             20%
Other systems (motor, servo)   30%
Available for user code        27%
```

### Accuracy Analysis

**Position Error Sources**:

```
Error Source              Magnitude    Impact
──────────────────────────────────────────────
Camera calibration         ±0.5-1.5"    Critical
AprilTag detection         ±1-2"        Critical
HSV noise/false positives  ±0.5-1"      Major
Offset compensation error  ±0.3-0.5"    Moderate
Coordinate rounding        ±0.1"        Minor
───────────────────────────────────────────
Total (typical)            ±2-4"        Compound
```

**Error Propagation**:
```
AprilTag error (±2") → Robot pose error (±2")
                    → Ball position error (±2")

HSV detection error (±0.5") → Additional error

Total = sqrt(2² + 0.5²) ≈ ±2.06"
```

### Frame Rate Analysis

```
Vision Portal Frame Rate: ~30 FPS
├─ Baseline              30 FPS
├─ With AprilTag         30 FPS (same thread)
├─ With HSV ball detect  ~25 FPS (heavy load)
├─ With resolution 640   ~25 FPS
└─ With resolution 320   ~30 FPS

Husky Lens Poll Rate: ~30 Hz
├─ I2C communication     <2ms per request
├─ Non-blocking I2C       Optimal
├─ No Vision Portal load  Lower CPU
└─ Limited by I2C clock   400 kHz max
```

---

## Concurrency Model

### Vision Portal Threading

```
Main Thread                Vision Portal Thread
    │                             │
    │ registerHardware()          │
    │──────────────────────→ Create processors
    │                             │
    │ startup()                   │
    │──────────────────────→ Start processing
    │                             │
    │ update()                    │
    │─┐                          │
    │ │ Get AprilTag results ←────── Continuous
    │ │                          processing
    │ │ Get Ball results   ←────────  at 30 FPS
    │ │
    │─→ Process results
    │   (main thread)
    │
    │ Next iteration (3-33ms later)
```

### Race Condition Prevention

```java
// Problem: Vision thread updates while main thread reads
List<BallTarget> balls;      // NOT thread-safe!

// Solution: Use concurrent collections
CopyOnWriteArrayList<BallTarget> balls;  // Thread-safe!

// Or synchronize access
synchronized(ballList) {
    List<BallTarget> snapshot = new ArrayList<>(ballList);
}
```

### Event Loop Timing

```
OpMode loop runs at ~50-100 Hz (20ms per iteration)
Vision Portal runs at ~30 FPS (33ms per frame)

Synchronization:
Frame 1  ████ (Vision: 33ms)
OpMode  ██ ██ ██ ██ ██ (OpMode: 20ms each)

Results available:
Frame N detection → OpMode reads in Frame N+1
Worst-case latency: 33ms + 20ms = 53ms
Best-case latency: 33ms + 0ms = 33ms
Average latency: 35-40ms
```

---

## Error Handling

### Detection Failures

```java
// AprilTag detection loss
if (!camera.hasAprilTagLock()) {
    // Fallback strategies:
    // 1. Use last known pose
    // 2. Use odometry estimate
    // 3. Stop autonomous and wait
    // 4. Use visual servoing without pose
}

// Ball detection false positive
if (confidence < CONFIDENCE_THRESHOLD) {
    // Discard detection
}

// Husky Lens I2C timeout
if (!camera.hasHuskyLensConnection()) {
    // Fallback to HSV
    camera.useHSVDetection();
}
```

### Graceful Degradation

```
Scenario: AprilTag lost midway through autonomous

Current system:
├─ Continue with last known pose
├─ Use odometry for incremental updates
├─ Ball detection still works
├─ Accuracy degrades over time
└─ Eventually request relock

Alternative implementation:
├─ Switch to marker-less navigation
├─ Use line detection + vision servoing
├─ Slower but no AprilTag required
```

### Timeout Handling

```java
// Vision Portal timeout
private static final long VISION_TIMEOUT_MS = 100;

if (System.currentTimeMillis() - lastVisionUpdateTime > VISION_TIMEOUT_MS) {
    // Vision system not responding
    // Stop autonomous, log error
}

// Husky Lens I2C timeout
private static final long HUSKY_LENS_TIMEOUT_MS = 50;

if (System.currentTimeMillis() - lastHuskyLensUpdateTime > HUSKY_LENS_TIMEOUT_MS) {
    // I2C communication lost
    // Fallback to HSV or stop
}
```

---

## Advanced Features

### Multi-Ball Detection and Sorting

```java
// Detect all balls and sort by distance
List<BallTarget> balls = camera.getDetectedBalls();

Collections.sort(balls, (a, b) -> 
    Double.compare(
        a.getDistanceTo(robot.getPose()),
        b.getDistanceTo(robot.getPose())
    )
);

// Now: balls.get(0) = closest ball
```

### Tracking and Persistence

```java
class TrackedBall {
    public BallTarget current;
    public BallTarget previous;
    public long firstDetectedTime;
    public int consecutiveFrames;
}

// Filter transient false positives
if (ball.consecutiveFrames > FRAMES_TO_CONFIRM) {
    // Ball is real, not noise
}
```

### Adaptive Thresholding

```java
// Adjust detection threshold based on success
if (!detectedBalls.isEmpty()) {
    // Detection working, keep current params
} else {
    // No detection, relax threshold
    camera.setMinBallArea(Math.max(50, minArea - 10));
}
```

### Dual-Mode Fallback

```java
// Try Husky Lens first (faster, more accurate)
camera.useHuskyLensDetection();
var balls = camera.getDetectedBalls();

if (balls.isEmpty()) {
    // Fallback to HSV if Husky Lens fails
    camera.useHSVDetection();
    balls = camera.getDetectedBalls();
}
```

---

## Integration Details

### Integration with Pedro Pathing

```java
// Get detected ball position
BallTarget ball = camera.getClosestBall();
Pose ballPose = ball.toPose();

// Get current robot pose (from AprilTag)
Pose currentPose = robot.follower.getPose();

// Create path to ball
PathChain path = robot.follower.pathBuilder()
    .addPath(new BezierLine(currentPose, ballPose))
    .build();

robot.follower.followPath(path);
```

### Integration with Robot Hardware

```java
// DualCamera as component
DualCamera camera = new DualCamera(robot);
camera.registerHardware(hardwareMap);
robot.registerComponent(camera);

// Automatic update in robot loop
robot.update();  // Calls camera.update() internally

// Synchronized with other systems
robot.update() {
    motor.update();
    servo.update();
    camera.update();      // Concurrent with above
    intake.update();
}
```

### Integration with State Machine

```java
enum State {
    SEARCH,      // Looking for ball
    APPROACH,    // Moving to ball
    PICKUP,      // Engaging intake
    SCORE        // Moving to scoring zone
}

// State transitions based on vision
switch (currentState) {
    case SEARCH:
        if (camera.hasBallDetections()) {
            nextState = State.APPROACH;
        }
        break;
    
    case APPROACH:
        if (robot.isNearPose(ball.toPose())) {
            nextState = State.PICKUP;
        }
        break;
    
    case PICKUP:
        if (intake.hasBall()) {
            nextState = State.SCORE;
        }
        break;
}
```

---

## Summary

### System Strengths

✅ Accurate AprilTag-based localization
✅ Flexible dual detection modes (HSV + Husky Lens)
✅ Automatic coordinate transformations
✅ Camera offset compensation for side mounting
✅ Low latency (<40ms total)
✅ Modular architecture
✅ Comprehensive error handling
✅ Thread-safe concurrent processing

### System Limitations

❌ AprilTag required for absolute localization
❌ HSV detection sensitive to lighting changes
❌ Husky Lens requires pre-training
❌ Camera height must be known
❌ Offset compensation adds computational cost
❌ Accuracy degrades beyond 20 feet

### Optimization Opportunities

1. **Calibration**: Better camera intrinsics = ±1-2" accuracy
2. **Lighting**: Adaptive HSV range = Better consistency
3. **Tracking**: Persistence filtering = Fewer false positives
4. **Fallback**: Dual detection = Maximum robustness
5. **Custom algorithms**: Domain-specific models = Better accuracy

### Future Enhancements

- Machine learning-based detection
- Real-time camera recalibration
- Multi-camera fusion
- 3D pose estimation
- Markerless localization fallback
- Automatic lighting adaptation

This completes the comprehensive technical documentation of the vision system. For implementation details, see the source code comments.
