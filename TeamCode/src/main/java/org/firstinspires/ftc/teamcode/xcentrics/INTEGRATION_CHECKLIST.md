# Ball Pickup System - Integration Checklist

## Pre-Integration (Before You Start)

- [ ] Project compiles without errors
- [ ] Pedro Pathing is properly integrated
- [ ] Intake system is working
- [ ] Turret system is working
- [ ] AprilTag library is available
- [ ] Webcam is properly connected
- [ ] Robot runs existing autonomous OpModes

## File Integration (30 minutes)

### Step 1: Create Directory Structure

- [ ] Create folder: `xcentrics/vision/`
- [ ] Create folder: `xcentrics/util/navigation/`
- [ ] Create folder: `xcentrics/OpModes/Auto/examples/`

### Step 2: Copy Vision Files

- [ ] Copy `BallDetectionProcessor.java` → `xcentrics/vision/`
- [ ] Copy `BallTarget.java` → `xcentrics/vision/`
- [ ] Copy `DualCamera.java` → `xcentrics/components/live/`

### Step 3: Copy Navigation Files

- [ ] Copy `BallPathPlanner.java` → `xcentrics/util/navigation/`

### Step 4: Copy Autonomous Files

- [ ] Copy `BallPickupAuto.java` → `xcentrics/OpModes/Auto/`
- [ ] Copy `AdvancedBallPickupAuto.java` → `xcentrics/OpModes/Auto/`
- [ ] Copy `SimpleBallPickupExample.java` → `xcentrics/OpModes/Auto/examples/`

### Step 5: Copy Debug Files

- [ ] Copy `CameraDebugTeleOp.java` → `xcentrics/OpModes/TeleOp/`

### Step 6: Copy Documentation

- [ ] Copy `README.md` → `xcentrics/`
- [ ] Copy `QUICK_REFERENCE.md` → `xcentrics/`
- [ ] Copy `SETUP_GUIDE.md` → `xcentrics/`
- [ ] Copy `BALL_PICKUP_SYSTEM.md` → `xcentrics/`
- [ ] Copy `IMPLEMENTATION_SUMMARY.md` → `xcentrics/`
- [ ] Copy `VISUAL_GUIDE.md` → `xcentrics/`

## Compilation Check (5 minutes)

### Verify All Files Compile

- [ ] Run `gradle build`
- [ ] All files compile without errors
- [ ] No import errors
- [ ] No type mismatch warnings

### Check Available OpModes

- [ ] BallPickupAuto appears in list
- [ ] AdvancedBallPickupAuto appears in list
- [ ] SimpleBallPickupExample appears in list
- [ ] CameraDebugTeleOp appears in list

## Configuration (10 minutes)

### Update LiveRobot.java (Already Done!)

- [x] Added `public DualCamera dualCamera;` field
- [x] Added `getRobotPose()` method
- [x] Imported `DualCamera` class

### Configure Your OpModes

In each autonomous OpMode that uses DualCamera:

```java
// In on_init()
dualCamera = new DualCamera(robot);
dualCamera.registerHardware(hardwareMap);
robot.registerComponent(dualCamera);
dualCamera.setBallColorRange(15, 40, 100, 255, 100, 255);
dualCamera.setMinBallArea(150);

// In on_start()
dualCamera.startup();

// In on_loop()
camera.update(this);

// In on_stop()
dualCamera.shutdown();
```

- [ ] Updated BallPickupAuto.java
- [ ] Updated AdvancedBallPickupAuto.java
- [ ] Updated your custom OpModes (if any)

## Hardware Verification (5 minutes)

### Check Camera Hardware

- [ ] Webcam is connected to RC
- [ ] Hardware name is "Webcam 1" (or adjust in code)
- [ ] Camera image is visible on RC dashboard
- [ ] Camera has good focus on AprilTags

### Check Field Setup

- [ ] AprilTags are visible to camera
- [ ] AprilTag positioning is correct
- [ ] Lighting is adequate for detection
- [ ] Test area has good lighting

## Initial Testing (30 minutes)

### Test 1: CameraDebugTeleOp (10 minutes)

- [ ] Load CameraDebugTeleOp
- [ ] Initialize OpMode
- [ ] Press START
- [ ] Watch telemetry
- [ ] AprilTag count shows >0
- [ ] Telemetry updates without errors
- [ ] Place ball in view
- [ ] Adjust X to cycle color presets
- [ ] Adjust DPAD UP/DOWN to tune HSV
- [ ] Ball detection shows in telemetry

**Expected Result**: Can see AprilTag and ball detections

### Test 2: SimpleBallPickupExample (15 minutes)

- [ ] Load SimpleBallPickupExample
- [ ] Initialize OpMode
- [ ] Place robot with AprilTag visible
- [ ] Place 1 ball near robot
- [ ] Press START
- [ ] Watch telemetry for AprilTag lock
- [ ] Verify ball is detected
- [ ] Robot navigates to ball
- [ ] Intake activates
- [ ] Complete sequence works
- [ ] No errors in telemetry

**Expected Result**: Basic autonomous cycle completes successfully

### Test 3: Basic Autonomous (5 minutes)

- [ ] Load BallPickupAuto
- [ ] Same setup as Test 2
- [ ] Verify same behavior
- [ ] Check state machine in telemetry

**Expected Result**: State machine progresses correctly

## Performance Testing (15 minutes)

### Measure Latency

```
In telemetry:
- AprilTag detections: Should be >0 within 2 seconds
- Ball detections: Should be >0 within 3 seconds
- Update frequency: Should be >20 Hz
```

- [ ] AprilTag appears within 2 seconds
- [ ] Ball detection appears within 3 seconds
- [ ] Update frequency >20 Hz
- [ ] No lag in movement

### Measure Accuracy

```
Place ball at known distance (e.g., 12 inches forward):
- Expected fieldX: ~0
- Expected fieldY: ~12
- Actual: Check telemetry
- Error: Should be <±2 inches
```

- [ ] Test with ball 12 inches forward
- [ ] Check position accuracy
- [ ] Error within acceptable range
- [ ] Test with ball at 24 inches
- [ ] Accuracy consistent at distance

### Measure Cycle Time

- [ ] Time from start to ball detection: <5 seconds
- [ ] Time from detection to pickup: <5 seconds
- [ ] Total single-ball cycle: <10 seconds
- [ ] No unexpected delays

## Advanced Testing (30 minutes)

### Test 4: Multi-Ball Collection

- [ ] Load AdvancedBallPickupAuto
- [ ] Place 2 balls on field
- [ ] Run autonomous
- [ ] Verify optimal path planning
- [ ] Both balls collected
- [ ] Return to scoring

- [ ] Repeat with 3 balls
- [ ] System handles multiple targets
- [ ] Path optimization works
- [ ] Total cycle <25 seconds

**Expected Result**: Multi-ball collection works correctly

### Test 5: Error Conditions

- [ ] Test with AprilTag not visible
  - [ ] System handles gracefully
  - [ ] Falls back safely

- [ ] Test with no ball visible
  - [ ] Timeout works
  - [ ] Moves to next state

- [ ] Test with poor lighting
  - [ ] Detection still works
  - [ ] May need HSV adjustment

- [ ] Test with camera out of focus
  - [ ] Detection fails gracefully
  - [ ] No crashes

**Expected Result**: System handles errors gracefully

## Calibration (Optional but Recommended)

### Camera Calibration

- [ ] Measure camera mount height: _____ inches
- [ ] Place target at 12 inches
- [ ] Check detected position
- [ ] Adjust focal length if needed
- [ ] Place target at 24 inches
- [ ] Verify linearity
- [ ] Update calibration in DualCamera.java

### HSV Calibration

- [ ] Note optimal HSV for your lighting
- [ ] Record in configuration
- [ ] Test in different areas of field
- [ ] Test at different times of day
- [ ] Finalize preset values

**Optimal HSV Values:**
- H Min: _____
- H Max: _____
- S Min: _____
- S Max: _____
- V Min: _____
- V Max: _____

**Min Ball Area:** _____

## Final Verification (15 minutes)

### Functionality Checklist

- [ ] AprilTag detection: ✓ Working
- [ ] Ball detection: ✓ Working
- [ ] Coordinate conversion: ✓ Accurate
- [ ] Path planning: ✓ Optimized
- [ ] Navigation: ✓ Smooth
- [ ] Pickup execution: ✓ Reliable
- [ ] Complete cycle: ✓ Successful

### Performance Checklist

- [ ] Latency: <40 ms ✓
- [ ] Accuracy: ±2-4 inches ✓
- [ ] Update rate: >20 Hz ✓
- [ ] Single ball: <10 seconds ✓
- [ ] Multi-ball: <25 seconds ✓
- [ ] No crashes ✓
- [ ] Telemetry working ✓

### Code Quality Checklist

- [ ] No compilation errors
- [ ] No runtime exceptions
- [ ] Clean telemetry output
- [ ] Proper resource cleanup
- [ ] Graceful error handling

## Field Testing (30 minutes)

### Pre-Competition Testing

- [ ] Test on actual field dimensions
- [ ] Test with actual ball color
- [ ] Test with actual lighting
- [ ] Test with actual AprilTag positions
- [ ] Run complete autonomous cycle
- [ ] Run multiple times for consistency
- [ ] Measure and document results

### Data to Record

- **Run 1**: Time: _____ Balls: _____ Success: Yes/No
- **Run 2**: Time: _____ Balls: _____ Success: Yes/No
- **Run 3**: Time: _____ Balls: _____ Success: Yes/No
- **Run 4**: Time: _____ Balls: _____ Success: Yes/No
- **Run 5**: Time: _____ Balls: _____ Success: Yes/No

**Average Time**: _____
**Success Rate**: _____% (Should be >80%)

## Production Deployment (5 minutes)

### Before Loading to Robot

- [ ] Code is fully tested
- [ ] Configuration is finalized
- [ ] HSV values are optimized
- [ ] Camera calibration is complete
- [ ] Field positions are correct
- [ ] All documentation is available

### Final Checklist

- [ ] Disable CameraDebugTeleOp (unless needed)
- [ ] Use AdvancedBallPickupAuto for competition
- [ ] Backup configuration values somewhere
- [ ] Document any custom modifications
- [ ] Have SimpleBallPickupExample as fallback
- [ ] Print QUICK_REFERENCE.md for pit crew

## Competition Day (10 minutes)

### Pre-Match Setup

- [ ] Robot and vision system powered on
- [ ] AprilTag visible and recognized
- [ ] Camera focus checked
- [ ] Lighting conditions noted
- [ ] Test autonomous briefly
- [ ] All systems responding

### Match Execution

- [ ] Load correct OpMode (AdvancedBallPickupAuto)
- [ ] Initialize OpMode
- [ ] Wait for Driver Station signal
- [ ] Press START when ready
- [ ] Monitor telemetry during match
- [ ] Verify mission completion

### Post-Match Analysis

- [ ] Check completion status
- [ ] Note any issues
- [ ] Compare to test results
- [ ] Plan any adjustments

## Troubleshooting Checklist

If something doesn't work:

### Compilation Issues
- [ ] Check all imports are correct
- [ ] Verify file locations
- [ ] Check for typos in class names
- [ ] Rebuild project (`gradle build`)

### Runtime Issues
- [ ] Check telemetry for error messages
- [ ] Verify hardware is connected
- [ ] Run CameraDebugTeleOp first
- [ ] Check camera focus

### Vision Issues
- [ ] Run CameraDebugTeleOp
- [ ] Verify AprilTag visible
- [ ] Adjust HSV range if needed
- [ ] Check lighting conditions
- [ ] Verify camera calibration

### Navigation Issues
- [ ] Check field coordinates are correct
- [ ] Verify robot pose is accurate
- [ ] Test SimpleAutoPickup first
- [ ] Check follower is working
- [ ] Verify intake system is ready

### Performance Issues
- [ ] Check update frequency in telemetry
- [ ] Increase min ball area threshold
- [ ] Lower vision resolution if needed
- [ ] Profile CPU usage

## Final Sign-Off

- [ ] All tests passed ✓
- [ ] Performance acceptable ✓
- [ ] System is reliable ✓
- [ ] Ready for competition ✓

**Date Tested**: _______________
**Tested By**: _______________
**Notes**: _______________________________________________

---

## Quick Command Reference

### Load CameraDebugTeleOp
```
Select Autonomous/TeleOp: TeleOp
Select OpMode: Camera Debug TeleOp
```

### Load SimpleBallPickupExample
```
Select Autonomous/TeleOp: Autonomous
Select OpMode: Simple Ball Pickup Example
```

### Load BallPickupAuto
```
Select Autonomous/TeleOp: Autonomous
Select OpMode: Ball Pickup Auto
```

### Load AdvancedBallPickupAuto
```
Select Autonomous/TeleOp: Autonomous
Select OpMode: Advanced Ball Pickup Auto
```

---

**You're all set! Ready for competition! 🎉**
